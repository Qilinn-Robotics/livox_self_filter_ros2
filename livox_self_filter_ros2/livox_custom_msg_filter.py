import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class BoxFilter:
    name: str
    center: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    size: Tuple[float, float, float]


def frames_match(left: str, right: str) -> bool:
    return str(left or "").lstrip("/") == str(right or "").lstrip("/")


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def transform_to_matrix(transform_msg) -> np.ndarray:
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("TF rotation quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), float(t.x)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), float(t.y)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), float(t.z)],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    transform = np.asarray(transform, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    if len(points) == 0:
        return points.copy()
    homogeneous = np.column_stack([points, np.ones(len(points), dtype=np.float64)])
    return (homogeneous @ transform.T)[:, :3]


def points_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")

    msg = PointCloud2()
    msg.header.frame_id = str(frame_id)
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = points.astype("<f4", copy=False).tobytes()
    return msg


def parse_box_filter(value: str) -> BoxFilter:
    """Parse 'name:x,y,z,roll,pitch,yaw,sx,sy,sz' or 'name:x,y,z,sx,sy,sz'."""
    name, sep, payload = str(value).partition(":")
    if not sep:
        name = f"box_{abs(hash(value)) % 100000}"
        payload = str(value)
    parts = [float(part.strip()) for part in payload.split(",") if part.strip()]
    if len(parts) == 6:
        x, y, z, sx, sy, sz = parts
        rpy = (0.0, 0.0, 0.0)
    elif len(parts) == 9:
        x, y, z, roll, pitch, yaw, sx, sy, sz = parts
        rpy = (roll, pitch, yaw)
    else:
        raise ValueError(
            "box filter must have 6 or 9 numeric values: "
            "x,y,z,sx,sy,sz or x,y,z,roll,pitch,yaw,sx,sy,sz"
        )
    if sx <= 0.0 or sy <= 0.0 or sz <= 0.0:
        raise ValueError(f"box filter '{name}' size must be positive")
    return BoxFilter(name.strip() or "box", (x, y, z), rpy, (sx, sy, sz))


def parse_box_filters(values: Sequence[str]) -> List[BoxFilter]:
    return [parse_box_filter(value) for value in values if str(value).strip()]


def points_from_livox_msg(msg: CustomMsg) -> np.ndarray:
    points = np.empty((len(msg.points), 3), dtype=np.float64)
    for idx, point in enumerate(msg.points):
        points[idx, 0] = float(point.x)
        points[idx, 1] = float(point.y)
        points[idx, 2] = float(point.z)
    return points


def box_reject_mask(points: np.ndarray, boxes: Sequence[BoxFilter], padding: float) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    rejected = np.zeros(len(points), dtype=bool)
    pad = max(0.0, float(padding))
    for box in boxes:
        center = np.asarray(box.center, dtype=np.float64)
        half = 0.5 * np.asarray(box.size, dtype=np.float64) + pad
        rotation = rotation_matrix_from_rpy(*box.rpy)
        local = (points - center) @ rotation
        rejected |= np.all(np.abs(local) <= half, axis=1)
    return rejected


def crop_reject_mask(
    points: np.ndarray,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    keep = np.isfinite(points).all(axis=1)
    keep &= points[:, 0] >= float(x_min)
    keep &= points[:, 0] <= float(x_max)
    keep &= points[:, 1] >= float(y_min)
    keep &= points[:, 1] <= float(y_max)
    keep &= points[:, 2] >= float(z_min)
    keep &= points[:, 2] <= float(z_max)
    return ~keep


def make_filtered_msg(msg: CustomMsg, keep_mask: np.ndarray, source_frame: str) -> CustomMsg:
    out = CustomMsg()
    out.header = msg.header
    out.header.frame_id = source_frame
    out.timebase = msg.timebase
    out.lidar_id = msg.lidar_id
    out.rsvd = msg.rsvd
    out.points = [msg.points[idx] for idx in np.flatnonzero(keep_mask)]
    out.point_num = len(out.points)
    return out


class LivoxCustomMsgFilter(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_msg_filter")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_filtered")
        self.declare_parameter("filter_frame", "base_link")
        self.declare_parameter("source_frame_override", "")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("tf_timeout", 0.05)
        self.declare_parameter("assume_same_frame_if_tf_missing", False)
        self.declare_parameter("box_filters", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("box_padding", 0.03)
        self.declare_parameter("front_crop_enabled", False)
        self.declare_parameter("front_x_min", -0.5)
        self.declare_parameter("front_x_max", 8.0)
        self.declare_parameter("front_y_min", -3.0)
        self.declare_parameter("front_y_max", 3.0)
        self.declare_parameter("front_z_min", -2.0)
        self.declare_parameter("front_z_max", 3.0)
        self.declare_parameter("publish_debug_clouds", False)
        self.declare_parameter("debug_filtered_topic", "/livox/points_filtered")
        self.declare_parameter("debug_rejected_topic", "/livox/points_rejected")
        self.declare_parameter("debug_max_points", 200000)
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("stats_log_period", 2.0)

        self.input_topic = self._string_param("input_topic")
        self.output_topic = self._string_param("output_topic")
        self.filter_frame = self._string_param("filter_frame")
        self.source_frame_override = self._string_param("source_frame_override")
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.assume_same_frame_if_tf_missing = bool(
            self.get_parameter("assume_same_frame_if_tf_missing").value
        )
        self.boxes = parse_box_filters(
            self.get_parameter("box_filters").get_parameter_value().string_array_value
        )
        self.box_padding = float(self.get_parameter("box_padding").value)
        self.front_crop_enabled = bool(self.get_parameter("front_crop_enabled").value)
        self.crop_bounds = (
            float(self.get_parameter("front_x_min").value),
            float(self.get_parameter("front_x_max").value),
            float(self.get_parameter("front_y_min").value),
            float(self.get_parameter("front_y_max").value),
            float(self.get_parameter("front_z_min").value),
            float(self.get_parameter("front_z_max").value),
        )
        self.publish_debug_clouds = bool(self.get_parameter("publish_debug_clouds").value)
        self.debug_max_points = int(self.get_parameter("debug_max_points").value)
        self.qos_depth = max(1, int(self.get_parameter("qos_depth").value))
        self.stats_log_period = float(self.get_parameter("stats_log_period").value)
        self.last_stats_log_time = self.get_clock().now()
        self.last_tf_warning = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud_qos = QoSProfile(depth=self.qos_depth)
        self.publisher = self.create_publisher(CustomMsg, self.output_topic, self.cloud_qos)
        self.subscription = self.create_subscription(
            CustomMsg, self.input_topic, self._on_msg, self.cloud_qos
        )
        qos = QoSProfile(depth=1)
        self.filtered_debug_publisher = None
        self.rejected_debug_publisher = None
        if self.publish_debug_clouds:
            self.filtered_debug_publisher = self.create_publisher(
                PointCloud2, self._string_param("debug_filtered_topic"), qos
            )
            self.rejected_debug_publisher = self.create_publisher(
                PointCloud2, self._string_param("debug_rejected_topic"), qos
            )

        box_names = ", ".join(box.name for box in self.boxes) or "(none)"
        self.get_logger().info(
            "Livox CustomMsg filter ready: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"filter_frame={self.filter_frame}, boxes={box_names}, "
            f"box_padding={self.box_padding:.3f}, "
            f"qos_depth={self.qos_depth}, "
            f"debug_clouds={self.publish_debug_clouds}, "
            f"front_crop_enabled={self.front_crop_enabled}"
        )

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _source_frame(self, msg: CustomMsg) -> str:
        return self.source_frame_override or msg.header.frame_id or self.filter_frame

    def _points_in_filter_frame(
        self, source_points: np.ndarray, source_frame: str, msg: CustomMsg
    ) -> Optional[np.ndarray]:
        if frames_match(source_frame, self.filter_frame):
            return source_points
        try:
            lookup_time = Time() if self.use_latest_tf else Time.from_msg(msg.header.stamp)
            transform_msg = self.tf_buffer.lookup_transform(
                self.filter_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=max(0.0, self.tf_timeout)),
            )
            return transform_points(source_points, transform_to_matrix(transform_msg))
        except TransformException as exc:
            warning = f"{source_frame}->{self.filter_frame}: {exc}"
            if warning != self.last_tf_warning:
                self.get_logger().warn(f"livox self-filter missing TF, passing raw cloud: {warning}")
                self.last_tf_warning = warning
            if self.assume_same_frame_if_tf_missing:
                return source_points
            return None

    def _sample_debug(self, points: np.ndarray) -> np.ndarray:
        if self.debug_max_points > 0 and len(points) > self.debug_max_points:
            step = int(math.ceil(len(points) / self.debug_max_points))
            return points[::step]
        return points

    def _publish_debug(self, kept: np.ndarray, rejected: np.ndarray, stamp) -> None:
        if not self.publish_debug_clouds:
            return
        if self.filtered_debug_publisher is not None:
            self.filtered_debug_publisher.publish(
                points_to_pointcloud2(
                    self._sample_debug(kept).astype(np.float32),
                    self.filter_frame,
                    stamp,
                )
            )
        if self.rejected_debug_publisher is not None:
            self.rejected_debug_publisher.publish(
                points_to_pointcloud2(
                    self._sample_debug(rejected).astype(np.float32),
                    self.filter_frame,
                    stamp,
                )
            )

    def _maybe_log_stats(
        self,
        total: int,
        kept: int,
        rejected_self: int,
        rejected_crop: int,
        input_age_ms: Optional[float],
        process_ms: float,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_stats_log_time).nanoseconds * 1.0e-9 < self.stats_log_period:
            return
        self.last_stats_log_time = now
        ratio = 100.0 * kept / max(total, 1)
        age_text = "unknown" if input_age_ms is None else f"{input_age_ms:.1f}ms"
        self.get_logger().info(
            f"livox filter stats: input={total}, kept={kept} ({ratio:.1f}%), "
            f"self_rejected={rejected_self}, crop_rejected={rejected_crop}, "
            f"stamp_age={age_text}, process={process_ms:.1f}ms"
        )

    def _stamp_age_ms(self, stamp) -> Optional[float]:
        if stamp is None:
            return None
        if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
            return None
        return (
            self.get_clock().now() - Time.from_msg(stamp)
        ).nanoseconds * 1.0e-6

    def _on_msg(self, msg: CustomMsg) -> None:
        callback_start = self.get_clock().now()
        input_age_ms = self._stamp_age_ms(msg.header.stamp)
        source_frame = self._source_frame(msg)
        source_points = points_from_livox_msg(msg)
        filter_points = self._points_in_filter_frame(source_points, source_frame, msg)
        if filter_points is None:
            out = make_filtered_msg(
                msg,
                np.ones(len(msg.points), dtype=bool),
                source_frame,
            )
            self.publisher.publish(out)
            return

        self_reject = box_reject_mask(filter_points, self.boxes, self.box_padding)
        crop_reject = np.zeros(len(filter_points), dtype=bool)
        if self.front_crop_enabled:
            crop_reject = crop_reject_mask(filter_points, *self.crop_bounds)
        keep_mask = ~(self_reject | crop_reject)

        out = make_filtered_msg(msg, keep_mask, source_frame)
        self.publisher.publish(out)
        self._publish_debug(
            filter_points[keep_mask],
            filter_points[~keep_mask],
            msg.header.stamp,
        )
        process_ms = (self.get_clock().now() - callback_start).nanoseconds * 1.0e-6
        self._maybe_log_stats(
            len(filter_points),
            int(np.count_nonzero(keep_mask)),
            int(np.count_nonzero(self_reject)),
            int(np.count_nonzero(crop_reject & ~self_reject)),
            input_age_ms,
            process_ms,
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LivoxCustomMsgFilter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
