# livox_self_filter_ros2

A lightweight ROS 2 package for filtering robot-body and tool/implement returns
from Livox `CustomMsg` point clouds before feeding them to LiDAR odometry or
mapping pipelines such as FAST-LIO.

The node is intentionally small and standalone. It does not depend on CHAMP,
FAST-LIO internals, or a specific robot driver. Robot geometry is configured
with YAML boxes.

```text
Livox driver -> /livox/lidar -> livox_custom_msg_filter -> /livox/lidar_filtered -> FAST-LIO
```

## Why

When a Livox sensor sees the robot chassis, brackets, tools, or implements, the
moving self-points can pollute the accumulated map and confuse downstream
planners. This package removes those points in the robot frame before the point
cloud reaches the odometry or mapping stack.

It is designed to run on the robot computer, close to the Livox driver, so the
filtered cloud does not need to round-trip through another machine.

## Features

- Subscribes to `livox_ros_driver2/msg/CustomMsg`
- Publishes filtered `livox_ros_driver2/msg/CustomMsg`
- Uses TF to transform points into a configurable filter frame, usually
  `base_link`
- Removes points inside configurable 3D boxes
- Supports optional front-region crop
- Optional debug `PointCloud2` topics for RViz
- Includes a launch file, real-robot example config, and RViz config

## Dependencies

- ROS 2 Humble
- `livox_ros_driver2`
- `rclpy`
- `tf2_ros`
- `sensor_msgs`
- `numpy`

## Installation

Create a small robot-side workspace:

```bash
mkdir -p ~/livox_filter_ws/src
cd ~/livox_filter_ws/src
git clone https://github.com/Qilinn-Robotics/livox_self_filter_ros2.git

cd ~/livox_filter_ws
source /opt/ros/humble/setup.bash
source ~/livox-ws/install/setup.bash
colcon build --symlink-install --packages-select livox_self_filter_ros2
source install/setup.bash
```

## Quick Start

Start the filter:

```bash
ros2 launch livox_self_filter_ros2 real_livox_self_filter.launch.py
```

Or use the helper script from the package source folder:

```bash
./start_livox_self_filter.sh
```

By default, only the filtered Livox `CustomMsg` is published:

```text
/livox/lidar_filtered
```

Debug point clouds are disabled by default to keep the real-time path light.
Enable RViz and debug clouds only when tuning:

```bash
ros2 launch livox_self_filter_ros2 real_livox_self_filter.launch.py \
  use_rviz:=true \
  publish_debug_clouds:=true
```

or:

```bash
./start_livox_self_filter.sh --vis
```

Debug topics:

```text
/livox/points_filtered
/livox/points_rejected
```

## FAST-LIO Integration

Set FAST-LIO to subscribe to the filtered Livox topic:

```yaml
lidar_topic: /livox/lidar_filtered
imu_topic: /livox/imu
```

A typical robot-side startup order is:

```bash
# Terminal 1: Livox driver
source /opt/ros/humble/setup.bash
source ~/livox-ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# Terminal 2: self filter
source /opt/ros/humble/setup.bash
source ~/livox-ws/install/setup.bash
source ~/livox_filter_ws/install/setup.bash
ros2 launch livox_self_filter_ros2 real_livox_self_filter.launch.py

# Terminal 3: FAST-LIO
source /opt/ros/humble/setup.bash
source ~/livox-ws/install/setup.bash
source ~/fastlio-ws/install/setup.bash
ros2 launch fastlio2 lio_launch.py
```

For multi-machine ROS 2 setups, remember:

```bash
export ROS_DOMAIN_ID=65
export ROS_LOCALHOST_ONLY=0
```

## Configuration

Main config:

```text
config/real_livox_self_filter.yaml
```

Important parameters:

```yaml
input_topic: /livox/lidar
output_topic: /livox/lidar_filtered

filter_frame: base_link
source_frame_override: livox_mid360_link
use_latest_tf: true

box_filters:
  - "base_body:0.0,0.0,-0.10,0.0,0.0,0.0,0.50,0.35,0.20"
  - "tool_box:-0.20,0.30,0.40,0.0,0.0,0.0,0.04,0.40,0.04"
box_padding: 0.08
```

Box format:

```text
name:x,y,z,roll,pitch,yaw,size_x,size_y,size_z
```

All values are in meters and radians. The box pose is expressed in
`filter_frame`.

`box_padding` expands each box equally in all directions. It is useful for
near-field edge returns and small measurement errors.

## Static LiDAR TF

The launch file can publish a static transform from `base_link` to the Livox
frame:

```bash
ros2 launch livox_self_filter_ros2 real_livox_self_filter.launch.py \
  lidar_x:=0.21 \
  lidar_y:=0.0 \
  lidar_z:=0.13 \
  lidar_yaw:=1.5708
```

If your robot already publishes this TF, disable the launcher's static TF:

```bash
ros2 launch livox_self_filter_ros2 real_livox_self_filter.launch.py \
  publish_lidar_static_tf:=false
```

## Sanity Checks

Check that the filter is running:

```bash
ros2 topic hz /livox/lidar_filtered
ros2 topic delay /livox/lidar_filtered
ros2 topic info /livox/lidar_filtered
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo base_link livox_mid360_link
```

With debug visualization enabled:

```bash
ros2 topic hz /livox/points_rejected
```

## Notes

- The filtered `CustomMsg` keeps the original Livox timestamp and per-point
  fields.
- If the required TF is missing, the node passes through the raw cloud only when
  `assume_same_frame_if_tf_missing` is set to `true`.
- For real-time odometry, keep `publish_debug_clouds` disabled.

## License

MIT
