#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace champ_livox_self_filter
{
using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3
{
  double v[3][3]{};
};

struct Mat4
{
  double v[4][4]{};
};

struct BoxFilter
{
  std::string name;
  Vec3 center;
  Vec3 rpy;
  Vec3 size;
};

static std::string trim(const std::string & text)
{
  const auto start = text.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(start, end - start + 1);
}

static std::vector<double> split_doubles(const std::string & text)
{
  std::vector<double> values;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = trim(token);
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}

static BoxFilter parse_box_filter(const std::string & value)
{
  auto separator = value.find(':');
  std::string name = "box";
  std::string payload = value;
  if (separator != std::string::npos) {
    name = trim(value.substr(0, separator));
    payload = value.substr(separator + 1);
  }
  if (name.empty()) {
    name = "box";
  }

  const auto parts = split_doubles(payload);
  BoxFilter box;
  box.name = name;
  if (parts.size() == 6) {
    box.center = {parts[0], parts[1], parts[2]};
    box.rpy = {0.0, 0.0, 0.0};
    box.size = {parts[3], parts[4], parts[5]};
  } else if (parts.size() == 9) {
    box.center = {parts[0], parts[1], parts[2]};
    box.rpy = {parts[3], parts[4], parts[5]};
    box.size = {parts[6], parts[7], parts[8]};
  } else {
    throw std::runtime_error(
      "box filter must have 6 or 9 numeric values: "
      "x,y,z,sx,sy,sz or x,y,z,roll,pitch,yaw,sx,sy,sz");
  }
  if (box.size.x <= 0.0 || box.size.y <= 0.0 || box.size.z <= 0.0) {
    throw std::runtime_error("box filter '" + box.name + "' size must be positive");
  }
  return box;
}

static bool frames_match(const std::string & left, const std::string & right)
{
  auto normalize = [](const std::string & frame) {
    if (!frame.empty() && frame.front() == '/') {
      return frame.substr(1);
    }
    return frame;
  };
  return normalize(left) == normalize(right);
}

static Mat3 rotation_matrix_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  Mat3 m;
  m.v[0][0] = cy * cp;
  m.v[0][1] = cy * sp * sr - sy * cr;
  m.v[0][2] = cy * sp * cr + sy * sr;
  m.v[1][0] = sy * cp;
  m.v[1][1] = sy * sp * sr + cy * cr;
  m.v[1][2] = sy * sp * cr - cy * sr;
  m.v[2][0] = -sp;
  m.v[2][1] = cp * sr;
  m.v[2][2] = cp * cr;
  return m;
}

static Mat4 transform_to_matrix(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm <= 0.0) {
    throw std::runtime_error("TF rotation quaternion has zero norm");
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  Mat4 m;
  m.v[0][0] = 1.0 - 2.0 * (y * y + z * z);
  m.v[0][1] = 2.0 * (x * y - z * w);
  m.v[0][2] = 2.0 * (x * z + y * w);
  m.v[0][3] = t.x;
  m.v[1][0] = 2.0 * (x * y + z * w);
  m.v[1][1] = 1.0 - 2.0 * (x * x + z * z);
  m.v[1][2] = 2.0 * (y * z - x * w);
  m.v[1][3] = t.y;
  m.v[2][0] = 2.0 * (x * z - y * w);
  m.v[2][1] = 2.0 * (y * z + x * w);
  m.v[2][2] = 1.0 - 2.0 * (x * x + y * y);
  m.v[2][3] = t.z;
  m.v[3][0] = 0.0;
  m.v[3][1] = 0.0;
  m.v[3][2] = 0.0;
  m.v[3][3] = 1.0;
  return m;
}

static Vec3 transform_point(const Vec3 & point, const Mat4 & transform)
{
  return {
    transform.v[0][0] * point.x + transform.v[0][1] * point.y +
      transform.v[0][2] * point.z + transform.v[0][3],
    transform.v[1][0] * point.x + transform.v[1][1] * point.y +
      transform.v[1][2] * point.z + transform.v[1][3],
    transform.v[2][0] * point.x + transform.v[2][1] * point.y +
      transform.v[2][2] * point.z + transform.v[2][3],
  };
}

static bool point_in_box(const Vec3 & point, const BoxFilter & box, double padding)
{
  const auto rotation = rotation_matrix_from_rpy(box.rpy.x, box.rpy.y, box.rpy.z);
  const double dx = point.x - box.center.x;
  const double dy = point.y - box.center.y;
  const double dz = point.z - box.center.z;

  // Matches the Python implementation: local = (point - center) @ rotation.
  const double lx = dx * rotation.v[0][0] + dy * rotation.v[1][0] + dz * rotation.v[2][0];
  const double ly = dx * rotation.v[0][1] + dy * rotation.v[1][1] + dz * rotation.v[2][1];
  const double lz = dx * rotation.v[0][2] + dy * rotation.v[1][2] + dz * rotation.v[2][2];

  return std::abs(lx) <= 0.5 * box.size.x + padding &&
         std::abs(ly) <= 0.5 * box.size.y + padding &&
         std::abs(lz) <= 0.5 * box.size.z + padding;
}

class LivoxCustomMsgFilter : public rclcpp::Node
{
public:
  LivoxCustomMsgFilter()
  : Node("livox_custom_msg_filter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    filter_frame_ = declare_parameter<std::string>("filter_frame", "base_link");
    source_frame_override_ = declare_parameter<std::string>("source_frame_override", "");
    use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);
    assume_same_frame_if_tf_missing_ =
      declare_parameter<bool>("assume_same_frame_if_tf_missing", false);
    box_padding_ = std::max(0.0, declare_parameter<double>("box_padding", 0.03));
    front_crop_enabled_ = declare_parameter<bool>("front_crop_enabled", false);
    front_x_min_ = declare_parameter<double>("front_x_min", -0.5);
    front_x_max_ = declare_parameter<double>("front_x_max", 8.0);
    front_y_min_ = declare_parameter<double>("front_y_min", -3.0);
    front_y_max_ = declare_parameter<double>("front_y_max", 3.0);
    front_z_min_ = declare_parameter<double>("front_z_min", -2.0);
    front_z_max_ = declare_parameter<double>("front_z_max", 3.0);
    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", false);
    debug_filtered_topic_ =
      declare_parameter<std::string>("debug_filtered_topic", "/livox/points_filtered");
    debug_rejected_topic_ =
      declare_parameter<std::string>("debug_rejected_topic", "/livox/points_rejected");
    debug_max_points_ = declare_parameter<int>("debug_max_points", 200000);
    qos_depth_ = std::max(1, static_cast<int>(declare_parameter<int>("qos_depth", 1)));
    stats_log_period_ = declare_parameter<double>("stats_log_period", 2.0);
    last_stats_log_time_ = now();

    const auto box_values =
      declare_parameter<std::vector<std::string>>("box_filters", std::vector<std::string>{});
    boxes_.reserve(box_values.size());
    for (const auto & value : box_values) {
      if (!trim(value).empty()) {
        boxes_.push_back(parse_box_filter(value));
      }
    }

    const rclcpp::QoS cloud_qos{rclcpp::KeepLast(qos_depth_)};
    publisher_ = create_publisher<CustomMsg>(output_topic_, cloud_qos);
    subscription_ = create_subscription<CustomMsg>(
      input_topic_, cloud_qos, std::bind(&LivoxCustomMsgFilter::on_msg, this, std::placeholders::_1));

    const rclcpp::QoS debug_qos(rclcpp::KeepLast(1));
    if (publish_debug_clouds_) {
      filtered_debug_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(debug_filtered_topic_, debug_qos);
      rejected_debug_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(debug_rejected_topic_, debug_qos);
    }

    std::ostringstream names;
    for (std::size_t i = 0; i < boxes_.size(); ++i) {
      if (i > 0) {
        names << ", ";
      }
      names << boxes_[i].name;
    }
    RCLCPP_INFO(
      get_logger(),
      "Livox CustomMsg C++ filter ready: %s -> %s, filter_frame=%s, boxes=%s, "
      "box_padding=%.3f, qos_depth=%d, debug_clouds=%s, front_crop_enabled=%s",
      input_topic_.c_str(), output_topic_.c_str(), filter_frame_.c_str(),
      boxes_.empty() ? "(none)" : names.str().c_str(), box_padding_, qos_depth_,
      publish_debug_clouds_ ? "true" : "false", front_crop_enabled_ ? "true" : "false");
  }

private:
  std::string source_frame(const CustomMsg & msg) const
  {
    if (!source_frame_override_.empty()) {
      return source_frame_override_;
    }
    if (!msg.header.frame_id.empty()) {
      return msg.header.frame_id;
    }
    return filter_frame_;
  }

  bool get_transform_matrix(
    const std::string & src_frame,
    const CustomMsg & msg,
    Mat4 * matrix)
  {
    if (frames_match(src_frame, filter_frame_)) {
      *matrix = Mat4{};
      for (int i = 0; i < 4; ++i) {
        matrix->v[i][i] = 1.0;
      }
      return true;
    }

    try {
      const rclcpp::Time lookup_time =
        use_latest_tf_ ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) :
        rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
      const auto transform = tf_buffer_.lookupTransform(
        filter_frame_, src_frame, lookup_time, rclcpp::Duration::from_seconds(std::max(0.0, tf_timeout_)));
      *matrix = transform_to_matrix(transform);
      return true;
    } catch (const std::exception & exc) {
      const std::string warning = src_frame + "->" + filter_frame_ + ": " + exc.what();
      if (warning != last_tf_warning_) {
        RCLCPP_WARN(get_logger(), "livox self-filter missing TF, passing raw cloud: %s", warning.c_str());
        last_tf_warning_ = warning;
      }
      if (assume_same_frame_if_tf_missing_) {
        *matrix = Mat4{};
        for (int i = 0; i < 4; ++i) {
          matrix->v[i][i] = 1.0;
        }
        return true;
      }
      return false;
    }
  }

  bool reject_by_crop(const Vec3 & point) const
  {
    if (!front_crop_enabled_) {
      return false;
    }
    return !(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           point.x >= front_x_min_ && point.x <= front_x_max_ &&
           point.y >= front_y_min_ && point.y <= front_y_max_ &&
           point.z >= front_z_min_ && point.z <= front_z_max_);
  }

  bool reject_by_self_boxes(const Vec3 & point) const
  {
    if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))) {
      return false;
    }
    for (const auto & box : boxes_) {
      if (point_in_box(point, box, box_padding_)) {
        return true;
      }
    }
    return false;
  }

  double stamp_age_ms(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto age = now() - rclcpp::Time(stamp, get_clock()->get_clock_type());
    return age.seconds() * 1000.0;
  }

  void maybe_log_stats(
    std::size_t total,
    std::size_t kept,
    std::size_t rejected_self,
    std::size_t rejected_crop,
    double input_age_ms,
    double process_ms)
  {
    const auto now_time = now();
    if ((now_time - last_stats_log_time_).seconds() < stats_log_period_) {
      return;
    }
    last_stats_log_time_ = now_time;
    const double ratio = 100.0 * static_cast<double>(kept) / std::max<std::size_t>(total, 1);
    if (std::isfinite(input_age_ms)) {
      RCLCPP_INFO(
        get_logger(),
        "livox filter stats: input=%zu, kept=%zu (%.1f%%), self_rejected=%zu, "
        "crop_rejected=%zu, stamp_age=%.1fms, process=%.1fms",
        total, kept, ratio, rejected_self, rejected_crop, input_age_ms, process_ms);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "livox filter stats: input=%zu, kept=%zu (%.1f%%), self_rejected=%zu, "
        "crop_rejected=%zu, stamp_age=unknown, process=%.1fms",
        total, kept, ratio, rejected_self, rejected_crop, process_ms);
    }
  }

  sensor_msgs::msg::PointCloud2 points_to_cloud(
    const std::vector<Vec3> & points,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = filter_frame_;
    msg.header.stamp = stamp;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;
    msg.data.resize(static_cast<std::size_t>(msg.row_step));
    for (std::size_t i = 0; i < points.size(); ++i) {
      const float x = static_cast<float>(points[i].x);
      const float y = static_cast<float>(points[i].y);
      const float z = static_cast<float>(points[i].z);
      std::memcpy(&msg.data[i * 12 + 0], &x, sizeof(float));
      std::memcpy(&msg.data[i * 12 + 4], &y, sizeof(float));
      std::memcpy(&msg.data[i * 12 + 8], &z, sizeof(float));
    }
    return msg;
  }

  std::vector<Vec3> sample_debug_points(const std::vector<Vec3> & points) const
  {
    if (debug_max_points_ <= 0 || points.size() <= static_cast<std::size_t>(debug_max_points_)) {
      return points;
    }
    const auto step = static_cast<std::size_t>(
      std::ceil(static_cast<double>(points.size()) / static_cast<double>(debug_max_points_)));
    std::vector<Vec3> sampled;
    sampled.reserve(static_cast<std::size_t>(debug_max_points_));
    for (std::size_t i = 0; i < points.size(); i += step) {
      sampled.push_back(points[i]);
    }
    return sampled;
  }

  void publish_debug(
    const std::vector<Vec3> & kept_points,
    const std::vector<Vec3> & rejected_points,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_debug_clouds_) {
      return;
    }
    if (filtered_debug_publisher_) {
      filtered_debug_publisher_->publish(points_to_cloud(sample_debug_points(kept_points), stamp));
    }
    if (rejected_debug_publisher_) {
      rejected_debug_publisher_->publish(points_to_cloud(sample_debug_points(rejected_points), stamp));
    }
  }

  void publish_passthrough(const CustomMsg::SharedPtr msg, const std::string & src_frame)
  {
    auto out = *msg;
    out.header.frame_id = src_frame;
    publisher_->publish(out);
  }

  void on_msg(const CustomMsg::SharedPtr msg)
  {
    const auto callback_start = now();
    const double input_age_ms = stamp_age_ms(msg->header.stamp);
    const std::string src_frame = source_frame(*msg);
    Mat4 transform;
    if (!get_transform_matrix(src_frame, *msg, &transform)) {
      publish_passthrough(msg, src_frame);
      return;
    }

    CustomMsg out;
    out.header = msg->header;
    out.header.frame_id = src_frame;
    out.timebase = msg->timebase;
    out.lidar_id = msg->lidar_id;
    out.rsvd = msg->rsvd;
    out.points.reserve(msg->points.size());

    std::vector<Vec3> kept_debug;
    std::vector<Vec3> rejected_debug;
    if (publish_debug_clouds_) {
      kept_debug.reserve(msg->points.size());
      rejected_debug.reserve(msg->points.size() / 4);
    }

    std::size_t rejected_self = 0;
    std::size_t rejected_crop = 0;
    for (const auto & point : msg->points) {
      const Vec3 source_point{point.x, point.y, point.z};
      const Vec3 filter_point = transform_point(source_point, transform);
      const bool self_reject = reject_by_self_boxes(filter_point);
      const bool crop_reject = !self_reject && reject_by_crop(filter_point);
      if (self_reject) {
        ++rejected_self;
        if (publish_debug_clouds_) {
          rejected_debug.push_back(filter_point);
        }
        continue;
      }
      if (crop_reject) {
        ++rejected_crop;
        if (publish_debug_clouds_) {
          rejected_debug.push_back(filter_point);
        }
        continue;
      }
      out.points.push_back(point);
      if (publish_debug_clouds_) {
        kept_debug.push_back(filter_point);
      }
    }

    out.point_num = static_cast<uint32_t>(out.points.size());
    publisher_->publish(out);
    publish_debug(kept_debug, rejected_debug, msg->header.stamp);

    const double process_ms = (now() - callback_start).seconds() * 1000.0;
    maybe_log_stats(
      msg->points.size(), out.points.size(), rejected_self, rejected_crop, input_age_ms, process_ms);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string filter_frame_;
  std::string source_frame_override_;
  std::string debug_filtered_topic_;
  std::string debug_rejected_topic_;
  bool use_latest_tf_{true};
  bool assume_same_frame_if_tf_missing_{false};
  bool front_crop_enabled_{false};
  bool publish_debug_clouds_{false};
  double tf_timeout_{0.05};
  double box_padding_{0.03};
  double front_x_min_{-0.5};
  double front_x_max_{8.0};
  double front_y_min_{-3.0};
  double front_y_max_{3.0};
  double front_z_min_{-2.0};
  double front_z_max_{3.0};
  double stats_log_period_{2.0};
  int debug_max_points_{200000};
  int qos_depth_{1};
  std::vector<BoxFilter> boxes_;
  std::string last_tf_warning_;
  rclcpp::Time last_stats_log_time_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_debug_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_debug_publisher_;
};

}  // namespace champ_livox_self_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<champ_livox_self_filter::LivoxCustomMsgFilter>());
  rclcpp::shutdown();
  return 0;
}
