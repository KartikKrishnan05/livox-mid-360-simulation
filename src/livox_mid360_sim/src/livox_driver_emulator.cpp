// livox_driver_emulator: makes the simulation's output look like livox_ros_driver2 on a
// real Mid-360.
//
//   /livox/points   (PointCloud2, 26-byte Livox layout, from gz)  -> /livox/lidar
//       xfer_format 1 (default): livox_ros_driver2/msg/CustomMsg
//       xfer_format 0          : PointCloud2 in the driver's layout (pass-through)
//   /livox/imu_raw  (sensor_msgs/Imu, m/s^2, from gz)              -> /livox/imu (in g)
//
// Lossless: every input point carries its own line and absolute timestamp.
#include <algorithm>
#include <cstring>
#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
constexpr double kGravity = 9.80665;   // the driver reports acceleration in g

struct FieldOffsets
{
  int x{-1}, y{-1}, z{-1}, intensity{-1}, tag{-1}, line{-1}, timestamp{-1};
};

FieldOffsets FindFields(const sensor_msgs::msg::PointCloud2 & pc)
{
  using PF = sensor_msgs::msg::PointField;
  FieldOffsets o;
  for (const auto & f : pc.fields) {
    const int off = static_cast<int>(f.offset);
    if (f.name == "x" && f.datatype == PF::FLOAT32) {o.x = off;}
    if (f.name == "y" && f.datatype == PF::FLOAT32) {o.y = off;}
    if (f.name == "z" && f.datatype == PF::FLOAT32) {o.z = off;}
    if (f.name == "intensity" && f.datatype == PF::FLOAT32) {o.intensity = off;}
    if (f.name == "tag" && f.datatype == PF::UINT8) {o.tag = off;}
    if (f.name == "line" && f.datatype == PF::UINT8) {o.line = off;}
    if (f.name == "timestamp" && f.datatype == PF::FLOAT64) {o.timestamp = off;}
  }
  return o;
}

template<typename T>
T Read(const uint8_t * p, int off)
{
  T v;
  std::memcpy(&v, p + off, sizeof(T));
  return v;
}
}  // namespace

class LivoxDriverEmulator : public rclcpp::Node
{
public:
  LivoxDriverEmulator()
  : Node("livox_driver_emulator")
  {
    xfer_format_ = declare_parameter<int>("xfer_format", 1);
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    lidar_id_ = declare_parameter<int>("lidar_id", 0);
    const auto points_in = declare_parameter<std::string>("points_in", "/livox/points");
    const auto imu_in = declare_parameter<std::string>("imu_in", "/livox/imu_raw");
    const auto lidar_out = declare_parameter<std::string>("lidar_out", "/livox/lidar");
    const auto imu_out = declare_parameter<std::string>("imu_out", "/livox/imu");

    if (xfer_format_ != 0 && xfer_format_ != 1) {
      RCLCPP_WARN(get_logger(), "xfer_format %d not supported (0 or 1); using 1", xfer_format_);
      xfer_format_ = 1;
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto out_qos = rclcpp::QoS(10);
    if (xfer_format_ == 1) {
      custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(lidar_out, out_qos);
    } else {
      pc2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_out, out_qos);
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_out, out_qos);

    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_in, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {OnPoints(std::move(msg));});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in, sensor_qos,
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {OnImu(std::move(msg));});

    RCLCPP_INFO(
      get_logger(), "%s -> %s (%s), %s -> %s (g), frame '%s'", points_in.c_str(),
      lidar_out.c_str(), xfer_format_ == 1 ? "CustomMsg" : "PointCloud2", imu_in.c_str(),
      imu_out.c_str(), frame_id_.c_str());
  }

private:
  void OnPoints(sensor_msgs::msg::PointCloud2::UniquePtr in)
  {
    const auto o = FindFields(*in);
    if (o.x < 0 || o.y < 0 || o.z < 0 || o.timestamp < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "input cloud lacks x/y/z/timestamp fields in the Livox layout; dropping");
      return;
    }
    const std::size_t n = static_cast<std::size_t>(in->width) * in->height;
    const uint8_t * data = in->data.data();
    const std::size_t step = in->point_step;

    // timebase = time of the first point; the header carries the same instant.
    uint64_t timebase = static_cast<uint64_t>(in->header.stamp.sec) * 1000000000ULL +
      in->header.stamp.nanosec;
    if (n > 0) {
      timebase = static_cast<uint64_t>(Read<double>(data, o.timestamp));
    }
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(timebase / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(timebase % 1000000000ULL);

    if (xfer_format_ == 0) {
      in->header.stamp = stamp;
      in->header.frame_id = frame_id_;
      pc2_pub_->publish(std::move(in));
      return;
    }

    auto out = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
    out->header.stamp = stamp;
    out->header.frame_id = frame_id_;
    out->timebase = timebase;
    out->point_num = static_cast<uint32_t>(n);
    out->lidar_id = static_cast<uint8_t>(lidar_id_);
    out->rsvd = {0, 0, 0};
    out->points.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      const uint8_t * p = data + i * step;
      auto & q = out->points[i];
      const double ts = Read<double>(p, o.timestamp);
      q.offset_time = static_cast<uint32_t>(static_cast<uint64_t>(ts) - timebase);
      q.x = Read<float>(p, o.x);
      q.y = Read<float>(p, o.y);
      q.z = Read<float>(p, o.z);
      q.reflectivity = o.intensity >= 0 ?
        static_cast<uint8_t>(std::clamp(Read<float>(p, o.intensity), 0.f, 255.f)) : 0;
      q.tag = o.tag >= 0 ? Read<uint8_t>(p, o.tag) : 0;
      q.line = o.line >= 0 ? Read<uint8_t>(p, o.line) : 0;
    }
    custom_pub_->publish(std::move(out));
  }

  void OnImu(sensor_msgs::msg::Imu::UniquePtr in)
  {
    // Like the real driver: angular velocity in rad/s, linear acceleration in g,
    // no orientation estimate, no covariances, frame livox_frame.
    auto out = std::make_unique<sensor_msgs::msg::Imu>();
    out->header.stamp = in->header.stamp;
    out->header.frame_id = frame_id_;
    out->angular_velocity = in->angular_velocity;
    out->linear_acceleration.x = in->linear_acceleration.x / kGravity;
    out->linear_acceleration.y = in->linear_acceleration.y / kGravity;
    out->linear_acceleration.z = in->linear_acceleration.z / kGravity;
    imu_pub_->publish(std::move(out));
  }

  int xfer_format_{1};
  int lidar_id_{0};
  std::string frame_id_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxDriverEmulator>());
  rclcpp::shutdown();
  return 0;
}
