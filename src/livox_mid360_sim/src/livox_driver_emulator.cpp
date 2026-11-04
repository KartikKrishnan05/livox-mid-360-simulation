// Copyright 2026 Kartik Krishnan
// SPDX-License-Identifier: Apache-2.0
//
// Makes the simulated Mid-360 look like livox_ros_driver2 on the ROS side.
//
//   /livox/points  (PointCloud2, driver layout x y z intensity tag line timestamp)
//     -> /livox/lidar  livox_ros_driver2/msg/CustomMsg  (xfer_format 1, default)
//                      or the PointCloud2 unchanged     (xfer_format 0)
//   /livox/imu_raw (Imu, m/s^2, from Gazebo)
//     -> /livox/imu   Imu with linear acceleration in g, like the real driver
//
// The conversion is lossless: every point already carries its own line, tag
// and absolute timestamp from the Gazebo plugin.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace livox_mid360_sim
{

class LivoxDriverEmulator : public rclcpp::Node
{
public:
  LivoxDriverEmulator()
  : Node("livox_driver_emulator")
  {
    const auto points_topic = declare_parameter<std::string>("points_topic", "/livox/points");
    const auto imu_in_topic = declare_parameter<std::string>("imu_in_topic", "/livox/imu_raw");
    const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    xfer_format_ = declare_parameter<int>("xfer_format", 1);
    lidar_id_ = static_cast<uint8_t>(declare_parameter<int>("lidar_id", 0));
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    gravity_ = declare_parameter<double>("gravity", 9.80665);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    if (xfer_format_ == 0) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, qos);
    } else {
      custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(lidar_topic, qos);
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, qos);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {OnCloud(std::move(msg));});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {OnImu(std::move(msg));});

    RCLCPP_INFO(
      get_logger(), "%s -> %s (%s), %s -> %s (g)", points_topic.c_str(), lidar_topic.c_str(),
      xfer_format_ == 0 ? "PointCloud2" : "CustomMsg", imu_in_topic.c_str(), imu_topic.c_str());
  }

private:
  struct Offsets
  {
    int x = -1, y = -1, z = -1, intensity = -1, tag = -1, line = -1, timestamp = -1;
  };

  static Offsets FindOffsets(const sensor_msgs::msg::PointCloud2 & msg)
  {
    Offsets o;
    for (const auto & f : msg.fields) {
      if (f.name == "x") {o.x = f.offset;}
      if (f.name == "y") {o.y = f.offset;}
      if (f.name == "z") {o.z = f.offset;}
      if (f.name == "intensity") {o.intensity = f.offset;}
      if (f.name == "tag") {o.tag = f.offset;}
      if (f.name == "line") {o.line = f.offset;}
      if (f.name == "timestamp") {o.timestamp = f.offset;}
    }
    return o;
  }

  template<typename T>
  static T Read(const uint8_t * p)
  {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
  }

  void OnCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    if (!frame_id_.empty()) {
      msg->header.frame_id = frame_id_;
    }
    if (cloud_pub_) {
      cloud_pub_->publish(std::move(msg));
      return;
    }

    const Offsets o = FindOffsets(*msg);
    if (o.x < 0 || o.y < 0 || o.z < 0 || o.timestamp < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 is missing x/y/z/timestamp fields");
      return;
    }

    const std::size_t n = static_cast<std::size_t>(msg->width) * msg->height;
    livox_ros_driver2::msg::CustomMsg out;
    out.header.frame_id = msg->header.frame_id;
    out.lidar_id = lidar_id_;
    out.rsvd = {0, 0, 0};
    out.points.resize(n);

    const uint64_t header_ns =
      static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
    const uint8_t * data = msg->data.data();
    uint64_t timebase = n > 0 ?
      static_cast<uint64_t>(Read<double>(data + o.timestamp)) : header_ns;

    for (std::size_t i = 0; i < n; ++i) {
      const uint8_t * p = data + i * msg->point_step;
      auto & q = out.points[i];
      q.x = Read<float>(p + o.x);
      q.y = Read<float>(p + o.y);
      q.z = Read<float>(p + o.z);
      const float intensity = o.intensity >= 0 ? Read<float>(p + o.intensity) : 0.0f;
      q.reflectivity = static_cast<uint8_t>(std::clamp(intensity, 0.0f, 255.0f));
      q.tag = o.tag >= 0 ? p[o.tag] : 0;
      q.line = o.line >= 0 ? p[o.line] : 0;
      const auto t = static_cast<uint64_t>(Read<double>(p + o.timestamp));
      q.offset_time = static_cast<uint32_t>(t >= timebase ? t - timebase : 0);
    }

    out.timebase = timebase;
    out.header.stamp = rclcpp::Time(static_cast<int64_t>(timebase), RCL_ROS_TIME);
    out.point_num = static_cast<uint32_t>(n);
    custom_pub_->publish(out);
  }

  void OnImu(sensor_msgs::msg::Imu::UniquePtr msg)
  {
    if (!frame_id_.empty()) {
      msg->header.frame_id = frame_id_;
    }
    msg->linear_acceleration.x /= gravity_;
    msg->linear_acceleration.y /= gravity_;
    msg->linear_acceleration.z /= gravity_;
    imu_pub_->publish(std::move(msg));
  }

  int xfer_format_ = 1;
  uint8_t lidar_id_ = 0;
  std::string frame_id_;
  double gravity_ = 9.80665;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace livox_mid360_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<livox_mid360_sim::LivoxDriverEmulator>());
  rclcpp::shutdown();
  return 0;
}
