// Copyright 2026 Kartik Krishnan
// SPDX-License-Identifier: Apache-2.0
//
// Gazebo (gz-sim 8 / Harmonic) system that simulates a Livox Mid-360.
//
// The rays follow the Mid-360's non-repetitive scan pattern. How their
// ranges are measured depends on <mode>:
//
//   snapshot  The gpu_lidar renders one dense range image per frame; every ray
//             of the frame reads it. Points are stamped as if fired over the
//             whole frame, but all of them see the world at the image's time.
//   sliced    The gpu_lidar renders <slices> images per frame (its update_rate
//             must be frame_rate * slices). Each image supplies the next
//             1/slices of the frame's rays, stamped with the image's time, so
//             motion during a frame skews the cloud like on the real sensor
//             (in steps of 1/(frame_rate * slices)).
//   raycast   No rendering: every physics step, the rays due by then are cast
//             on the CPU against the visual geometry of the world, from the
//             sensor pose of that step, and stamped with their own firing time.
//             Exact angles and continuous motion skew, at a CPU cost.
//
// Output: a gz.msgs.PointCloudPacked whose layout matches the PointCloud2 of
// livox_ros_driver2 (xfer_format 0):
//
//   x, y, z, intensity : float32
//   tag, line          : uint8
//   timestamp          : float64 [ns]
//
// Bridge it to ROS 2 with ros_gz_bridge as sensor_msgs/msg/PointCloud2.

#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "livox_mid360_sim/raycaster.hpp"
#include "livox_mid360_sim/scan_pattern.hpp"

namespace livox_mid360_sim
{

namespace
{
// Same packed layout as livox_ros_driver2's LivoxPointXyzrtlt.
#pragma pack(push, 1)
struct LivoxPoint
{
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
};
#pragma pack(pop)
static_assert(sizeof(LivoxPoint) == 26, "LivoxPoint must be packed to 26 bytes");

enum class Mode {kSnapshot, kSliced, kRaycast};

template<typename T>
T Get(const std::shared_ptr<const sdf::Element> & sdf, const std::string & key, T fallback)
{
  if (!sdf || !sdf->HasElement(key)) {
    return fallback;
  }
  return sdf->Get<T>(key);
}

void AddField(
  gz::msgs::PointCloudPacked & msg, const std::string & name, uint32_t offset,
  gz::msgs::PointCloudPacked::Field::DataType type)
{
  auto * field = msg.add_field();
  field->set_name(name);
  field->set_offset(offset);
  field->set_datatype(type);
  field->set_count(1);
}

int64_t StampNs(const gz::msgs::Time & t)
{
  return static_cast<int64_t>(t.sec()) * 1000000000LL + t.nsec();
}

/// Read access to the dense range image of the gpu_lidar.
class DenseImage
{
public:
  DenseImage(const gz::msgs::LaserScan & scan, double min_range, double max_range)
  : scan_(scan), min_range_(min_range), max_range_(max_range)
  {
    h_count_ = static_cast<int>(scan.count());
    v_count_ = std::max(1, static_cast<int>(scan.vertical_count()));
    h_min_ = scan.angle_min();
    h_step_ = scan.angle_step();
    v_min_ = scan.vertical_angle_min();
    v_step_ = v_count_ > 1 ? scan.vertical_angle_step() : 0.0;
    has_intensity_ = scan.intensities_size() == scan.ranges_size();
  }

  bool Valid() const
  {
    return h_count_ >= 2 && scan_.ranges_size() >= h_count_ * v_count_;
  }

  /// Range of `ray` [m] and the hit surface's retro value (0 if unknown).
  /// False if the ray is outside the image or has no return.
  bool Sample(const PatternRay & ray, bool interpolate, double & range, double & retro) const
  {
    // Continuous image coordinates of the ray in the dense range image.
    const double fh = (ray.azimuth - h_min_) / h_step_;
    const double fv = v_step_ > 0.0 ? (ray.elevation - v_min_) / v_step_ : 0.0;
    if (fh < 0.0 || fh > h_count_ - 1 || fv < 0.0 || fv > v_count_ - 1) {
      return false;  // outside the rendered FOV
    }

    const int h0 = static_cast<int>(fh);
    const int v0 = static_cast<int>(fv);
    const int h1 = std::min(h0 + 1, h_count_ - 1);
    const int v1 = std::min(v0 + 1, v_count_ - 1);
    const double ah = fh - h0;
    const double av = fv - v0;

    const int hn = ah < 0.5 ? h0 : h1;
    const int vn = av < 0.5 ? v0 : v1;
    range = At(hn, vn);

    if (interpolate) {
      // Bilinear interpolation between the four neighbouring rays, but only
      // on continuous surfaces so edges do not smear into the void.
      const double r00 = At(h0, v0), r10 = At(h1, v0);
      const double r01 = At(h0, v1), r11 = At(h1, v1);
      if (InRange(r00) && InRange(r10) && InRange(r01) && InRange(r11)) {
        const double lo = std::min({r00, r10, r01, r11});
        const double hi = std::max({r00, r10, r01, r11});
        if (hi - lo < 0.05 + 0.02 * lo) {
          range = (1 - av) * ((1 - ah) * r00 + ah * r10) + av * ((1 - ah) * r01 + ah * r11);
        }
      }
    }
    if (!InRange(range)) {
      return false;  // no return
    }
    retro = has_intensity_ ? scan_.intensities(vn * h_count_ + hn) : 0.0;
    return true;
  }

private:
  double At(int h, int v) const {return scan_.ranges(v * h_count_ + h);}
  bool InRange(double r) const {return std::isfinite(r) && r >= min_range_ && r <= max_range_;}

  const gz::msgs::LaserScan & scan_;
  double min_range_, max_range_;
  int h_count_ = 0, v_count_ = 1;
  double h_min_ = 0.0, h_step_ = 1.0, v_min_ = 0.0, v_step_ = 0.0;
  bool has_intensity_ = false;
};
}  // namespace

class LivoxMid360System
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    model_ = entity;
    input_topic_ = Get<std::string>(sdf, "input_topic", "/livox/dense_scan");
    output_topic_ = Get<std::string>(sdf, "output_topic", "/livox/points");
    frame_id_ = Get<std::string>(sdf, "frame_id", "livox_frame");
    frame_rate_ = Get<double>(sdf, "frame_rate", 10.0);
    min_range_ = Get<double>(sdf, "min_range", 0.1);
    max_range_ = Get<double>(sdf, "max_range", 40.0);
    default_reflectivity_ = Get<double>(sdf, "default_reflectivity", 100.0);
    interpolate_ = Get<bool>(sdf, "interpolate", true);
    slices_ = std::max(1, Get<int>(sdf, "slices", 5));
    sensor_name_ = Get<std::string>(sdf, "sensor_name", "");
    range_noise_ = std::max(0.0, Get<double>(sdf, "range_noise", 0.02));

    const auto mode = Get<std::string>(sdf, "mode", "snapshot");
    if (mode == "snapshot") {
      mode_ = Mode::kSnapshot;
    } else if (mode == "sliced") {
      mode_ = Mode::kSliced;
    } else if (mode == "raycast") {
      mode_ = Mode::kRaycast;
    } else {
      gzerr << "[LivoxMid360] unknown <mode> [" << mode
            << "], expected snapshot, sliced or raycast; using snapshot\n";
      mode_ = Mode::kSnapshot;
    }

    ScanPattern::Params p;
    p.points_per_second = Get<double>(sdf, "points_per_second", p.points_per_second);
    p.rotation_rate = Get<double>(sdf, "rotation_rate", p.rotation_rate);
    p.num_lines = Get<int>(sdf, "num_lines", p.num_lines);
    p.sweeps_per_revolution =
      Get<double>(sdf, "sweeps_per_revolution", p.sweeps_per_revolution);
    p.elevation_min = Get<double>(sdf, "elevation_min", p.elevation_min);
    p.elevation_max = Get<double>(sdf, "elevation_max", p.elevation_max);
    points_per_second_ = p.points_per_second;
    pattern_ = std::make_unique<ScanPattern>(p);

    // An explicitly empty <pattern_file/> selects the procedural pattern.
    const auto pattern_uri = Get<std::string>(
      sdf, "pattern_file", "model://livox_mid360/scan_mode/mid360.csv");
    if (!pattern_uri.empty()) {
      const auto pattern_file = ScanPattern::ResolvePath(pattern_uri);
      std::string error = "cannot resolve " + pattern_uri;
      if (!pattern_file.empty() && pattern_->LoadCsv(pattern_file, &error)) {
        gzmsg << "[LivoxMid360] Loaded " << pattern_->CsvSize()
              << " rays from scan pattern [" << pattern_file << "]\n";
      } else {
        gzwarn << "[LivoxMid360] " << error
               << ", falling back to the procedural pattern\n";
      }
    }

    if (frame_rate_ <= 0.0 || points_per_second_ <= 0.0) {
      gzerr << "[LivoxMid360] frame_rate and points_per_second must be > 0\n";
      return;
    }
    points_per_frame_ =
      static_cast<std::size_t>(std::llround(points_per_second_ / frame_rate_));

    publisher_ = node_.Advertise<gz::msgs::PointCloudPacked>(output_topic_);

    if (mode_ == Mode::kRaycast) {
      if (sensor_name_.empty()) {
        gzerr << "[LivoxMid360] raycast mode needs <sensor_name>: the sensor whose pose "
              << "is the lidar origin\n";
        return;
      }
      gzmsg << "[LivoxMid360] raycast mode: " << points_per_frame_ << " pts/frame @ "
            << frame_rate_ << " Hz from the pose of sensor [" << sensor_name_ << "], range noise "
            << range_noise_ << " m -> " << output_topic_ << "\n";
      configured_ = true;
      return;
    }

    if (!node_.Subscribe(input_topic_, &LivoxMid360System::OnScan, this)) {
      gzerr << "[LivoxMid360] Failed to subscribe to [" << input_topic_ << "]\n";
      return;
    }
    gzmsg << "[LivoxMid360] " << (mode_ == Mode::kSliced ? "sliced" : "snapshot") << " mode: "
          << input_topic_ << " -> " << output_topic_ << " (" << points_per_frame_
          << " pts/frame @ " << frame_rate_ << " Hz";
    if (mode_ == Mode::kSliced) {
      gzmsg << " in " << slices_ << " images, the gpu_lidar must render at "
            << frame_rate_ * slices_ << " Hz";
    }
    gzmsg << ", frame_id " << frame_id_ << ")\n";
    configured_ = true;
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (mode_ != Mode::kRaycast || !configured_ || info.paused) {
      return;
    }
    const int64_t now =
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();

    if (sensor_ == gz::sim::kNullEntity) {
      for (const auto e : ecm.Descendants(model_)) {
        const auto * name = ecm.Component<gz::sim::components::Name>(e);
        if (name && name->Data() == sensor_name_ &&
          ecm.Component<gz::sim::components::Sensor>(e))
        {
          sensor_ = e;
          break;
        }
      }
      if (sensor_ == gz::sim::kNullEntity) {
        if (!warned_no_sensor_) {
          gzwarn << "[LivoxMid360] waiting for sensor [" << sensor_name_ << "] in the model\n";
          warned_no_sensor_ = true;
        }
        return;
      }
    }

    if (started_ && now < last_ns_) {
      started_ = false;  // world reset: time went back, drop the partial frame
    }
    last_ns_ = now;
    if (!started_) {
      t0_ns_ = now;
      fired_ = 0;
      frame_.clear();
      started_ = true;
    }

    scene_.Update(ecm);
    const gz::math::Pose3d pose = gz::sim::worldPose(sensor_, ecm);

    // Rays whose firing time is <= now are due; they see the world as of this
    // step (at most one physics step after their firing time).
    const double ns_per_ray = 1e9 / points_per_second_;
    const uint64_t due = static_cast<uint64_t>((now - t0_ns_) / ns_per_ray) + 1;
    if (due <= fired_) {
      return;
    }
    rays_.clear();
    pattern_->Next(static_cast<std::size_t>(due - fired_), rays_);

    for (const auto & ray : rays_) {
      const uint64_t k = fired_++;
      const std::size_t i = static_cast<std::size_t>(k % points_per_frame_);
      if (i == 0) {
        frame_.clear();
        frame_.reserve(points_per_frame_);
        frame_ns_ = t0_ns_ + std::llround(static_cast<double>(k) * ns_per_ray);
      }

      const double cos_el = std::cos(ray.elevation);
      const gz::math::Vector3d dir(
        cos_el * std::cos(ray.azimuth), cos_el * std::sin(ray.azimuth), std::sin(ray.elevation));
      double range = 0.0, retro = 0.0;
      if (scene_.Intersect(pose.Pos(), pose.Rot() * dir, min_range_, max_range_, range, retro)) {
        if (range_noise_ > 0.0) {
          range += range_noise_ * gauss_(rng_);
        }
        if (range >= min_range_ && range <= max_range_) {
          frame_.push_back(
            MakePoint(
              ray, range, retro,
              static_cast<double>(t0_ns_) + static_cast<double>(k) * ns_per_ray));
        }
      }

      if (i + 1 == points_per_frame_) {
        Publish(frame_, frame_ns_);
      }
    }
  }

private:
  LivoxPoint MakePoint(
    const PatternRay & ray, double range, double retro, double timestamp_ns) const
  {
    const double cos_el = std::cos(ray.elevation);
    LivoxPoint pt;
    pt.x = static_cast<float>(range * cos_el * std::cos(ray.azimuth));
    pt.y = static_cast<float>(range * cos_el * std::sin(ray.azimuth));
    pt.z = static_cast<float>(range * std::sin(ray.elevation));
    const double reflectivity = retro > 0.0 ? retro : default_reflectivity_;
    pt.intensity = static_cast<float>(std::clamp(reflectivity, 0.0, 255.0));
    pt.tag = 0;
    pt.line = ray.line;
    pt.timestamp = timestamp_ns;
    return pt;
  }

  void OnScan(const gz::msgs::LaserScan & scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const DenseImage image(scan, min_range_, max_range_);
    if (!image.Valid()) {
      return;
    }
    const int64_t stamp_ns = StampNs(scan.header().stamp());

    if (mode_ == Mode::kSnapshot) {
      // One image per frame; points stamped as if fired over the frame.
      rays_.clear();
      pattern_->Next(points_per_frame_, rays_);
      const double dt_ns = 1e9 / points_per_second_;
      frame_.clear();
      frame_.reserve(rays_.size());
      for (std::size_t i = 0; i < rays_.size(); ++i) {
        double range = 0.0, retro = 0.0;
        if (image.Sample(rays_[i], interpolate_, range, retro)) {
          frame_.push_back(
            MakePoint(
              rays_[i], range, retro,
              static_cast<double>(stamp_ns) + static_cast<double>(i) * dt_ns));
        }
      }
      Publish(frame_, stamp_ns);
      return;
    }

    // Sliced: this image supplies the next 1/slices of the frame's rays.
    if (last_scan_ns_ > 0 && stamp_ns < last_scan_ns_) {
      slice_ = 0;  // world reset: drop the partial frame
    } else if (last_scan_ns_ > 0 && !warned_rate_) {
      const double expected = 1e9 / (frame_rate_ * slices_);
      const double dt = static_cast<double>(stamp_ns - last_scan_ns_);
      if (std::abs(dt - expected) > 0.2 * expected) {
        gzwarn << "[LivoxMid360] sliced mode expects a dense image every " << expected * 1e-6
               << " ms but got one after " << dt * 1e-6 << " ms: set the gpu_lidar "
               << "update_rate to frame_rate * slices = " << frame_rate_ * slices_ << " Hz\n";
        warned_rate_ = true;
      }
    }
    last_scan_ns_ = stamp_ns;

    if (slice_ == 0) {
      frame_.clear();
      frame_.reserve(points_per_frame_);
      frame_ns_ = stamp_ns;
    }
    const std::size_t begin = points_per_frame_ * slice_ / slices_;
    const std::size_t end = points_per_frame_ * (slice_ + 1) / slices_;
    rays_.clear();
    pattern_->Next(end - begin, rays_);
    for (const auto & ray : rays_) {
      double range = 0.0, retro = 0.0;
      if (image.Sample(ray, interpolate_, range, retro)) {
        frame_.push_back(MakePoint(ray, range, retro, static_cast<double>(stamp_ns)));
      }
    }
    if (++slice_ < slices_) {
      return;
    }
    slice_ = 0;
    Publish(frame_, frame_ns_);
  }

  void Publish(const std::vector<LivoxPoint> & points, int64_t stamp_ns)
  {
    gz::msgs::PointCloudPacked msg;
    msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(stamp_ns / 1000000000LL));
    msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(stamp_ns % 1000000000LL));
    auto * frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(frame_id_);

    using Field = gz::msgs::PointCloudPacked::Field;
    AddField(msg, "x", offsetof(LivoxPoint, x), Field::FLOAT32);
    AddField(msg, "y", offsetof(LivoxPoint, y), Field::FLOAT32);
    AddField(msg, "z", offsetof(LivoxPoint, z), Field::FLOAT32);
    AddField(msg, "intensity", offsetof(LivoxPoint, intensity), Field::FLOAT32);
    AddField(msg, "tag", offsetof(LivoxPoint, tag), Field::UINT8);
    AddField(msg, "line", offsetof(LivoxPoint, line), Field::UINT8);
    AddField(msg, "timestamp", offsetof(LivoxPoint, timestamp), Field::FLOAT64);

    msg.set_height(1);
    msg.set_width(static_cast<uint32_t>(points.size()));
    msg.set_is_bigendian(false);
    msg.set_point_step(sizeof(LivoxPoint));
    msg.set_row_step(static_cast<uint32_t>(sizeof(LivoxPoint) * points.size()));
    msg.set_is_dense(true);
    msg.mutable_data()->assign(
      reinterpret_cast<const char *>(points.data()), sizeof(LivoxPoint) * points.size());

    publisher_.Publish(msg);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::mutex mutex_;

  std::unique_ptr<ScanPattern> pattern_;
  std::vector<PatternRay> rays_;
  std::vector<LivoxPoint> frame_;   ///< frame being collected (sliced / raycast)
  int64_t frame_ns_ = 0;            ///< its header stamp

  Mode mode_ = Mode::kSnapshot;
  bool configured_ = false;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double frame_rate_ = 10.0;
  double points_per_second_ = 200000.0;
  std::size_t points_per_frame_ = 20000;
  double min_range_ = 0.1;
  double max_range_ = 40.0;
  double default_reflectivity_ = 100.0;
  bool interpolate_ = true;

  // sliced
  int slices_ = 5;
  int slice_ = 0;
  int64_t last_scan_ns_ = 0;
  bool warned_rate_ = false;

  // raycast
  gz::sim::Entity model_ = gz::sim::kNullEntity;
  gz::sim::Entity sensor_ = gz::sim::kNullEntity;
  std::string sensor_name_;
  bool warned_no_sensor_ = false;
  Scene scene_;
  bool started_ = false;
  int64_t t0_ns_ = 0;
  int64_t last_ns_ = 0;
  uint64_t fired_ = 0;
  double range_noise_ = 0.02;
  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace livox_mid360_sim

GZ_ADD_PLUGIN(
  livox_mid360_sim::LivoxMid360System,
  gz::sim::System,
  livox_mid360_sim::LivoxMid360System::ISystemConfigure,
  livox_mid360_sim::LivoxMid360System::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(livox_mid360_sim::LivoxMid360System, "livox_mid360_sim::LivoxMid360")
