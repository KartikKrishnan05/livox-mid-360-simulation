// Livox Mid-360 gz-sim system (approach: dense GPU render sampled along the real pattern).
//
// A dense gpu_lidar (e.g. 1440 x 160 over 360 x -7.5..52.5 deg) renders the scene. This
// system subscribes to its gz.msgs.LaserScan and, for every 100 ms frame, samples the next
// 20 000 directions of the recorded Mid-360 pattern from it. The result is published as
// gz.msgs.PointCloudPacked in livox_ros_driver2's 26-byte layout (x,y,z,intensity,tag,line,
// timestamp[ns]); livox_driver_emulator turns that into CustomMsg.
#include <algorithm>
#include <iostream>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "dense_scan.hpp"
#include "livox_output.hpp"
#include "scan_pattern.hpp"

namespace livox_mid360_sim
{

class LivoxMid360 : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    std::cerr << "[livox_mid360] sensor: main - single GPU render per frame (no motion skew)" << std::endl;
    auto get_s = [&](const char * k, const std::string & d) {
        return sdf->HasElement(k) ? sdf->Get<std::string>(k) : d;
      };
    auto get_d = [&](const char * k, double d) {
        return sdf->HasElement(k) ? sdf->Get<double>(k) : d;
      };

    frame_id_ = get_s("frame_id", "livox_frame");
    points_topic_ = get_s("points_topic", "/livox/points");
    dense_topic_ = get_s("dense_topic", "/livox/dense_scan");
    point_rate_ = get_d("point_rate", 200000.0);
    frame_rate_ = get_d("frame_rate", 10.0);
    edge_threshold_ = get_d("edge_threshold", 0.1);
    points_per_frame_ = static_cast<std::size_t>(std::lround(point_rate_ / frame_rate_));

    ReturnModelParams rp;
    rp.min_range = get_d("min_range", rp.min_range);
    rp.max_range = get_d("max_range", rp.max_range);
    rp.nominal_range = get_d("nominal_range", rp.nominal_range);
    rp.range_noise_stddev = get_d("range_noise_stddev", rp.range_noise_stddev);
    rp.default_reflectivity = get_d("default_reflectivity", rp.default_reflectivity);
    rp.seed = static_cast<unsigned>(get_d("seed", 0));
    model_ = ReturnModel(rp);

    const std::string uri = get_s("pattern_file", "model://livox_mid360/scan_mode/mid360.csv");
    const std::string path = ResolveResourceUri(uri);
    std::string err;
    if (!path.empty() && pattern_.LoadCsv(path, &err)) {
      gzmsg << "[livox_mid360] loaded scan pattern " << path << " (" << pattern_.size()
            << " points, el " << pattern_.MinElevation() * 180 / M_PI << ".."
            << pattern_.MaxElevation() * 180 / M_PI << " deg)\n";
    } else {
      gzwarn << "[livox_mid360] scan pattern '" << uri << "' not found"
             << (err.empty() ? "" : " (" + err + ")")
             << "; falling back to the procedural 4-laser pattern\n";
      pattern_.MakeProcedural(static_cast<std::size_t>(4 * point_rate_));
    }

    pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(points_topic_);
    node_.Subscribe(dense_topic_, &LivoxMid360::OnDenseScan, this);
    gzmsg << "[livox_mid360] " << dense_topic_ << " -> " << points_topic_ << ", "
          << points_per_frame_ << " pts/frame @ " << frame_rate_ << " Hz, frame '"
          << frame_id_ << "'\n";
  }

private:
  void OnDenseScan(const gz::msgs::LaserScan & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto scan = DenseScan::FromMsg(msg);
    if (!scan) {return;}

    const double dt_ns = 1e9 / point_rate_;
    const int64_t frame_ns = static_cast<int64_t>(std::llround(1e9 / frame_rate_));
    if (scan->stamp_ns < last_end_ns_) {last_end_ns_ = -1;}       // world reset
    if (last_end_ns_ >= 0 && scan->stamp_ns - last_end_ns_ < frame_ns / 2) {return;}

    // The frame ends at the render time; like the real driver, it is published once its
    // last point has been measured, stamped with the time of its first point.
    const int64_t timebase =
      scan->stamp_ns - static_cast<int64_t>(std::llround((points_per_frame_ - 1) * dt_ns));
    if (timebase < 0) {return;}
    last_end_ns_ = scan->stamp_ns;

    points_.clear();
    points_.reserve(points_per_frame_);
    for (std::size_t i = 0; i < points_per_frame_; ++i) {
      const PatternRay & ray = pattern_[cursor_ + i];
      double r, retro;
      if (!scan->Sample(ray.az, ray.el, edge_threshold_, r, retro)) {continue;}
      float range;
      uint8_t refl;
      if (!model_.Apply(r, retro, range, refl)) {continue;}
      LivoxPoint p;
      p.x = ray.dx * range;
      p.y = ray.dy * range;
      p.z = ray.dz * range;
      p.intensity = refl;
      p.tag = 0;
      p.line = ray.line;
      p.timestamp = static_cast<double>(timebase + std::llround(i * dt_ns));
      points_.push_back(p);
    }
    cursor_ = (cursor_ + points_per_frame_) % pattern_.size();

    FillPointCloudPacked(out_, points_, frame_id_, timebase);
    pub_.Publish(out_);
  }

  std::mutex mutex_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  std::string frame_id_, points_topic_, dense_topic_;
  double point_rate_{200000}, frame_rate_{10}, edge_threshold_{0.1};
  std::size_t points_per_frame_{20000};
  std::size_t cursor_{0};
  int64_t last_end_ns_{-1};
  ScanPattern pattern_;
  ReturnModel model_;
  std::vector<LivoxPoint> points_;
  gz::msgs::PointCloudPacked out_;
};

}  // namespace livox_mid360_sim

GZ_ADD_PLUGIN(
  livox_mid360_sim::LivoxMid360, gz::sim::System,
  livox_mid360_sim::LivoxMid360::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(livox_mid360_sim::LivoxMid360, "livox_mid360_sim::LivoxMid360")
