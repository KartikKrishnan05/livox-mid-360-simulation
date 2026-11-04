// Livox Mid-360 gz-sim system (approach 1: dense GPU render sampled along the real pattern).
//
// A dense gpu_lidar (e.g. 1440 x 160 over 360 x -7.5..52.5 deg) renders the scene at
// 50-100 Hz. This system buffers those renders (gz.msgs.LaserScan) and builds one frame of
// 20 000 points every 100 ms: point i of the recorded Mid-360 pattern is emitted at
// timebase + i * 5 us and is sampled from the render *nearest its own timestamp*. Each render
// is expressed in the sensor frame at its render time, so motion skew within a frame is
// real (to within half a render period). Output: gz.msgs.PointCloudPacked in
// livox_ros_driver2's 26-byte layout (x,y,z,intensity,tag,line,timestamp[ns]);
// livox_driver_emulator turns that into CustomMsg.
#include <algorithm>
#include <iostream>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
// Second plugin in this library: RegisterMore (Register.hh is in the raycast system).
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "dense_scan.hpp"
#include "livox_output.hpp"
#include "scan_pattern.hpp"

namespace livox_mid360_gazebo
{

class LivoxMid360Gpu : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    std::cerr << "[livox_mid360] sensor: gpu (approach 1) - dense GPU render, nearest-render sampling" << std::endl;
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

    model_ = ReturnModel(ReturnModelParamsFromSdf(get_d));

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
    node_.Subscribe(dense_topic_, &LivoxMid360Gpu::OnDenseScan, this);
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

    if (!renders_.empty() && scan->stamp_ns <= renders_.back()->stamp_ns) {
      renders_.clear();                                   // world reset / time jump
      next_timebase_ = -1;
    }
    renders_.push_back(scan);
    if (next_timebase_ < 0) {next_timebase_ = scan->stamp_ns;}

    const double dt_ns = 1e9 / point_rate_;
    const int64_t frame_ns = static_cast<int64_t>(std::llround(points_per_frame_ * dt_ns));
    // A frame can be built once a render at/after its last point exists: no later render
    // can be nearer to any of its points.
    while (true) {
      const int64_t t_last =
        next_timebase_ + static_cast<int64_t>(std::llround((points_per_frame_ - 1) * dt_ns));
      if (renders_.back()->stamp_ns < t_last) {break;}
      BuildAndPublish(next_timebase_, dt_ns);
      next_timebase_ += frame_ns;
      // Keep the newest render before the next frame starts (it may be the nearest).
      while (renders_.size() > 1 && renders_[1]->stamp_ns <= next_timebase_) {
        renders_.pop_front();
      }
    }
  }

  void BuildAndPublish(int64_t timebase, double dt_ns)
  {
    points_.clear();
    points_.reserve(points_per_frame_);
    std::size_t k = 0;                              // index of the current nearest render
    for (std::size_t i = 0; i < points_per_frame_; ++i) {
      const int64_t t = timebase + static_cast<int64_t>(std::llround(i * dt_ns));
      while (k + 1 < renders_.size() &&
        std::llabs(renders_[k + 1]->stamp_ns - t) <= std::llabs(renders_[k]->stamp_ns - t))
      {
        ++k;
      }
      const DenseScan & scan = *renders_[k];
      const PatternRay & ray = pattern_[cursor_ + i];
      // Sample along the scattered beam; report along the nominal pattern direction.
      double d_az, d_el;
      model_.AngularNoise(ray.el, d_az, d_el);
      double r, retro, cos_inc;
      if (!scan.Sample(ray.az + d_az, ray.el + d_el, edge_threshold_, r, retro, cos_inc)) {
        continue;
      }
      float range;
      uint8_t refl;
      if (!model_.Apply(r, retro, cos_inc, range, refl)) {continue;}
      LivoxPoint p;
      p.x = ray.dx * range;
      p.y = ray.dy * range;
      p.z = ray.dz * range;
      p.intensity = refl;
      p.tag = 0;
      p.line = ray.line;
      p.timestamp = static_cast<double>(t);
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
  int64_t next_timebase_{-1};
  std::deque<std::shared_ptr<DenseScan>> renders_;
  ScanPattern pattern_;
  ReturnModel model_;
  std::vector<LivoxPoint> points_;
  gz::msgs::PointCloudPacked out_;
};

}  // namespace livox_mid360_gazebo

GZ_ADD_PLUGIN(
  livox_mid360_gazebo::LivoxMid360Gpu, gz::sim::System,
  livox_mid360_gazebo::LivoxMid360Gpu::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(livox_mid360_gazebo::LivoxMid360Gpu, "livox_mid360_gazebo::LivoxMid360Gpu")
