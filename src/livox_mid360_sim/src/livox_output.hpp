// Output contract shared by all sensor back-ends:
//  * the driver's packed 26-byte point (LivoxPointXyzrtlt),
//  * a return model (min/max range, reflectivity-dependent detection range, range noise).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <gz/msgs/pointcloud_packed.pb.h>

namespace livox_mid360_sim
{

#pragma pack(push, 1)
struct LivoxPoint
{
  float x, y, z;
  float intensity;      // reflectivity 0..255
  uint8_t tag;
  uint8_t line;
  double timestamp;     // absolute time [ns]
};
#pragma pack(pop)
static_assert(sizeof(LivoxPoint) == 26, "Livox point must be 26 bytes");

inline void FillPointCloudPacked(
  gz::msgs::PointCloudPacked & msg, const std::vector<LivoxPoint> & pts,
  const std::string & frame_id, int64_t stamp_ns)
{
  msg.Clear();
  auto * h = msg.mutable_header();
  h->mutable_stamp()->set_sec(static_cast<int64_t>(stamp_ns / 1000000000LL));
  h->mutable_stamp()->set_nsec(static_cast<int32_t>(stamp_ns % 1000000000LL));
  auto * d = h->add_data();
  d->set_key("frame_id");
  d->add_value(frame_id);

  using F = gz::msgs::PointCloudPacked::Field;
  auto add = [&msg](const char * name, uint32_t off, F::DataType t) {
      auto * f = msg.add_field();
      f->set_name(name);
      f->set_offset(off);
      f->set_datatype(t);
      f->set_count(1);
    };
  add("x", 0, F::FLOAT32);
  add("y", 4, F::FLOAT32);
  add("z", 8, F::FLOAT32);
  add("intensity", 12, F::FLOAT32);
  add("tag", 16, F::UINT8);
  add("line", 17, F::UINT8);
  add("timestamp", 18, F::FLOAT64);

  msg.set_height(1);
  msg.set_width(static_cast<uint32_t>(pts.size()));
  msg.set_is_bigendian(false);
  msg.set_point_step(sizeof(LivoxPoint));
  msg.set_row_step(static_cast<uint32_t>(sizeof(LivoxPoint) * pts.size()));
  msg.set_is_dense(true);
  msg.mutable_data()->assign(
    reinterpret_cast<const char *>(pts.data()), pts.size() * sizeof(LivoxPoint));
}

struct ReturnModelParams
{
  double min_range{0.1};
  double max_range{70.0};          // hard limit (80 % reflectivity)
  double nominal_range{40.0};      // detection range at 10 % reflectivity
  double range_noise_stddev{0.02};
  double default_reflectivity{100.0};   // Livox scale: 0..150 diffuse, 151..255 retro
  unsigned seed{0};
};

/// Turns a true range into what the Mid-360 reports, or rejects it (no return).
class ReturnModel
{
public:
  explicit ReturnModel(const ReturnModelParams & p = {})
  : p_(p), rng_(p.seed ? p.seed : std::random_device{}()), noise_(0.0, p.range_noise_stddev) {}

  const ReturnModelParams & params() const {return p_;}

  /// retro: laser_retro of the surface (0 if unknown). Returns false for "no return".
  bool Apply(double true_range, double retro, float & range_out, uint8_t & refl_out)
  {
    if (!std::isfinite(true_range) || true_range <= 0.0) {return false;}
    const double refl = retro > 0.0 ? std::min(retro, 255.0) : p_.default_reflectivity;
    // Detection range ~ sqrt(reflectance): 40 m @ 10 %, capped at max_range.
    const double rho = std::max(refl, 1.0) / 150.0;
    const double r_det = std::min(p_.max_range, p_.nominal_range * std::sqrt(rho / 0.1));
    if (true_range > r_det) {return false;}
    const double r = p_.range_noise_stddev > 0.0 ? true_range + noise_(rng_) : true_range;
    if (r < p_.min_range || r > r_det) {return false;}
    range_out = static_cast<float>(r);
    refl_out = static_cast<uint8_t>(std::clamp(refl, 0.0, 255.0));
    return true;
  }

private:
  ReturnModelParams p_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
};

}  // namespace livox_mid360_sim
