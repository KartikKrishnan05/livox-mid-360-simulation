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

namespace livox_mid360_gazebo
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
  double max_range{70.0};          // hard limit (datasheet: 70 m @ 80 % reflectivity)
  double nominal_range{40.0};      // detection range at 10 % reflectivity, normal incidence
  double range_noise_stddev{0.02};      // 1 sigma at 10 m (datasheet: <= 2 cm @ 10 m)
  double near_range_noise_stddev{0.03}; // 1 sigma at 0.2 m (datasheet: <= 3 cm @ 0.2 m)
  double angular_noise_stddev{0.15 * M_PI / 180.0};  // [rad] (datasheet: < 0.15 deg, 1 sigma)
  double default_reflectivity{100.0};   // Livox scale: 0..150 diffuse, 151..255 retro
  unsigned seed{0};
};

/// Turns a true range into what the Mid-360 reports, or rejects it (no return).
///
///  * Diffuse surfaces (reflectivity <= 150, i.e. reflectance rho = refl / 150) return
///    rho * cos(incidence) of the light (Lambertian). That effective reflectance sets
///    - the detection range, r_det = nominal_range * (rho_eff / 0.1)^k, k = ln(70/40) / ln 8,
///      which passes through both datasheet points (40 m @ 10 %, 70 m @ 80 %), and
///    - the reported reflectivity, refl * cos(incidence) (Livox calibrates for range, not
///      for incidence angle, so grazing surfaces read dark).
///    Retro-reflectors (> 150) are treated as angle-independent.
///  * Detection falls off smoothly around r_det (logistic, 5 % of r_det wide), not as a
///    hard edge; max_range is a hard cap.
///  * Range noise: Gaussian along the ray, 1 sigma interpolated (log range) from
///    near_range_noise_stddev @ 0.2 m to range_noise_stddev @ 10 m, growing with the
///    weakness of the return (x (1 + (r / r_det)^2), i.e. x2 at the detection limit).
///    range_noise_stddev = 0 turns all range noise off.
///  * Angular noise (PerturbDirection): the beam leaves along a direction scattered by
///    angular_noise_stddev, but the point is reported along the nominal pattern direction.
class ReturnModel
{
public:
  explicit ReturnModel(const ReturnModelParams & p = {})
  : p_(p), rng_(p.seed ? p.seed : std::random_device{}()) {}

  const ReturnModelParams & params() const {return p_;}

  /// Angular offsets [rad] (d_az, d_el) for one beam: isotropic on the sphere (d_az is
  /// scaled by 1 / cos(el)). Zero when angular noise is off.
  void AngularNoise(double el, double & d_az, double & d_el)
  {
    d_az = d_el = 0.0;
    if (p_.angular_noise_stddev <= 0.0) {return;}
    d_el = p_.angular_noise_stddev * unit_(rng_);
    d_az = p_.angular_noise_stddev * unit_(rng_) / std::max(std::cos(el), 1e-3);
  }

  /// retro: laser_retro of the surface (0 if unknown). cos_inc: |cos| of the angle between
  /// the beam and the surface normal (1 if unknown). Returns false for "no return".
  bool Apply(
    double true_range, double retro, double cos_inc, float & range_out, uint8_t & refl_out)
  {
    if (!std::isfinite(true_range) || true_range <= 0.0) {return false;}
    const double refl = retro > 0.0 ? std::min(retro, 255.0) : p_.default_reflectivity;
    const bool diffuse = refl <= 150.0;
    const double c = std::clamp(std::isfinite(cos_inc) ? cos_inc : 1.0, 0.0, 1.0);
    const double rho_eff = std::max(refl, 1.0) / 150.0 * (diffuse ? c : 1.0);
    if (rho_eff <= 0.0) {return false;}
    static const double k = std::log(70.0 / 40.0) / std::log(8.0);
    const double r_det = p_.nominal_range * std::pow(rho_eff / 0.1, k);
    if (true_range > p_.max_range) {return false;}
    // Smooth detection limit: P(return) = 1 / (1 + exp((r - r_det) / (0.05 r_det))).
    const double z = (true_range - r_det) / (0.05 * r_det);
    if (z > 30.0) {return false;}
    if (z > -30.0 && prob_(rng_) * (1.0 + std::exp(z)) > 1.0) {return false;}

    double r = true_range;
    if (p_.range_noise_stddev > 0.0) {
      const double a = std::clamp(
        std::log(std::max(true_range, 0.2) / 0.2) / std::log(10.0 / 0.2), 0.0, 1.0);
      const double near = std::max(p_.near_range_noise_stddev, 0.0);
      const double sigma = (near + (p_.range_noise_stddev - near) * a) *
        (1.0 + std::pow(std::min(true_range / r_det, 1.5), 2));
      r += sigma * unit_(rng_);
    }
    if (r < p_.min_range || r > p_.max_range) {return false;}
    range_out = static_cast<float>(r);
    refl_out = static_cast<uint8_t>(std::clamp(std::round(diffuse ? refl * c : refl), 0.0, 255.0));
    return true;
  }

private:
  ReturnModelParams p_;
  std::mt19937 rng_;
  std::normal_distribution<double> unit_{0.0, 1.0};
  std::uniform_real_distribution<double> prob_{0.0, 1.0};
};

/// The return-model parameters from a plugin's SDF (same element names in both systems).
template<typename GetD>
ReturnModelParams ReturnModelParamsFromSdf(GetD get_d)
{
  ReturnModelParams rp;
  rp.min_range = get_d("min_range", rp.min_range);
  rp.max_range = get_d("max_range", rp.max_range);
  rp.nominal_range = get_d("nominal_range", rp.nominal_range);
  rp.range_noise_stddev = get_d("range_noise_stddev", rp.range_noise_stddev);
  // Default keeps the datasheet ratio (3 cm @ 0.2 m : 2 cm @ 10 m).
  rp.near_range_noise_stddev = get_d("near_range_noise_stddev", 1.5 * rp.range_noise_stddev);
  rp.angular_noise_stddev =
    get_d("angular_noise_stddev_deg", rp.angular_noise_stddev * 180.0 / M_PI) * M_PI / 180.0;
  rp.default_reflectivity = get_d("default_reflectivity", rp.default_reflectivity);
  rp.seed = static_cast<unsigned>(get_d("seed", 0));
  return rp;
}

}  // namespace livox_mid360_gazebo
