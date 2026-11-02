// Livox Mid-360 scan pattern: one unit direction + laser line per emitted point.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace livox_mid360_sim
{

struct PatternRay
{
  float dx, dy, dz;   // unit direction in the lidar frame
  float az, el;       // [rad], az CCW from +x in [0, 2pi), el above the xy plane
  uint8_t line;       // laser 0..3
};

class ScanPattern
{
public:
  /// Load the CSV ("Time/s,Azimuth/deg,Zenith/deg"; first column is an index,
  /// zenith measured from +z). Rows come in groups of 4 lasers, so line = row % 4.
  bool LoadCsv(const std::string & path, std::string * error);

  /// Fallback: 4 stacked lasers ~3 deg apart at the same azimuth, rotating
  /// non-repetitively. Only used when the CSV can't be found.
  void MakeProcedural(std::size_t num_points);

  std::size_t size() const {return rays_.size();}
  const PatternRay & operator[](std::size_t i) const {return rays_[i % rays_.size()];}
  float MinElevation() const {return min_el_;}
  float MaxElevation() const {return max_el_;}

private:
  void Finish();
  std::vector<PatternRay> rays_;
  float min_el_{0.f}, max_el_{0.f};
};

/// Resolve "model://a/b", "file://..." or plain paths. model:// is searched in
/// GZ_SIM_RESOURCE_PATH (then GZ_FILE_PATH / SDF_PATH). Returns "" if not found.
std::string ResolveResourceUri(const std::string & uri);

}  // namespace livox_mid360_sim
