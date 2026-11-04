// Copyright 2026 Kartik Krishnan
// SPDX-License-Identifier: Apache-2.0

#ifndef LIVOX_MID360_SIM__SCAN_PATTERN_HPP_
#define LIVOX_MID360_SIM__SCAN_PATTERN_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace livox_mid360_sim::modes
{

/// One emitted laser shot of the Livox scan pattern.
struct PatternRay
{
  double azimuth;    ///< [rad], 0 = +x, counter-clockwise, in [-pi, pi)
  double elevation;  ///< [rad], 0 = horizon, positive = up
  uint8_t line;      ///< emitter / line index (Mid-360 has 4)
};

/// Generates the non-repetitive scan pattern of a Livox Mid-360.
///
/// The real pattern is loaded from a CSV file in the format used by Livox'
/// `livox_laser_simulation` (`index, azimuth_deg, zenith_deg` per row, zenith
/// measured from +z). Rows come in groups of `num_lines` stacked lasers, so
/// `line = row % num_lines`. The package ships the Mid-360 pattern as
/// `model://livox_mid360/scan_mode/mid360.csv`.
///
/// Without a CSV a procedural fallback is used: `num_lines` lasers stacked
/// `line_spacing` apart in elevation spin around z at `rotation_rate` while
/// sweeping the vertical FOV with a triangle wave whose frequency is an
/// irrational multiple of the rotation rate, so frames never repeat.
class ScanPattern
{
public:
  struct Params
  {
    double points_per_second = 200000.0;
    double rotation_rate = 10.0;           ///< [Hz] azimuth revolutions per second
    double elevation_min = -0.12217305;    ///< [rad] -7 deg
    double elevation_max = 0.90757121;     ///< [rad] +52 deg
    int num_lines = 4;
    /// Vertical sweeps per revolution of one line. Keep it irrational so
    /// the pattern does not repeat.
    double sweeps_per_revolution = 61.80339887498949;
    double line_spacing = 0.05236;         ///< [rad] ~3 deg between stacked lasers
  };

  /// Resolve `model://name/...` against GZ_SIM_RESOURCE_PATH (and SDF_PATH).
  /// Other paths are returned unchanged. Returns "" if a model URI is not found.
  static std::string ResolvePath(const std::string & uri);

  explicit ScanPattern(const Params & params);

  /// Load a recorded pattern. Returns false (and keeps the procedural
  /// pattern) if the file cannot be parsed.
  bool LoadCsv(const std::string & path, std::string * error = nullptr);

  /// Append the next `count` rays of the pattern to `out`.
  void Next(std::size_t count, std::vector<PatternRay> & out);

  std::size_t CsvSize() const {return csv_rays_.size();}

private:
  PatternRay Procedural(uint64_t index) const;

  Params params_;
  uint64_t cursor_ = 0;
  std::vector<PatternRay> csv_rays_;
};

}  // namespace livox_mid360_sim::modes

#endif  // LIVOX_MID360_SIM__SCAN_PATTERN_HPP_
