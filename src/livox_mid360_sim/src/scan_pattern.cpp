// Copyright 2026 Kartik Krishnan
// SPDX-License-Identifier: Apache-2.0

#include "livox_mid360_sim/scan_pattern.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace livox_mid360_sim
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double WrapPi(double a)
{
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0) {
    a += kTwoPi;
  }
  return a - kPi;
}

// Triangle wave in [0, 1] with period 1.
double Triangle(double x)
{
  const double f = x - std::floor(x);
  return 1.0 - std::fabs(2.0 * f - 1.0);
}
}  // namespace

ScanPattern::ScanPattern(const Params & params)
: params_(params)
{
  if (params_.num_lines < 1) {
    params_.num_lines = 1;
  }
}

std::string ScanPattern::ResolvePath(const std::string & uri)
{
  const std::string prefix = "model://";
  if (uri.rfind(prefix, 0) != 0) {
    return uri;
  }
  const std::string rel = uri.substr(prefix.size());
  for (const char * var : {"GZ_SIM_RESOURCE_PATH", "SDF_PATH"}) {
    const char * value = std::getenv(var);
    if (!value) {
      continue;
    }
    std::stringstream dirs(value);
    std::string dir;
    while (std::getline(dirs, dir, ':')) {
      if (dir.empty()) {
        continue;
      }
      const std::string candidate = dir + "/" + rel;
      if (std::ifstream(candidate).good()) {
        return candidate;
      }
    }
  }
  return "";
}

bool ScanPattern::LoadCsv(const std::string & path, std::string * error)
{
  std::ifstream file(path);
  if (!file) {
    if (error) {*error = "cannot open " + path;}
    return false;
  }

  std::vector<PatternRay> rays;
  std::string row;
  while (std::getline(file, row)) {
    for (auto & c : row) {
      if (c == ',' || c == ';' || c == '\t') {c = ' ';}
    }
    std::istringstream ss(row);
    double time = 0.0, azimuth_deg = 0.0, zenith_deg = 0.0;
    if (!(ss >> time >> azimuth_deg >> zenith_deg)) {
      continue;  // header or malformed line
    }
    PatternRay ray;
    ray.azimuth = WrapPi(azimuth_deg * kPi / 180.0);
    ray.elevation = (90.0 - zenith_deg) * kPi / 180.0;
    ray.line = static_cast<uint8_t>(rays.size() % params_.num_lines);
    rays.push_back(ray);
  }

  if (rays.empty()) {
    if (error) {*error = "no rays found in " + path;}
    return false;
  }
  csv_rays_ = std::move(rays);
  cursor_ = 0;
  return true;
}

void ScanPattern::Next(std::size_t count, std::vector<PatternRay> & out)
{
  out.reserve(out.size() + count);
  for (std::size_t i = 0; i < count; ++i, ++cursor_) {
    if (!csv_rays_.empty()) {
      out.push_back(csv_rays_[cursor_ % csv_rays_.size()]);
    } else {
      out.push_back(Procedural(cursor_));
    }
  }
}

PatternRay ScanPattern::Procedural(uint64_t index) const
{
  const int lines = params_.num_lines;
  const uint8_t line = static_cast<uint8_t>(index % lines);
  // All lasers of one group fire at (almost) the same azimuth, stacked in
  // elevation, like the rows of the real Mid-360 pattern.
  const uint64_t group = index / lines;
  const double group_rate = params_.points_per_second / lines;
  const double revolutions = params_.rotation_rate * static_cast<double>(group) / group_rate;

  const double stack = params_.line_spacing * (lines - 1);
  const double span = std::max(0.0, params_.elevation_max - params_.elevation_min - stack);

  PatternRay ray;
  ray.line = line;
  ray.azimuth = WrapPi(kTwoPi * revolutions);
  ray.elevation = params_.elevation_min + params_.line_spacing * line +
    span * Triangle(params_.sweeps_per_revolution * revolutions);
  return ray;
}

}  // namespace livox_mid360_sim
