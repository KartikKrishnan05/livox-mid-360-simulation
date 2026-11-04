#include "scan_pattern.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace livox_mid360_gazebo
{
namespace
{
constexpr double kDeg = M_PI / 180.0;

bool FileExists(const std::string & p)
{
  std::ifstream f(p);
  return f.good();
}

std::vector<std::string> SplitPaths(const char * env)
{
  std::vector<std::string> out;
  if (!env) {return out;}
  std::stringstream ss(env);
  std::string item;
  while (std::getline(ss, item, ':')) {
    if (!item.empty()) {out.push_back(item);}
  }
  return out;
}

PatternRay MakeRay(double az, double el, uint8_t line)
{
  az = std::fmod(az, 2.0 * M_PI);
  if (az < 0) {az += 2.0 * M_PI;}
  PatternRay r;
  r.dx = static_cast<float>(std::cos(el) * std::cos(az));
  r.dy = static_cast<float>(std::cos(el) * std::sin(az));
  r.dz = static_cast<float>(std::sin(el));
  r.az = static_cast<float>(az);
  r.el = static_cast<float>(el);
  r.line = line;
  return r;
}
}  // namespace

std::string ResolveResourceUri(const std::string & uri)
{
  if (uri.empty()) {return "";}
  const std::string model = "model://";
  const std::string file = "file://";
  if (uri.rfind(file, 0) == 0) {
    const std::string p = uri.substr(file.size());
    return FileExists(p) ? p : "";
  }
  if (uri.rfind(model, 0) != 0) {
    return FileExists(uri) ? uri : "";
  }
  const std::string rel = uri.substr(model.size());
  for (const char * var : {"GZ_SIM_RESOURCE_PATH", "GZ_FILE_PATH", "SDF_PATH"}) {
    for (const auto & dir : SplitPaths(std::getenv(var))) {
      const std::string p = dir + "/" + rel;
      if (FileExists(p)) {return p;}
    }
  }
  return "";
}

bool ScanPattern::LoadCsv(const std::string & path, std::string * error)
{
  std::FILE * f = std::fopen(path.c_str(), "rb");
  if (!f) {
    if (error) {*error = "cannot open " + path;}
    return false;
  }
  std::fseek(f, 0, SEEK_END);
  const long len = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  std::string buf(static_cast<std::size_t>(len), '\0');
  const std::size_t got = std::fread(buf.data(), 1, buf.size(), f);
  std::fclose(f);
  buf.resize(got);

  rays_.clear();
  rays_.reserve(800000);
  const char * p = buf.c_str();
  const char * end = p + buf.size();
  // Skip the header line.
  while (p < end && *p != '\n') {++p;}
  std::size_t row = 0;
  while (p < end) {
    while (p < end && (*p == '\n' || *p == '\r')) {++p;}
    if (p >= end) {break;}
    char * q;
    std::strtod(p, &q);                       // index column (ignored)
    if (q == p || *q != ',') {break;}
    p = q + 1;
    const double az_deg = std::strtod(p, &q);
    if (q == p || *q != ',') {break;}
    p = q + 1;
    const double zen_deg = std::strtod(p, &q);
    if (q == p) {break;}
    p = q;
    while (p < end && *p != '\n') {++p;}
    rays_.push_back(MakeRay(az_deg * kDeg, (90.0 - zen_deg) * kDeg,
      static_cast<uint8_t>(row % 4)));
    ++row;
  }
  if (rays_.size() < 4) {
    if (error) {*error = "no rows parsed from " + path;}
    rays_.clear();
    return false;
  }
  // Keep whole groups of 4 so line = index % 4 stays valid when wrapping.
  rays_.resize(rays_.size() / 4 * 4);
  Finish();
  return true;
}

void ScanPattern::MakeProcedural(std::size_t num_points)
{
  rays_.clear();
  num_points = std::max<std::size_t>(4, num_points / 4 * 4);
  rays_.reserve(num_points);
  const double lower = -7.0 * kDeg;
  const double span = 50.0 * kDeg;           // lowest laser covers -7..43, top one -> 52
  const double spacing = 3.0 * kDeg;
  const double daz = 1.1713 * kDeg;          // per group of 4, ~16 rev per 100 ms frame
  const double golden = 0.6180339887498949;  // irrational -> no exact repetition
  for (std::size_t g = 0; g < num_points / 4; ++g) {
    const double phase = std::fmod(g * golden * 0.0137, 1.0);
    const double base = lower + span * (0.5 - 0.5 * std::cos(2.0 * M_PI * phase));
    const double az = g * daz + std::fmod(g * golden, 1.0) * 0.05;
    for (uint8_t l = 0; l < 4; ++l) {
      rays_.push_back(MakeRay(az, std::min(base + l * spacing, 52.0 * kDeg), l));
    }
  }
  Finish();
}

void ScanPattern::Finish()
{
  min_el_ = 1e9f;
  max_el_ = -1e9f;
  for (const auto & r : rays_) {
    min_el_ = std::min(min_el_, r.el);
    max_el_ = std::max(max_el_, r.el);
  }
}

}  // namespace livox_mid360_gazebo
