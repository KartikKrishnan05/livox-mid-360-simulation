// A dense gz gpu_lidar render (gz.msgs.LaserScan) and sampling of it along arbitrary
// directions. Layout (verified against the ground plane, see README):
//   ranges[v * count + h], h = 0 at angle_min (CCW), v = 0 at vertical_angle_min.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <gz/msgs/laserscan.pb.h>

namespace livox_mid360_sim
{

struct DenseScan
{
  int64_t stamp_ns{0};
  int nh{0}, nv{0};
  double hmin{0}, hstep{0}, vmin{0}, vstep{0};
  bool wrap{false};
  std::vector<float> range;
  std::vector<float> retro;

  static std::shared_ptr<DenseScan> FromMsg(const gz::msgs::LaserScan & m)
  {
    auto s = std::make_shared<DenseScan>();
    s->stamp_ns = m.header().stamp().sec() * 1000000000LL + m.header().stamp().nsec();
    s->nh = static_cast<int>(m.count());
    s->nv = std::max(1, static_cast<int>(m.vertical_count()));
    s->hmin = m.angle_min();
    s->hstep = m.angle_step();
    s->vmin = m.vertical_angle_min();
    s->vstep = m.vertical_angle_step();
    s->wrap = std::abs(s->nh * s->hstep - 2.0 * M_PI) < 0.5 * std::abs(s->hstep);
    const std::size_t n = static_cast<std::size_t>(s->nh) * s->nv;
    if (static_cast<std::size_t>(m.ranges_size()) < n || s->nh < 2) {return nullptr;}
    s->range.resize(n);
    s->retro.assign(n, 0.f);
    for (std::size_t i = 0; i < n; ++i) {s->range[i] = static_cast<float>(m.ranges(i));}
    if (static_cast<std::size_t>(m.intensities_size()) >= n) {
      for (std::size_t i = 0; i < n; ++i) {s->retro[i] = static_cast<float>(m.intensities(i));}
    }
    return s;
  }

  float R(int v, int h) const {return range[static_cast<std::size_t>(v) * nh + h];}

  /// Range along (az, el). Bilinear (in 1/r, exact for planes to first order) when the
  /// four neighbours are hits on one continuous surface (spread <= rel_edge * r_min),
  /// nearest neighbour otherwise (edges, mixed hit/miss). False = no return.
  bool Sample(double az, double el, double rel_edge, double & r_out, double & retro_out) const
  {
    double fh = (az - hmin) / hstep;
    if (wrap) {
      fh = std::fmod(fh, static_cast<double>(nh));
      if (fh < 0) {fh += nh;}
    } else if (fh < -0.5 || fh > nh - 0.5) {
      return false;
    }
    double fv = nv > 1 ? (el - vmin) / vstep : 0.0;
    if (fv < -0.5 || fv > nv - 0.5) {return false;}
    fh = std::max(0.0, fh);
    fv = std::clamp(fv, 0.0, static_cast<double>(nv - 1));
    int h0 = static_cast<int>(fh);
    int v0 = static_cast<int>(fv);
    const double a = fh - h0;
    const double b = fv - v0;
    int h1 = h0 + 1;
    if (h1 >= nh) {h1 = wrap ? 0 : nh - 1;}
    if (h0 >= nh) {h0 = wrap ? 0 : nh - 1;}
    const int v1 = std::min(v0 + 1, nv - 1);

    const float r00 = R(v0, h0), r01 = R(v0, h1), r10 = R(v1, h0), r11 = R(v1, h1);
    const int hn = a < 0.5 ? h0 : h1;
    const int vn = b < 0.5 ? v0 : v1;
    retro_out = retro[static_cast<std::size_t>(vn) * nh + hn];
    if (std::isfinite(r00) && std::isfinite(r01) && std::isfinite(r10) && std::isfinite(r11) &&
      r00 > 0 && r01 > 0 && r10 > 0 && r11 > 0)
    {
      const float lo = std::min(std::min(r00, r01), std::min(r10, r11));
      const float hi = std::max(std::max(r00, r01), std::max(r10, r11));
      if (hi - lo <= rel_edge * lo) {
        const double inv = (1 - b) * ((1 - a) / r00 + a / r01) + b * ((1 - a) / r10 + a / r11);
        r_out = 1.0 / inv;
        return true;
      }
    }
    const float rn = R(vn, hn);
    if (!std::isfinite(rn) || rn <= 0) {return false;}
    r_out = rn;
    return true;
  }
};

}  // namespace livox_mid360_sim
