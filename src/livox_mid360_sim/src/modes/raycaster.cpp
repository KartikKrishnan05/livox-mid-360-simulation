// Copyright 2026 Kartik Krishnan
// SPDX-License-Identifier: Apache-2.0

#include "raycaster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/LaserRetro.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/Visual.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

namespace livox_mid360_sim::modes
{

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr uint32_t kLeafSize = 4;
constexpr int kBins = 16;
constexpr int kMaxDepth = 64;
constexpr int kStackSize = 2 * kMaxDepth + 8;
constexpr double kPi = 3.14159265358979323846;

// Tessellation of curved primitives. 64 segments put an inscribed facet at
// most r * (1 - cos(pi / 64)) = 0.12 % of the radius inside the true surface.
constexpr int kSegments = 64;
constexpr int kRings = 32;

inline Vec3f Sub(const Vec3f & a, const Vec3f & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline float Dot(const Vec3f & a, const Vec3f & b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
inline Vec3f Cross(const Vec3f & a, const Vec3f & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float Get(const Vec3f & v, int axis) {return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);}

float HalfArea(const float * bmin, const float * bmax)
{
  const float dx = bmax[0] - bmin[0], dy = bmax[1] - bmin[1], dz = bmax[2] - bmin[2];
  return dx * dy + dy * dz + dz * dx;
}

void Grow(float * bmin, float * bmax, const float * omin, const float * omax)
{
  for (int a = 0; a < 3; ++a) {
    bmin[a] = std::min(bmin[a], omin[a]);
    bmax[a] = std::max(bmax[a], omax[a]);
  }
}

// ---- tessellation -----------------------------------------------------------

using Tris = std::vector<Vec3f>;

void AddTri(
  Tris & out, const gz::math::Vector3d & a, const gz::math::Vector3d & b,
  const gz::math::Vector3d & c)
{
  out.push_back({static_cast<float>(a.X()), static_cast<float>(a.Y()), static_cast<float>(a.Z())});
  out.push_back({static_cast<float>(b.X()), static_cast<float>(b.Y()), static_cast<float>(b.Z())});
  out.push_back({static_cast<float>(c.X()), static_cast<float>(c.Y()), static_cast<float>(c.Z())});
}

void AddQuad(
  Tris & out, const gz::math::Vector3d & a, const gz::math::Vector3d & b,
  const gz::math::Vector3d & c, const gz::math::Vector3d & d)
{
  AddTri(out, a, b, c);
  AddTri(out, a, c, d);
}

Tris BoxTris(const gz::math::Vector3d & size)
{
  const gz::math::Vector3d h = size / 2.0;
  gz::math::Vector3d c[8];
  for (int i = 0; i < 8; ++i) {
    c[i].Set((i & 1) ? h.X() : -h.X(), (i & 2) ? h.Y() : -h.Y(), (i & 4) ? h.Z() : -h.Z());
  }
  const int faces[6][4] = {
    {0, 2, 6, 4}, {1, 3, 7, 5}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 5, 7, 6}};
  Tris out;
  for (const auto & f : faces) {
    AddQuad(out, c[f[0]], c[f[1]], c[f[2]], c[f[3]]);
  }
  return out;
}

// Part of an ellipsoid between polar angles theta0 and theta1 (0 = +z), centred at z0.
void AddEllipsoidBand(
  Tris & out, const gz::math::Vector3d & radii, double theta0, double theta1, int rings,
  double z0)
{
  auto p = [&](int i, int j) {
      const double th = theta0 + (theta1 - theta0) * i / rings;
      const double ph = 2.0 * kPi * j / kSegments;
      return gz::math::Vector3d(
        radii.X() * std::sin(th) * std::cos(ph),
        radii.Y() * std::sin(th) * std::sin(ph),
        z0 + radii.Z() * std::cos(th));
    };
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < kSegments; ++j) {
      AddQuad(out, p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1));
    }
  }
}

// Side of a cylinder / cone along z from z = -length/2 (radius r0) to +length/2 (radius r1).
void AddSide(Tris & out, double r0, double r1, double length, bool caps)
{
  const double z0 = -length / 2.0, z1 = length / 2.0;
  for (int j = 0; j < kSegments; ++j) {
    const double a = 2.0 * kPi * j / kSegments, b = 2.0 * kPi * (j + 1) / kSegments;
    const gz::math::Vector3d a0(r0 * std::cos(a), r0 * std::sin(a), z0);
    const gz::math::Vector3d b0(r0 * std::cos(b), r0 * std::sin(b), z0);
    const gz::math::Vector3d a1(r1 * std::cos(a), r1 * std::sin(a), z1);
    const gz::math::Vector3d b1(r1 * std::cos(b), r1 * std::sin(b), z1);
    AddQuad(out, a0, b0, b1, a1);
    if (caps) {
      AddTri(out, gz::math::Vector3d(0, 0, z0), b0, a0);
      if (r1 > 0.0) {
        AddTri(out, gz::math::Vector3d(0, 0, z1), a1, b1);
      }
    }
  }
}

Tris PlaneTris(const gz::math::Vector3d & normal, const gz::math::Vector2d & size)
{
  gz::math::Quaterniond q;
  q.SetFrom2Axes(gz::math::Vector3d::UnitZ, normal.Normalized());
  const double hx = size.X() / 2.0, hy = size.Y() / 2.0;
  Tris out;
  AddQuad(
    out, q * gz::math::Vector3d(-hx, -hy, 0), q * gz::math::Vector3d(hx, -hy, 0),
    q * gz::math::Vector3d(hx, hy, 0), q * gz::math::Vector3d(-hx, hy, 0));
  return out;
}

bool MeshTris(const sdf::Mesh & mesh_sdf, Tris & out, std::string & error)
{
  const gz::common::Mesh * mesh = gz::sim::loadMesh(mesh_sdf);
  if (!mesh) {
    error = "cannot load mesh [" + mesh_sdf.Uri() + "]";
    return false;
  }
  const gz::math::Vector3d scale = mesh_sdf.Scale();
  const std::string & only = mesh_sdf.Submesh();
  bool skipped = false;
  for (unsigned int s = 0; s < mesh->SubMeshCount(); ++s) {
    auto sub = mesh->SubMeshByIndex(s).lock();
    if (!sub || (!only.empty() && sub->Name() != only)) {
      continue;
    }
    if (sub->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) {
      skipped = true;
      continue;
    }
    gz::math::Vector3d offset = gz::math::Vector3d::Zero;
    if (!only.empty() && mesh_sdf.CenterSubmesh()) {
      offset = (sub->Min() + sub->Max()) / 2.0;
    }
    auto vertex = [&](unsigned int k) {
        const unsigned int idx = sub->IndexCount() > 0 ?
          static_cast<unsigned int>(sub->Index(k)) : k;
        return (sub->Vertex(idx) - offset) * scale;
      };
    const unsigned int n = sub->IndexCount() > 0 ? sub->IndexCount() : sub->VertexCount();
    for (unsigned int k = 0; k + 2 < n; k += 3) {
      AddTri(out, vertex(k), vertex(k + 1), vertex(k + 2));
    }
  }
  if (skipped) {
    gzwarn << "[LivoxMid360] mesh [" << mesh_sdf.Uri()
           << "]: submeshes that are not triangle lists are ignored by the ray caster\n";
  }
  if (out.empty()) {
    error = "mesh [" + mesh_sdf.Uri() + "] has no triangles";
    return false;
  }
  return true;
}

// Tessellate a geometry; `key` identifies identical shapes for sharing.
bool Tessellate(const sdf::Geometry & g, Tris & out, std::string & key, std::string & error)
{
  std::ostringstream k;
  k.precision(9);
  switch (g.Type()) {
    case sdf::GeometryType::BOX:
      k << "box " << g.BoxShape()->Size();
      out = BoxTris(g.BoxShape()->Size());
      break;
    case sdf::GeometryType::SPHERE: {
        const double r = g.SphereShape()->Radius();
        k << "sphere " << r;
        AddEllipsoidBand(out, {r, r, r}, 0.0, kPi, kRings, 0.0);
        break;
      }
    case sdf::GeometryType::ELLIPSOID:
      k << "ellipsoid " << g.EllipsoidShape()->Radii();
      AddEllipsoidBand(out, g.EllipsoidShape()->Radii(), 0.0, kPi, kRings, 0.0);
      break;
    case sdf::GeometryType::CYLINDER: {
        const double r = g.CylinderShape()->Radius(), l = g.CylinderShape()->Length();
        k << "cylinder " << r << " " << l;
        AddSide(out, r, r, l, true);
        break;
      }
    case sdf::GeometryType::CONE: {
        const double r = g.ConeShape()->Radius(), l = g.ConeShape()->Length();
        k << "cone " << r << " " << l;
        AddSide(out, r, 0.0, l, true);
        break;
      }
    case sdf::GeometryType::CAPSULE: {
        const double r = g.CapsuleShape()->Radius(), l = g.CapsuleShape()->Length();
        k << "capsule " << r << " " << l;
        AddSide(out, r, r, l, false);
        AddEllipsoidBand(out, {r, r, r}, 0.0, kPi / 2, kRings / 2, l / 2);
        AddEllipsoidBand(out, {r, r, r}, kPi / 2, kPi, kRings / 2, -l / 2);
        break;
      }
    case sdf::GeometryType::PLANE:
      k << "plane " << g.PlaneShape()->Normal() << " " << g.PlaneShape()->Size();
      out = PlaneTris(g.PlaneShape()->Normal(), g.PlaneShape()->Size());
      break;
    case sdf::GeometryType::MESH: {
        const sdf::Mesh & m = *g.MeshShape();
        k << "mesh " << m.FilePath() << "|" << m.Uri() << "|" << m.Scale() << "|" << m.Submesh()
          << "|" << m.CenterSubmesh();
        key = k.str();
        return MeshTris(m, out, error);
      }
    case sdf::GeometryType::HEIGHTMAP:
      error = "heightmaps are not supported by the ray caster (use a mesh)";
      return false;
    default:
      error = "geometry type " + std::to_string(static_cast<int>(g.Type())) +
        " is not supported by the ray caster";
      return false;
  }
  key = k.str();
  return true;
}
}  // namespace

// ---- TriangleBvh ------------------------------------------------------------

TriangleBvh::TriangleBvh(const std::vector<Vec3f> & triangles)
{
  const uint32_t n = static_cast<uint32_t>(triangles.size() / 3);
  std::vector<Tri> raw(n);
  build_.resize(n);
  order_.resize(n);
  for (uint32_t i = 0; i < n; ++i) {
    const Vec3f & a = triangles[3 * i], & b = triangles[3 * i + 1], & c = triangles[3 * i + 2];
    raw[i] = {a, Sub(b, a), Sub(c, a)};
    BuildTri & bt = build_[i];
    for (int ax = 0; ax < 3; ++ax) {
      bt.bmin[ax] = std::min({Get(a, ax), Get(b, ax), Get(c, ax)});
      bt.bmax[ax] = std::max({Get(a, ax), Get(b, ax), Get(c, ax)});
      bt.c[ax] = 0.5f * (bt.bmin[ax] + bt.bmax[ax]);
    }
    order_[i] = i;
  }
  if (n > 0) {
    nodes_.reserve(2 * static_cast<std::size_t>(n));
    nodes_.push_back(Node{});
    Subdivide(0, 0, n, 0);
    tris_.reserve(n);
    for (uint32_t i : order_) {
      tris_.push_back(raw[i]);
    }
  }
  build_.clear();
  build_.shrink_to_fit();
  order_.clear();
  order_.shrink_to_fit();
}

void TriangleBvh::Subdivide(uint32_t node, uint32_t first, uint32_t count, int depth)
{
  float bmin[3] = {kInf, kInf, kInf}, bmax[3] = {-kInf, -kInf, -kInf};
  float cmin[3] = {kInf, kInf, kInf}, cmax[3] = {-kInf, -kInf, -kInf};
  for (uint32_t i = first; i < first + count; ++i) {
    const BuildTri & bt = build_[order_[i]];
    Grow(bmin, bmax, bt.bmin, bt.bmax);
    Grow(cmin, cmax, bt.c, bt.c);
  }
  std::copy(bmin, bmin + 3, nodes_[node].bmin);
  std::copy(bmax, bmax + 3, nodes_[node].bmax);
  nodes_[node].first = first;
  nodes_[node].count = count;
  if (count <= kLeafSize || depth >= kMaxDepth) {
    return;
  }

  // Binned surface area heuristic over all three axes.
  float best_cost = kInf;
  int best_axis = -1, best_split = 0;
  for (int ax = 0; ax < 3; ++ax) {
    const float extent = cmax[ax] - cmin[ax];
    if (!(extent > 1e-9f)) {
      continue;
    }
    struct Bin
    {
      float bmin[3] = {kInf, kInf, kInf}, bmax[3] = {-kInf, -kInf, -kInf};
      uint32_t n = 0;
    } bins[kBins];
    const float scale = kBins / extent;
    for (uint32_t i = first; i < first + count; ++i) {
      const BuildTri & bt = build_[order_[i]];
      const int b = std::min(kBins - 1, static_cast<int>((bt.c[ax] - cmin[ax]) * scale));
      Grow(bins[b].bmin, bins[b].bmax, bt.bmin, bt.bmax);
      ++bins[b].n;
    }
    float left_area[kBins - 1];
    uint32_t left_n[kBins - 1];
    float lmin[3] = {kInf, kInf, kInf}, lmax[3] = {-kInf, -kInf, -kInf};
    uint32_t ln = 0;
    for (int s = 0; s < kBins - 1; ++s) {
      if (bins[s].n) {
        Grow(lmin, lmax, bins[s].bmin, bins[s].bmax);
      }
      ln += bins[s].n;
      left_n[s] = ln;
      left_area[s] = ln ? HalfArea(lmin, lmax) : 0.0f;
    }
    float rmin[3] = {kInf, kInf, kInf}, rmax[3] = {-kInf, -kInf, -kInf};
    uint32_t rn = 0;
    for (int s = kBins - 2; s >= 0; --s) {
      if (bins[s + 1].n) {
        Grow(rmin, rmax, bins[s + 1].bmin, bins[s + 1].bmax);
      }
      rn += bins[s + 1].n;
      if (left_n[s] == 0 || rn == 0) {
        continue;
      }
      const float cost = left_area[s] * left_n[s] + HalfArea(rmin, rmax) * rn;
      if (cost < best_cost) {
        best_cost = cost;
        best_axis = ax;
        best_split = s;
      }
    }
  }
  if (best_axis < 0 || (best_cost >= HalfArea(bmin, bmax) * count && count <= 4 * kLeafSize)) {
    return;  // leaf
  }

  const float scale = kBins / (cmax[best_axis] - cmin[best_axis]);
  const float c0 = cmin[best_axis];
  auto mid = std::partition(
    order_.begin() + first, order_.begin() + first + count, [&](uint32_t t) {
      return std::min(kBins - 1, static_cast<int>((build_[t].c[best_axis] - c0) * scale)) <=
      best_split;
    });
  const uint32_t left_count = static_cast<uint32_t>(mid - (order_.begin() + first));
  if (left_count == 0 || left_count == count) {
    return;  // leaf
  }
  const uint32_t left = static_cast<uint32_t>(nodes_.size());
  nodes_.push_back(Node{});
  nodes_.push_back(Node{});
  nodes_[node].first = left;
  nodes_[node].count = 0;
  Subdivide(left, first, left_count, depth + 1);
  Subdivide(left + 1, first + left_count, count - left_count, depth + 1);
}

bool TriangleBvh::Intersect(
  const Vec3f & o, const Vec3f & d, float tmin, float & tmax) const
{
  if (tris_.empty()) {
    return false;
  }
  auto inv = [](float v) {return 1.0f / (std::fabs(v) > 1e-20f ? v : std::copysign(1e-20f, v));};
  const float ix = inv(d.x), iy = inv(d.y), iz = inv(d.z);
  auto slab = [&](const Node & n, float & tnear) {
      float t1 = (n.bmin[0] - o.x) * ix, t2 = (n.bmax[0] - o.x) * ix;
      float lo = std::min(t1, t2), hi = std::max(t1, t2);
      t1 = (n.bmin[1] - o.y) * iy;
      t2 = (n.bmax[1] - o.y) * iy;
      lo = std::max(lo, std::min(t1, t2));
      hi = std::min(hi, std::max(t1, t2));
      t1 = (n.bmin[2] - o.z) * iz;
      t2 = (n.bmax[2] - o.z) * iz;
      lo = std::max(lo, std::min(t1, t2));
      hi = std::min(hi, std::max(t1, t2));
      tnear = std::max(lo, tmin);
      return tnear <= std::min(hi, tmax);
    };

  struct Entry
  {
    uint32_t node;
    float tnear;
  } stack[kStackSize];
  int sp = 0;
  float t0;
  if (!slab(nodes_[0], t0)) {
    return false;
  }
  stack[sp++] = {0, t0};
  bool hit = false;
  while (sp > 0) {
    const Entry e = stack[--sp];
    if (e.tnear > tmax) {
      continue;  // a closer hit was found after this node was pushed
    }
    const Node & n = nodes_[e.node];
    if (n.count > 0) {
      for (uint32_t i = n.first; i < n.first + n.count; ++i) {
        // Moller-Trumbore, two-sided
        const Tri & tri = tris_[i];
        const Vec3f p = Cross(d, tri.e2);
        const float det = Dot(tri.e1, p);
        if (std::fabs(det) < 1e-12f) {
          continue;
        }
        const float inv_det = 1.0f / det;
        const Vec3f s = Sub(o, tri.v0);
        const float u = Dot(s, p) * inv_det;
        if (u < 0.0f || u > 1.0f) {
          continue;
        }
        const Vec3f q = Cross(s, tri.e1);
        const float v = Dot(d, q) * inv_det;
        if (v < 0.0f || u + v > 1.0f) {
          continue;
        }
        const float t = Dot(tri.e2, q) * inv_det;
        if (t >= tmin && t < tmax) {
          tmax = t;
          hit = true;
        }
      }
      continue;
    }
    float ta, tb;
    const bool ha = slab(nodes_[n.first], ta), hb = slab(nodes_[n.first + 1], tb);
    if (ha && hb) {
      // push the far child first so the near one is visited first
      if (ta <= tb) {
        stack[sp++] = {n.first + 1, tb};
        stack[sp++] = {n.first, ta};
      } else {
        stack[sp++] = {n.first, ta};
        stack[sp++] = {n.first + 1, tb};
      }
    } else if (ha) {
      stack[sp++] = {n.first, ta};
    } else if (hb) {
      stack[sp++] = {n.first + 1, tb};
    }
  }
  return hit;
}

// ---- Scene ------------------------------------------------------------------

std::shared_ptr<const TriangleBvh> Scene::ShapeFor(const sdf::Geometry & geometry)
{
  Tris tris;
  std::string key, error;
  const bool ok = Tessellate(geometry, tris, key, error);
  if (!key.empty()) {
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      return it->second;
    }
  }
  if (!ok) {
    gzwarn << "[LivoxMid360] ray caster ignores a visual: " << error << "\n";
    if (!key.empty()) {
      cache_[key] = nullptr;
    }
    return nullptr;
  }
  auto bvh = std::make_shared<const TriangleBvh>(tris);
  cache_[key] = bvh;
  return bvh;
}

void Scene::Rebuild(const gz::sim::EntityComponentManager & ecm)
{
  instances_.clear();
  std::size_t triangles = 0;
  ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
    [&](const gz::sim::Entity & entity, const gz::sim::components::Visual *,
    const gz::sim::components::Geometry * geometry) {
      auto bvh = ShapeFor(geometry->Data());
      if (!bvh) {
        return true;
      }
      Instance inst;
      inst.visual = entity;
      inst.bvh = bvh;
      if (auto retro = ecm.Component<gz::sim::components::LaserRetro>(entity)) {
        inst.retro = retro->Data();
      }
      const gz::sim::Entity model = gz::sim::topLevelModel(entity, ecm);
      if (auto is_static = ecm.Component<gz::sim::components::Static>(model)) {
        inst.is_static = is_static->Data();
      }
      inst.world = gz::sim::worldPose(entity, ecm);
      inst.inv_rot = inst.world.Rot().Inverse();
      triangles += bvh->TriangleCount();
      instances_.push_back(std::move(inst));
      return true;
    });
  built_ = true;
  gzmsg << "[LivoxMid360] ray caster scene: " << instances_.size() << " visuals, "
        << triangles << " triangles\n";
}

void Scene::Update(const gz::sim::EntityComponentManager & ecm)
{
  bool rebuild = !built_ || ecm.HasNewEntities() || ecm.HasEntitiesMarkedForRemoval();
  for (std::size_t i = 0; !rebuild && i < instances_.size(); ++i) {
    rebuild = !ecm.HasEntity(instances_[i].visual);
  }
  if (rebuild) {
    Rebuild(ecm);
    return;
  }
  for (auto & inst : instances_) {
    if (!inst.is_static) {
      inst.world = gz::sim::worldPose(inst.visual, ecm);
      inst.inv_rot = inst.world.Rot().Inverse();
    }
  }
}

bool Scene::Intersect(
  const gz::math::Vector3d & origin, const gz::math::Vector3d & dir, double tmin, double tmax,
  double & range, double & retro) const
{
  float best = static_cast<float>(tmax);
  bool hit = false;
  for (const auto & inst : instances_) {
    const gz::math::Vector3d lo = inst.inv_rot * (origin - inst.world.Pos());
    const gz::math::Vector3d ld = inst.inv_rot * dir;
    const Vec3f o{static_cast<float>(lo.X()), static_cast<float>(lo.Y()),
      static_cast<float>(lo.Z())};
    const Vec3f d{static_cast<float>(ld.X()), static_cast<float>(ld.Y()),
      static_cast<float>(ld.Z())};
    if (inst.bvh->Intersect(o, d, static_cast<float>(tmin), best)) {
      hit = true;
      retro = inst.retro;
    }
  }
  range = best;
  return hit;
}

}  // namespace livox_mid360_sim::modes
