#include "raycaster.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>

namespace livox_mid360_sim
{
namespace
{
constexpr int kLeafSize = 4;

// Slab test; returns entry distance in [tmin, tmax] or false.
inline bool HitBox(const Aabb & b, const Vector3d & o, const Vector3d & inv, double tmin,
  double tmax)
{
  for (int a = 0; a < 3; ++a) {
    double t0 = (b.lo[a] - o[a]) * inv[a];
    double t1 = (b.hi[a] - o[a]) * inv[a];
    if (t0 > t1) {std::swap(t0, t1);}
    if (std::isnan(t0) || std::isnan(t1)) {      // o on the slab plane, d parallel
      if (o[a] < b.lo[a] || o[a] > b.hi[a]) {return false;}
      continue;
    }
    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);
    if (tmax < tmin) {return false;}
  }
  return true;
}

Vector3d Inverse(const Vector3d & d)
{
  auto inv = [](double x) {
      return x != 0.0 ? 1.0 / x : std::copysign(std::numeric_limits<double>::infinity(), x);
    };
  return {inv(d.X()), inv(d.Y()), inv(d.Z())};
}

// Generic median-split BVH over boxes. NodeT needs box/left/first/count.
template<typename NodeT>
void BuildBvh(const std::vector<Aabb> & boxes, std::vector<NodeT> & nodes,
  std::vector<int> & order)
{
  nodes.clear();
  order.resize(boxes.size());
  std::iota(order.begin(), order.end(), 0);
  if (boxes.empty()) {return;}
  struct Task {int node, first, count;};
  nodes.emplace_back();
  std::vector<Task> stack{{0, 0, static_cast<int>(boxes.size())}};
  while (!stack.empty()) {
    const Task t = stack.back();
    stack.pop_back();
    Aabb box, cbox;
    for (int i = t.first; i < t.first + t.count; ++i) {
      box.Grow(boxes[order[i]]);
      cbox.Grow(boxes[order[i]].Center());
    }
    nodes[t.node].box = box;
    if (t.count <= kLeafSize) {
      nodes[t.node].first = t.first;
      nodes[t.node].count = t.count;
      continue;
    }
    const Vector3d ext = cbox.hi - cbox.lo;
    const int axis = ext.X() > ext.Y() ? (ext.X() > ext.Z() ? 0 : 2) : (ext.Y() > ext.Z() ? 1 : 2);
    const int mid = t.first + t.count / 2;
    std::nth_element(order.begin() + t.first, order.begin() + mid,
      order.begin() + t.first + t.count, [&](int a, int b) {
        return boxes[a].Center()[axis] < boxes[b].Center()[axis];
      });
    const int l = static_cast<int>(nodes.size());
    nodes.emplace_back();
    nodes.emplace_back();
    nodes[t.node].left = l;
    nodes[t.node].count = 0;
    stack.push_back({l, t.first, mid - t.first});
    stack.push_back({l + 1, mid, t.first + t.count - mid});
  }
}

// Smallest root of a t^2 + 2 b t + c = 0 inside (tmin, tmax).
inline bool QuadRoot(double a, double b, double c, double tmin, double tmax, double & t)
{
  const double disc = b * b - a * c;
  if (disc < 0 || a == 0.0) {return false;}
  const double sq = std::sqrt(disc);
  const double r0 = (-b - sq) / a, r1 = (-b + sq) / a;
  if (r0 > tmin && r0 < tmax) {t = r0; return true;}
  if (r1 > tmin && r1 < tmax) {t = r1; return true;}
  return false;
}

inline bool Sphere(const Vector3d & o, const Vector3d & d, double r, double tmin, double & tmax)
{
  double t;
  if (QuadRoot(d.Dot(d), o.Dot(d), o.Dot(o) - r * r, tmin, tmax, t)) {tmax = t; return true;}
  return false;
}

// Cylinder along local z, |z| <= h; caps optional.
inline bool Cylinder(const Vector3d & o, const Vector3d & d, double r, double h, bool caps,
  double tmin, double & tmax)
{
  bool hit = false;
  const double a = d.X() * d.X() + d.Y() * d.Y();
  const double b = o.X() * d.X() + o.Y() * d.Y();
  const double c = o.X() * o.X() + o.Y() * o.Y() - r * r;
  const double disc = b * b - a * c;
  if (a > 0 && disc >= 0) {
    const double sq = std::sqrt(disc);
    for (double t : {(-b - sq) / a, (-b + sq) / a}) {
      if (t > tmin && t < tmax && std::abs(o.Z() + t * d.Z()) <= h) {tmax = t; hit = true; break;}
    }
  }
  if (caps && d.Z() != 0.0) {
    for (double zc : {-h, h}) {
      const double t = (zc - o.Z()) / d.Z();
      if (t > tmin && t < tmax) {
        const double x = o.X() + t * d.X(), y = o.Y() + t * d.Y();
        if (x * x + y * y <= r * r) {tmax = t; hit = true;}
      }
    }
  }
  return hit;
}
}  // namespace

// ------------------------------------------------------------------------- TriMesh
void TriMesh::Build()
{
  std::vector<Aabb> boxes(v0.size());
  bounds = Aabb();
  for (std::size_t i = 0; i < v0.size(); ++i) {
    boxes[i].Grow(v0[i]);
    boxes[i].Grow(v0[i] + e1[i]);
    boxes[i].Grow(v0[i] + e2[i]);
    bounds.Grow(boxes[i]);
  }
  BuildBvh(boxes, nodes, order);
}

bool TriMesh::Intersect(const Vector3d & o, const Vector3d & d, double tmin, double & tmax) const
{
  if (nodes.empty()) {return false;}
  const Vector3d inv = Inverse(d);
  bool hit = false;
  int stack[64];
  int sp = 0;
  stack[sp++] = 0;
  while (sp) {
    const Node & n = nodes[stack[--sp]];
    if (!HitBox(n.box, o, inv, tmin, tmax)) {continue;}
    if (n.left < 0) {
      for (int k = n.first; k < n.first + n.count; ++k) {
        const int i = order[k];
        const Vector3d p = d.Cross(e2[i]);
        const double det = e1[i].Dot(p);
        if (std::abs(det) < 1e-14) {continue;}
        const double id = 1.0 / det;
        const Vector3d tv = o - v0[i];
        const double u = tv.Dot(p) * id;
        if (u < 0 || u > 1) {continue;}
        const Vector3d q = tv.Cross(e1[i]);
        const double v = d.Dot(q) * id;
        if (v < 0 || u + v > 1) {continue;}
        const double t = e2[i].Dot(q) * id;
        if (t > tmin && t < tmax) {tmax = t; hit = true;}
      }
    } else if (sp < 62) {
      stack[sp++] = n.left;
      stack[sp++] = n.left + 1;
    }
  }
  return hit;
}

std::shared_ptr<const TriMesh> LoadTriMesh(
  const std::string & path, const Vector3d & scale, const std::string & submesh,
  std::string * error)
{
  static std::mutex mtx;
  static std::map<std::string, std::shared_ptr<const TriMesh>> cache;
  const std::string key = path + "|" + submesh + "|" + std::to_string(scale.X()) + "," +
    std::to_string(scale.Y()) + "," + std::to_string(scale.Z());
  std::lock_guard<std::mutex> lock(mtx);
  auto it = cache.find(key);
  if (it != cache.end()) {return it->second;}

  const gz::common::Mesh * mesh = gz::common::MeshManager::Instance()->Load(path);
  if (!mesh) {
    if (error) {*error = "cannot load mesh " + path;}
    return nullptr;
  }
  auto tm = std::make_shared<TriMesh>();
  for (unsigned s = 0; s < mesh->SubMeshCount(); ++s) {
    auto sm = mesh->SubMeshByIndex(s).lock();
    if (!sm || (!submesh.empty() && sm->Name() != submesh)) {continue;}
    if (sm->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) {continue;}
    for (unsigned k = 0; k + 2 < sm->IndexCount(); k += 3) {
      const Vector3d a = sm->Vertex(sm->Index(k)) * scale;
      const Vector3d b = sm->Vertex(sm->Index(k + 1)) * scale;
      const Vector3d c = sm->Vertex(sm->Index(k + 2)) * scale;
      tm->v0.push_back(a);
      tm->e1.push_back(b - a);
      tm->e2.push_back(c - a);
    }
  }
  if (tm->v0.empty()) {
    if (error) {*error = "no triangles in mesh " + path;}
    return nullptr;
  }
  tm->Build();
  cache[key] = tm;
  return tm;
}

// ------------------------------------------------------------------------- Raycaster
void Raycaster::SetShapes(std::vector<Shape> shapes)
{
  shapes_ = std::move(shapes);
  Commit();
}

bool Raycaster::SetPose(std::size_t i, const Pose3d & p)
{
  Shape & s = shapes_[i];
  if (s.world.Pos().Equal(p.Pos(), 1e-12) && s.world.Rot().Equal(p.Rot(), 1e-12)) {
    return false;
  }
  s.world = p;
  return true;
}

void Raycaster::Commit()
{
  planes_.clear();
  std::vector<int> finite;
  std::vector<Aabb> boxes;
  for (std::size_t i = 0; i < shapes_.size(); ++i) {
    Shape & s = shapes_[i];
    if (s.kind == ShapeKind::kPlane) {planes_.push_back(static_cast<int>(i)); continue;}
    Vector3d h;
    Aabb local;
    switch (s.kind) {
      case ShapeKind::kBox: h = s.size * 0.5; break;
      case ShapeKind::kSphere: h.Set(s.size.X(), s.size.X(), s.size.X()); break;
      case ShapeKind::kCylinder: h.Set(s.size.X(), s.size.X(), s.size.Y() * 0.5); break;
      case ShapeKind::kCapsule:
        h.Set(s.size.X(), s.size.X(), s.size.Y() * 0.5 + s.size.X()); break;
      case ShapeKind::kEllipsoid: h = s.size; break;
      default: break;
    }
    if (s.kind == ShapeKind::kMesh) {local = s.mesh->bounds;} else {local.Grow(-h); local.Grow(h);}
    s.world_box = Aabb();
    for (int c = 0; c < 8; ++c) {
      const Vector3d corner(c & 1 ? local.hi.X() : local.lo.X(),
        c & 2 ? local.hi.Y() : local.lo.Y(), c & 4 ? local.hi.Z() : local.lo.Z());
      s.world_box.Grow(s.world.CoordPositionAdd(corner));
    }
    finite.push_back(static_cast<int>(i));
    boxes.push_back(s.world_box);
  }
  std::vector<int> ord;
  BuildBvh(boxes, nodes_, ord);
  order_.resize(ord.size());
  for (std::size_t k = 0; k < ord.size(); ++k) {order_[k] = finite[ord[k]];}
}

bool Raycaster::IntersectShape(const Shape & s, const Vector3d & ow, const Vector3d & dw,
  double tmin, double & tmax) const
{
  // To the shape's local frame (rotation preserves t).
  const Vector3d o = s.world.Rot().RotateVectorReverse(ow - s.world.Pos());
  const Vector3d d = s.world.Rot().RotateVectorReverse(dw);
  switch (s.kind) {
    case ShapeKind::kBox: {
        const Vector3d h = s.size * 0.5;
        Aabb b;
        b.lo = -h;
        b.hi = h;
        const Vector3d inv = Inverse(d);
        double t0 = tmin, t1 = tmax;
        for (int a = 0; a < 3; ++a) {
          double ta = (b.lo[a] - o[a]) * inv[a], tb = (b.hi[a] - o[a]) * inv[a];
          if (std::isnan(ta) || std::isnan(tb)) {
            if (o[a] < b.lo[a] || o[a] > b.hi[a]) {return false;}
            continue;
          }
          if (ta > tb) {std::swap(ta, tb);}
          t0 = std::max(t0, ta);
          t1 = std::min(t1, tb);
          if (t1 < t0) {return false;}
        }
        // Entry point, or exit point if the origin is inside the box.
        const double t = t0 > tmin ? t0 : t1;
        if (t > tmin && t < tmax) {tmax = t; return true;}
        return false;
      }
    case ShapeKind::kSphere: return Sphere(o, d, s.size.X(), tmin, tmax);
    case ShapeKind::kEllipsoid: {
        const Vector3d os(o.X() / s.size.X(), o.Y() / s.size.Y(), o.Z() / s.size.Z());
        const Vector3d ds(d.X() / s.size.X(), d.Y() / s.size.Y(), d.Z() / s.size.Z());
        return Sphere(os, ds, 1.0, tmin, tmax);   // t is invariant under the scaling
      }
    case ShapeKind::kCylinder:
      return Cylinder(o, d, s.size.X(), s.size.Y() * 0.5, true, tmin, tmax);
    case ShapeKind::kCapsule: {
        bool hit = Cylinder(o, d, s.size.X(), s.size.Y() * 0.5, false, tmin, tmax);
        for (double zc : {-0.5 * s.size.Y(), 0.5 * s.size.Y()}) {
          hit |= Sphere(o - Vector3d(0, 0, zc), d, s.size.X(), tmin, tmax);
        }
        return hit;
      }
    case ShapeKind::kPlane: {
        const double den = s.size.Dot(d);
        if (den == 0.0) {return false;}
        const double t = -s.size.Dot(o) / den;
        if (t > tmin && t < tmax) {tmax = t; return true;}
        return false;
      }
    case ShapeKind::kMesh: return s.mesh->Intersect(o, d, tmin, tmax);
  }
  return false;
}

bool Raycaster::Cast(const Vector3d & o, const Vector3d & d, double tmin, double tmax,
  RayHit & hit) const
{
  bool any = false;
  for (int i : planes_) {
    if (IntersectShape(shapes_[i], o, d, tmin, tmax)) {hit.shape = i; any = true;}
  }
  if (nodes_.empty()) {
    if (any) {hit.t = tmax;}
    return any;
  }
  const Vector3d inv = Inverse(d);
  int stack[64];
  int sp = 0;
  stack[sp++] = 0;
  while (sp) {
    const Node & n = nodes_[stack[--sp]];
    if (!HitBox(n.box, o, inv, tmin, tmax)) {continue;}
    if (n.left < 0) {
      for (int k = n.first; k < n.first + n.count; ++k) {
        const int i = order_[k];
        if (IntersectShape(shapes_[i], o, d, tmin, tmax)) {hit.shape = i; any = true;}
      }
    } else if (sp < 62) {
      stack[sp++] = n.left;
      stack[sp++] = n.left + 1;
    }
  }
  if (any) {hit.t = tmax;}
  return any;
}

}  // namespace livox_mid360_sim
