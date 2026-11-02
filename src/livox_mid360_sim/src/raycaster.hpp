// Exact CPU ray caster over gz-sim geometry (collision or visual shapes from the ECM).
//
//  * analytic primitives: box, sphere, cylinder, capsule, ellipsoid, plane (infinite);
//  * meshes: triangle BVH per mesh (cached by uri + scale), Moller-Trumbore;
//  * a top-level BVH over the world AABBs of all finite shapes, rebuilt when a pose changes.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace livox_mid360_sim
{

using gz::math::Pose3d;
using gz::math::Vector3d;

struct Aabb
{
  Vector3d lo{1e300, 1e300, 1e300};
  Vector3d hi{-1e300, -1e300, -1e300};
  void Grow(const Vector3d & p)
  {
    lo.Set(std::min(lo.X(), p.X()), std::min(lo.Y(), p.Y()), std::min(lo.Z(), p.Z()));
    hi.Set(std::max(hi.X(), p.X()), std::max(hi.Y(), p.Y()), std::max(hi.Z(), p.Z()));
  }
  void Grow(const Aabb & b) {Grow(b.lo); Grow(b.hi);}
  Vector3d Center() const {return (lo + hi) * 0.5;}
};

/// Triangle mesh with its own BVH, in the shape's local (scaled) frame.
struct TriMesh
{
  std::vector<Vector3d> v0, e1, e2;     // per triangle
  struct Node {Aabb box; int left{-1}; int first{0}; int count{0};};
  std::vector<Node> nodes;
  std::vector<int> order;
  Aabb bounds;
  void Build();
  bool Intersect(const Vector3d & o, const Vector3d & d, double tmin, double & tmax) const;
};

enum class ShapeKind {kBox, kSphere, kCylinder, kCapsule, kEllipsoid, kPlane, kMesh};

struct Shape
{
  ShapeKind kind;
  Vector3d size;          // box: full size; sphere: (r); cylinder/capsule: (r, len);
                          // ellipsoid: radii; plane: normal (local)
  std::shared_ptr<const TriMesh> mesh;
  double retro{0.0};      // laser_retro (0 = unknown)
  Pose3d world;           // current world pose of the shape
  Aabb world_box;
  std::string name;
};

struct RayHit
{
  double t{0.0};
  int shape{-1};
};

class Raycaster
{
public:
  /// Replace the shape set (poses filled in later with SetPose + Commit).
  void SetShapes(std::vector<Shape> shapes);
  std::size_t NumShapes() const {return shapes_.size();}
  const Shape & GetShape(std::size_t i) const {return shapes_[i];}
  /// Update a shape's world pose; returns true if it changed.
  bool SetPose(std::size_t i, const Pose3d & p);
  /// Recompute world AABBs and the top-level BVH (call after poses changed).
  void Commit();

  /// Closest hit along o + t d (d unit) with tmin < t < tmax.
  bool Cast(const Vector3d & o, const Vector3d & d, double tmin, double tmax, RayHit & hit) const;

private:
  bool IntersectShape(const Shape & s, const Vector3d & o, const Vector3d & d, double tmin,
    double & tmax) const;
  std::vector<Shape> shapes_;
  std::vector<int> planes_;                    // unbounded shapes, tested always
  struct Node {Aabb box; int left{-1}; int first{0}; int count{0};};
  std::vector<Node> nodes_;
  std::vector<int> order_;                     // finite shape indices in BVH order
};

/// Mesh cache: load a mesh file with gz-common (any format it supports), scaled.
std::shared_ptr<const TriMesh> LoadTriMesh(
  const std::string & path, const Vector3d & scale, const std::string & submesh,
  std::string * error);

}  // namespace livox_mid360_sim
