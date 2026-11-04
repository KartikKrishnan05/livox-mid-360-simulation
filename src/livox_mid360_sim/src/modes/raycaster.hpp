// Copyright 2026 Kartik Krishnan
// SPDX-License-Identifier: Apache-2.0

#ifndef LIVOX_MID360_SIM__RAYCASTER_HPP_
#define LIVOX_MID360_SIM__RAYCASTER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <sdf/Geometry.hh>

namespace livox_mid360_sim::modes
{

struct Vec3f
{
  float x, y, z;
};

/// Bounding volume hierarchy over a triangle soup, for exact ray casting.
///
/// Built once per distinct shape (binned SAH), then queried with rays in the
/// shape's local frame. Triangles are two-sided.
class TriangleBvh
{
public:
  /// \param[in] triangles 3 vertices per triangle.
  explicit TriangleBvh(const std::vector<Vec3f> & triangles);

  /// Closest hit with distance in [tmin, tmax]. On a hit, tmax is lowered to
  /// the hit distance. `dir` must be a unit vector for t to be in metres.
  bool Intersect(const Vec3f & origin, const Vec3f & dir, float tmin, float & tmax) const;

  std::size_t TriangleCount() const {return tris_.size();}

private:
  struct Node
  {
    float bmin[3];
    float bmax[3];
    uint32_t first;  ///< leaf: first triangle; inner: index of the left child
    uint32_t count;  ///< leaf: number of triangles; inner: 0
  };
  struct Tri
  {
    Vec3f v0, e1, e2;
  };
  struct BuildTri
  {
    float bmin[3], bmax[3], c[3];
  };

  void Subdivide(uint32_t node, uint32_t first, uint32_t count, int depth);

  std::vector<Node> nodes_;
  std::vector<Tri> tris_;
  std::vector<BuildTri> build_;     ///< only used while building
  std::vector<uint32_t> order_;     ///< only used while building
};

/// The visual geometry of a Gazebo world, ray cast on the CPU.
///
/// Every visual with a supported geometry (box, sphere, cylinder, capsule,
/// ellipsoid, cone, plane, mesh) becomes an instance of a shared BVH. Poses
/// of static models are read once; all others every Update().
class Scene
{
public:
  /// Pick up new / removed visuals and refresh the poses of moving ones.
  void Update(const gz::sim::EntityComponentManager & ecm);

  /// Closest hit of a world-frame ray with distance in [tmin, tmax].
  /// \param[out] range distance to the hit [m]
  /// \param[out] retro <laser_retro> of the hit visual, 0 if none
  bool Intersect(
    const gz::math::Vector3d & origin, const gz::math::Vector3d & dir,
    double tmin, double tmax, double & range, double & retro) const;

  std::size_t InstanceCount() const {return instances_.size();}

private:
  struct Instance
  {
    gz::sim::Entity visual;
    std::shared_ptr<const TriangleBvh> bvh;
    bool is_static = false;
    double retro = 0.0;
    gz::math::Pose3d world;           ///< visual frame in the world
    gz::math::Quaterniond inv_rot;    ///< world -> visual rotation
  };

  void Rebuild(const gz::sim::EntityComponentManager & ecm);
  std::shared_ptr<const TriangleBvh> ShapeFor(const sdf::Geometry & geometry);

  bool built_ = false;
  std::vector<Instance> instances_;
  std::unordered_map<std::string, std::shared_ptr<const TriangleBvh>> cache_;
};

}  // namespace livox_mid360_sim::modes

#endif  // LIVOX_MID360_SIM__RAYCASTER_HPP_
