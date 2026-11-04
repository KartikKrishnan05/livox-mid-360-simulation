// Livox Mid-360 gz-sim system (approach 2: one exact ray per pattern direction).
//
// Every physics step, the system casts exactly the pattern rays whose timestamps fall into
// that step (200 per 1 ms step at 200 k pts/s). Each ray starts from the sensor pose
// interpolated to the ray's own time (lerp + slerp between the previous and current step), so
// motion skew is exact. Rays are intersected with the scene's collision geometry (or visual
// geometry, <geometry_source>visual</geometry_source>) by an exact CPU ray caster: analytic
// primitives + BVH'd meshes, read from the ECM (no rendering, no physics-engine dependency).
// Output: gz.msgs.PointCloudPacked in livox_ros_driver2's 26-byte layout, one frame of
// 20 000 rays every 100 ms; livox_driver_emulator turns that into CustomMsg.
#include <algorithm>
#include <iostream>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/LaserRetro.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include "livox_output.hpp"
#include "raycaster.hpp"
#include "scan_pattern.hpp"

namespace livox_mid360_gazebo
{
using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::kNullEntity;
namespace components = gz::sim::components;

class LivoxMid360Raycast : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    std::cerr << "[livox_mid360] sensor: raycast (approach 2) - exact CPU ray per pattern direction" << std::endl;
    auto get_s = [&](const char * k, const std::string & d) {
        return sdf->HasElement(k) ? sdf->Get<std::string>(k) : d;
      };
    auto get_d = [&](const char * k, double d) {
        return sdf->HasElement(k) ? sdf->Get<double>(k) : d;
      };

    model_ = gz::sim::Model(entity);
    link_name_ = get_s("link_name", "lidar_link");
    frame_offset_ = sdf->HasElement("frame_offset") ?
      sdf->Get<gz::math::Pose3d>("frame_offset") : gz::math::Pose3d::Zero;
    frame_id_ = get_s("frame_id", "livox_frame");
    points_topic_ = get_s("points_topic", "/livox/points");
    use_visuals_ = get_s("geometry_source", "collision") == "visual";
    point_rate_ = get_d("point_rate", 200000.0);
    frame_rate_ = get_d("frame_rate", 10.0);
    points_per_frame_ = static_cast<std::size_t>(std::lround(point_rate_ / frame_rate_));
    dt_ns_ = 1e9 / point_rate_;

    ReturnModelParams rp;
    rp.min_range = get_d("min_range", rp.min_range);
    rp.max_range = get_d("max_range", rp.max_range);
    rp.nominal_range = get_d("nominal_range", rp.nominal_range);
    rp.range_noise_stddev = get_d("range_noise_stddev", rp.range_noise_stddev);
    rp.default_reflectivity = get_d("default_reflectivity", rp.default_reflectivity);
    rp.seed = static_cast<unsigned>(get_d("seed", 0));
    params_ = rp;
    returns_ = ReturnModel(rp);

    const std::string uri = get_s("pattern_file", "model://livox_mid360/scan_mode/mid360.csv");
    const std::string path = ResolveResourceUri(uri);
    std::string err;
    if (!path.empty() && pattern_.LoadCsv(path, &err)) {
      gzmsg << "[livox_mid360] loaded scan pattern " << path << " (" << pattern_.size()
            << " points)\n";
    } else {
      gzwarn << "[livox_mid360] scan pattern '" << uri << "' not found"
             << (err.empty() ? "" : " (" + err + ")")
             << "; falling back to the procedural 4-laser pattern\n";
      pattern_.MakeProcedural(static_cast<std::size_t>(4 * point_rate_));
    }

    link_ = model_.LinkByName(ecm, link_name_);
    if (link_ == kNullEntity) {
      gzerr << "[livox_mid360] link '" << link_name_ << "' not found in model '"
            << model_.Name(ecm) << "'; the sensor is disabled\n";
    }
    pub_ = node_.Advertise<gz::msgs::PointCloudPacked>(points_topic_);
    points_.reserve(points_per_frame_);
    gzmsg << "[livox_mid360] exact ray casting on " << (use_visuals_ ? "visual" : "collision")
          << " geometry, link '" << link_name_ << "' -> " << points_topic_ << "\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo & info, const EntityComponentManager & ecm) override
  {
    if (link_ == kNullEntity || info.paused) {return;}
    const int64_t t_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      info.simTime).count();

    if (started_ && t_now < t_prev_) {     // world reset
      started_ = false;
      points_.clear();
    }
    const auto c0 = std::chrono::steady_clock::now();
    UpdateScene(ecm);
    const gz::math::Pose3d pose_now = gz::sim::worldPose(link_, ecm) * frame_offset_;

    if (!started_) {
      started_ = true;
      t0_ = t_now;
      next_ray_ = 0;
      t_prev_ = t_now;
      pose_prev_ = pose_now;
    }

    const double span = static_cast<double>(t_now - t_prev_);
    for (;;) {
      const int64_t t_ray = t0_ + static_cast<int64_t>(std::llround(next_ray_ * dt_ns_));
      if (t_ray > t_now) {break;}
      // Sensor pose at the ray's own time.
      gz::math::Pose3d pose = pose_now;
      if (span > 0) {
        const double a = std::clamp((t_ray - t_prev_) / span, 0.0, 1.0);
        pose.Pos() = pose_prev_.Pos() + (pose_now.Pos() - pose_prev_.Pos()) * a;
        pose.Rot() = gz::math::Quaterniond::Slerp(a, pose_prev_.Rot(), pose_now.Rot(), true);
      }
      CastRay(next_ray_, t_ray, pose);
      ++next_ray_;
      if (next_ray_ % points_per_frame_ == 0) {
        const int64_t timebase = t0_ + static_cast<int64_t>(std::llround(
            (next_ray_ - points_per_frame_) * dt_ns_));
        FillPointCloudPacked(out_, points_, frame_id_, timebase);
        pub_.Publish(out_);
        points_.clear();
      }
    }
    t_prev_ = t_now;
    pose_prev_ = pose_now;

    cost_ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - c0).count();
    if (++steps_ % 10000 == 0) {
      gzmsg << "[livox_mid360] " << rays_ << " rays, " << 1e-3 * cost_ns_ / rays_
            << " us/ray incl. scene update (" << 1e-3 * cost_ns_ / steps_ << " us/step)\n";
    }
  }

private:
  void CastRay(uint64_t g, int64_t t_ray, const gz::math::Pose3d & pose)
  {
    ++rays_;
    const PatternRay & ray = pattern_[g % pattern_.size()];
    const Vector3d d = pose.Rot().RotateVector(Vector3d(ray.dx, ray.dy, ray.dz));
    RayHit hit;
    // Rays start at min_range, like gz lidars: the housing is never seen from inside.
    if (!caster_.Cast(pose.Pos(), d, params_.min_range, params_.max_range, hit)) {return;}
    float range;
    uint8_t refl;
    if (!returns_.Apply(hit.t, caster_.GetShape(hit.shape).retro, range, refl)) {return;}
    LivoxPoint p;
    p.x = ray.dx * range;
    p.y = ray.dy * range;
    p.z = ray.dz * range;
    p.intensity = refl;
    p.tag = 0;
    p.line = ray.line;
    p.timestamp = static_cast<double>(t_ray);
    points_.push_back(p);
  }

  // ---------------------------------------------------------------- scene from the ECM
  static bool IsStatic(Entity e, const EntityComponentManager & ecm)
  {
    while (e != kNullEntity) {
      if (ecm.Component<components::Model>(e)) {
        auto * st = ecm.Component<components::Static>(e);
        if (st && st->Data()) {return true;}
      }
      auto * parent = ecm.Component<components::ParentEntity>(e);
      e = parent ? parent->Data() : kNullEntity;
    }
    return false;
  }

  void UpdateScene(const EntityComponentManager & ecm)
  {
    bool changed = !scene_built_;
    if (use_visuals_) {
      ecm.EachNew<components::Visual, components::Geometry>(
        [&](const Entity &, const components::Visual *, const components::Geometry *) {
          changed = true; return true;
        });
      ecm.EachRemoved<components::Visual>(
        [&](const Entity &, const components::Visual *) {changed = true; return true;});
    } else {
      ecm.EachNew<components::Collision, components::Geometry>(
        [&](const Entity &, const components::Collision *, const components::Geometry *) {
          changed = true; return true;
        });
      ecm.EachRemoved<components::Collision>(
        [&](const Entity &, const components::Collision *) {changed = true; return true;});
    }
    if (changed) {RebuildScene(ecm);}

    bool moved = changed;
    for (std::size_t i = 0; i < entities_.size(); ++i) {
      if (static_[i]) {continue;}
      moved |= caster_.SetPose(i, gz::sim::worldPose(entities_[i], ecm));
    }
    if (moved) {caster_.Commit();}
  }

  void RebuildScene(const EntityComponentManager & ecm)
  {
    std::vector<Shape> shapes;
    entities_.clear();
    static_.clear();
    int skipped = 0;
    auto add = [&](const Entity & e, const sdf::Geometry & g) {
        Shape s;
        switch (g.Type()) {
          case sdf::GeometryType::BOX:
            s.kind = ShapeKind::kBox; s.size = g.BoxShape()->Size(); break;
          case sdf::GeometryType::SPHERE:
            s.kind = ShapeKind::kSphere; s.size.X(g.SphereShape()->Radius()); break;
          case sdf::GeometryType::CYLINDER:
            s.kind = ShapeKind::kCylinder;
            s.size.Set(g.CylinderShape()->Radius(), g.CylinderShape()->Length(), 0);
            break;
          case sdf::GeometryType::CAPSULE:
            s.kind = ShapeKind::kCapsule;
            s.size.Set(g.CapsuleShape()->Radius(), g.CapsuleShape()->Length(), 0);
            break;
          case sdf::GeometryType::ELLIPSOID:
            s.kind = ShapeKind::kEllipsoid; s.size = g.EllipsoidShape()->Radii(); break;
          case sdf::GeometryType::PLANE:
            s.kind = ShapeKind::kPlane; s.size = g.PlaneShape()->Normal().Normalized(); break;
          case sdf::GeometryType::MESH: {
              const sdf::Mesh * m = g.MeshShape();
              std::string path = gz::sim::asFullPath(m->Uri(), m->FilePath());
              if (!std::filesystem::exists(path)) {path = gz::common::findFile(m->Uri());}
              std::string err;
              s.mesh = path.empty() ? nullptr :
                LoadTriMesh(path, m->Scale(), m->Submesh(), &err);
              if (!s.mesh) {
                gzwarn << "[livox_mid360] skipping mesh '" << m->Uri() << "' " << err << "\n";
                ++skipped;
                return true;
              }
              s.kind = ShapeKind::kMesh;
              break;
            }
          default:
            ++skipped;       // heightmap, polyline, cone, ...: not supported
            return true;
        }
        if (auto * r = ecm.Component<components::LaserRetro>(e)) {s.retro = r->Data();}
        if (auto * n = ecm.Component<components::Name>(e)) {s.name = n->Data();}
        s.world = gz::sim::worldPose(e, ecm);
        shapes.push_back(std::move(s));
        entities_.push_back(e);
        static_.push_back(IsStatic(e, ecm));
        return true;
      };
    if (use_visuals_) {
      ecm.Each<components::Visual, components::Geometry>(
        [&](const Entity & e, const components::Visual *, const components::Geometry * g) {
          return add(e, g->Data());
        });
    } else {
      ecm.Each<components::Collision, components::Geometry>(
        [&](const Entity & e, const components::Collision *, const components::Geometry * g) {
          return add(e, g->Data());
        });
    }
    caster_.SetShapes(std::move(shapes));
    scene_built_ = true;
    gzmsg << "[livox_mid360] scene: " << caster_.NumShapes() << " shapes"
          << (skipped ? ", " + std::to_string(skipped) + " unsupported skipped" : "") << "\n";
  }

  // config
  gz::sim::Model model_{kNullEntity};
  Entity link_{kNullEntity};
  std::string link_name_, frame_id_, points_topic_;
  gz::math::Pose3d frame_offset_;
  bool use_visuals_{false};
  double point_rate_{200000}, frame_rate_{10}, dt_ns_{5000};
  std::size_t points_per_frame_{20000};
  ScanPattern pattern_;
  ReturnModelParams params_;
  ReturnModel returns_;
  // scene
  Raycaster caster_;
  std::vector<Entity> entities_;
  std::vector<bool> static_;
  bool scene_built_{false};
  // timing
  bool started_{false};
  int64_t t0_{0}, t_prev_{0};
  uint64_t next_ray_{0};
  gz::math::Pose3d pose_prev_;
  int64_t cost_ns_{0}, rays_{0}, steps_{0};
  // output
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
  std::vector<LivoxPoint> points_;
  gz::msgs::PointCloudPacked out_;
};

}  // namespace livox_mid360_gazebo

GZ_ADD_PLUGIN(
  livox_mid360_gazebo::LivoxMid360Raycast, gz::sim::System,
  livox_mid360_gazebo::LivoxMid360Raycast::ISystemConfigure,
  livox_mid360_gazebo::LivoxMid360Raycast::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(livox_mid360_gazebo::LivoxMid360Raycast, "livox_mid360_gazebo::LivoxMid360Raycast")
// Backwards-compatible name: the recommended (ray casting) sensor.
GZ_ADD_PLUGIN_ALIAS(livox_mid360_gazebo::LivoxMid360Raycast, "livox_mid360_gazebo::LivoxMid360")
