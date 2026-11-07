#include "robot_registry.hpp"
#include <bit>
#include <memory>
#include <openssl/evp.h>
namespace sim {
namespace {
const RobotDefinition *const definitions[] = {&hydrus, &proteus, &bluerov, &girona500};
struct AssetHash { std::string_view name, sha256; };
#include "robots/asset_hashes.inc"
// Canonical scalar encoding hashes semantic values, never C++ padding, pointer
// addresses or JSON spelling. Asset metadata and the hash itself are excluded.
class Digest {
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx{
      EVP_MD_CTX_new(), EVP_MD_CTX_free};
public:
  Digest() {
    require(ctx && EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) == 1,
            "cannot create robot configuration digest");
  }
  void bytes(const void *data, size_t n) {
    require(EVP_DigestUpdate(ctx.get(), data, n) == 1, "robot digest failed");
  }
  void integer(uint32_t value) {
    unsigned char data[4];
    for (size_t i = 0; i < 4; ++i) data[i] = uint8_t(value >> (8 * i));
    bytes(data, sizeof(data));
  }
  void scalar(double value) {
    uint64_t bits = std::bit_cast<uint64_t>(value);
    unsigned char data[8];
    for (size_t i = 0; i < 8; ++i) data[i] = uint8_t(bits >> (8 * i));
    bytes(data, sizeof(data));
  }
  void text(std::string_view value) { integer(value.size()); bytes(value.data(), value.size()); }
  void vec(SimVec3 v) { scalar(v.x); scalar(v.y); scalar(v.z); }
  void quat(SimQuat q) { scalar(q.x); scalar(q.y); scalar(q.z); scalar(q.w); }
  void pose(SimPose p) { vec(p.position); quat(p.orientation); }
  void points(std::span<const SimVec3> data) {
    integer(data.size());
    for (auto value : data) vec(value);
  }
  std::string finish() {
    unsigned char digest[EVP_MAX_MD_SIZE]; unsigned length = 0;
    require(EVP_DigestFinal_ex(ctx.get(), digest, &length) == 1 && length == 32,
            "robot digest failed");
    constexpr char hex[] = "0123456789abcdef";
    std::string result; result.reserve(64);
    for (unsigned i = 0; i < length; ++i) {
      result += hex[digest[i] >> 4]; result += hex[digest[i] & 15];
    }
    return result;
  }
};
} // namespace
std::span<const RobotDefinition *const> robot_definitions() { return definitions; }
const RobotDefinition *robot_definition(std::string_view name) {
  for (const auto *robot : definitions)
    if (robot->name == name) return robot;
  return nullptr;
}
std::string_view robot_asset_hash(std::string_view name) {
  for (const auto &asset : asset_hashes)
    if (asset.name == name) return asset.sha256;
  return {};
}
std::string robot_configuration_hash(const RobotDefinition &robot) {
  require(robot.shape_count <= robot.shapes.size() && robot.hull_count <= robot.hulls.size() &&
              robot.thruster_count <= robot.thrusters.size(), "invalid fixed robot counts");
  Digest d;
  d.text("fixed-robot-v2"); d.text(robot.name);
  const auto &b = robot.body;
  d.scalar(b.mass); d.scalar(b.dry_mass); d.vec(b.inertia); d.pose(b.center_of_mass);
  d.scalar(b.volume); d.vec(b.buoyancy_center);
  d.vec(b.initial_linear_velocity); d.vec(b.initial_angular_velocity);
  d.integer(robot.shape_count);
  for (const auto &s : std::span(robot.shapes).first(robot.shape_count)) {
    d.text(s.id); d.integer(s.kind); d.integer(s.collisions); d.points(s.points);
  }
  d.integer(robot.hull_count);
  for (const auto &h : std::span(robot.hulls).first(robot.hull_count)) {
    const auto &p = h.physics;
    d.pose(p.pose); d.vec(p.drag); d.vec(p.skin); d.vec(p.centroid);
    d.quat(p.coefficient_frame); d.scalar(p.volume); d.integer(h.collider); d.points(h.positions);
  }
  d.integer(robot.thruster_count);
  for (const auto &t : std::span(robot.thrusters).first(robot.thruster_count)) {
    d.vec(t.position); d.vec(t.direction);
    const auto &s = t.spec;
    for (double value : {s.max_rpm, s.diameter, s.thrust, s.torque, s.inertia,
                         s.kp, s.ki, s.command_sign}) d.scalar(value);
  }
  const auto &c = robot.camera;
  d.integer(c.width); d.integer(c.height); d.scalar(c.horizontal_fov); d.pose(c.pose);
  return d.finish();
}
} // namespace sim
