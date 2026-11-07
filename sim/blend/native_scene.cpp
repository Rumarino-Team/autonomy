// Read the saved Blender datablocks directly. No Blender process, Python scene
// exporter or intermediate scene document participates in world creation.
#include "image.hpp"
#include "robot_registry.hpp"
#include <cblend.hpp>
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <libqhull_r/qhull_ra.h>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <optional>
#include <set>
#include <sstream>
#include <zlib.h>
#include <zstd.h>
namespace sim {
namespace {
using Json = nlohmann::json;
using Bytes = cblend::MemorySpan;
using Matrix = glm::mat4;
std::string sha256(const void *data, size_t count);
[[noreturn]] void Fail(const std::string &text) {
  throw std::invalid_argument("native .blend: " + text);
}
template <class T> T Scalar(Bytes bytes) {
  if (bytes.size() < sizeof(T))
    Fail("truncated datablock");
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}
std::string String(Bytes bytes) {
  auto end = std::find(bytes.begin(), bytes.end(), 0);
  return {reinterpret_cast<const char *>(bytes.data()),
          size_t(end - bytes.begin())};
}
struct Node {
  cblend::BlendType type;
  Bytes bytes;
  Node Field(std::string_view name) const {
    auto field = type.GetField(name);
    if (!field)
      Fail("missing DNA field: " + std::string(name));
    auto data = field->GetData(bytes);
    if (data.empty())
      Fail("invalid DNA field: " + std::string(name));
    return {field->GetFieldType(), data};
  }
  template <class T> T Value(std::string_view name) const {
    return Scalar<T>(Field(name).bytes);
  }
  std::string Text(std::string_view name) const {
    return String(Field(name).bytes);
  }
  std::string Name() const { return Field("id").Text("name").substr(2); }
};
std::vector<uint8_t> ReadBytes(const std::filesystem::path &path,
                              std::string *file_hash = nullptr) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);

  if (!file || file.tellg() <= 0)
    Fail("cannot read " + path.string());
  std::vector<uint8_t> raw(static_cast<size_t>(file.tellg()));
  file.seekg(0);
  if (!file.read(reinterpret_cast<char *>(raw.data()), raw.size()))
    Fail("cannot read complete blend file");
  if (file_hash)
    *file_hash = sha256(raw.data(), raw.size());
  if (raw.size() >= 2 && raw[0] == 0x1f && raw[1] == 0x8b) {
    gzFile gzip = gzopen(path.c_str(), "rb");
    if (!gzip)
      Fail("cannot open gzip blend");
    std::vector<uint8_t> out;
    std::array<uint8_t, 65536> buffer;
    int n;
    while ((n = gzread(gzip, buffer.data(), buffer.size())) > 0)
      out.insert(out.end(), buffer.begin(), buffer.begin() + n);
    int status = gzclose(gzip);
    if (n < 0 || status != Z_OK)
      Fail("invalid gzip blend");
    return out;
  }
  if (raw.size() >= 4 && Scalar<uint32_t>(raw) == ZSTD_MAGICNUMBER) {
    auto *stream = ZSTD_createDStream();
    if (!stream)
      Fail("cannot allocate zstd decoder");
    std::unique_ptr<ZSTD_DStream, decltype(&ZSTD_freeDStream)> guard(
        stream, ZSTD_freeDStream);
    ZSTD_initDStream(stream);
    ZSTD_inBuffer input{raw.data(), raw.size(), 0};
    std::vector<uint8_t> out;
    std::array<uint8_t, 65536> buffer;
    size_t remaining = 1;
    while (input.pos < input.size || remaining) {
      ZSTD_outBuffer output{buffer.data(), buffer.size(), 0};
      size_t before = input.pos;
      remaining = ZSTD_decompressStream(stream, &output, &input);
      if (ZSTD_isError(remaining))
        Fail("invalid zstd blend");
      out.insert(out.end(), buffer.begin(), buffer.begin() + output.pos);
      if (input.pos == before && !output.pos)
        Fail("truncated zstd blend");
    }
    return out;
  }
  return raw;
}

std::string sha256(const void *data, size_t count) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(static_cast<const unsigned char *>(data), count, digest);
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  for (auto byte : digest) {
    result += hex[byte >> 4];
    result += hex[byte & 15];
  }
  return result;
}
uint32_t body_id(const std::string &id) {
  if (id.empty())
    Fail("missing sim_id");
  uint64_t number = 0;
  if (std::all_of(id.begin(), id.end(),
                  [](unsigned char c) { return c >= '0' && c <= '9'; })) {
    auto [end, error] =
        std::from_chars(id.data(), id.data() + id.size(), number);
    if (error != std::errc() || end != id.data() + id.size() ||
        number >= UINT32_MAX)
      Fail("invalid or reserved body ID: " + id);
  } else {
    auto digest = sha256(id.data(), id.size());
    std::from_chars(digest.data(), digest.data() + 8, number, 16);
    if (number == UINT32_MAX)
      Fail("reserved body ID: " + id);
  }
  return uint32_t(number);
}
uint32_t count_property(const Json &properties, const char *key,
                        uint32_t fallback) {
  if (!properties.contains(key))
    return fallback;
  const auto &value = properties.at(key);
  if (!value.is_number_integer() || value < 0 || value > UINT32_MAX)
    Fail(std::string(key) + " must be a nonnegative uint32 integer");
  return value.get<uint32_t>();
}
Matrix rigid(const Matrix &m) {
  glm::mat3 r(m);
  for (int i = 0; i < 3; ++i) {
    auto length = glm::length(r[i]);
    if (!(length > 1e-12f))
      Fail("singular object transform");
    r[i] /= length;
  }
  if (glm::determinant(r) < 0)
    r = -r;
  Matrix result(glm::mat4_cast(glm::normalize(glm::quat_cast(r))));
  result[3] = m[3];
  return result;
}
SimPose pose(const Matrix &m) {
  auto r = rigid(m);
  auto q = glm::normalize(glm::quat_cast(glm::mat3(r)));
  return {{-r[3].x, r[3].y, -r[3].z}, {-q.x, q.y, -q.z, q.w}};
}
SimVec3 ned(glm::vec3 v) { return {-v.x, v.y, -v.z}; }
class Reader {
public:
  cblend::Blend &blend;
  std::map<uint64_t, const cblend::Block *> blocks;
  std::map<uint64_t, Matrix> transforms;
  std::set<uint64_t> transforming;
  explicit Reader(cblend::Blend &file) : blend(file) {
    // Saved pointers are identifiers into file datablocks, never live pointers.
    // Resolve every access with bounds checks while the parsed file is alive.
    for (auto code :
         {cblend::BLOCK_CODE_DATA, cblend::BLOCK_CODE_OB, cblend::BLOCK_CODE_ME,
          cblend::BLOCK_CODE_SC, cblend::BLOCK_CODE_GR, cblend::BLOCK_CODE_TX,
          cblend::BLOCK_CODE_CA})
      for (auto &block : blend.GetBlocks(code))
        if (block.header.address)
          blocks.emplace(block.header.address, &block);
  }
  Bytes Resolve(uint64_t address) const {
    auto it = blocks.upper_bound(address);
    if (!address || it == blocks.begin())
      Fail("unresolved Blender pointer");
    --it;
    auto offset = address - it->first;
    if (offset >= it->second->body.size())
      Fail("pointer outside Blender datablock");
    return Bytes(it->second->body).subspan(offset);
  }
  Node At(uint64_t address, std::string_view name) const {
    auto type = blend.GetType(name);
    if (!type)
      Fail("missing DNA type: " + std::string(name));
    auto bytes = Resolve(address);
    if (bytes.size() < type->GetSize())
      Fail("truncated " + std::string(name));
    return {*type, bytes.first(type->GetSize())};
  }
  std::vector<Node> List(Node list, const char *name) const {
    std::vector<Node> result;
    std::set<uint64_t> visited;
    auto address = list.Value<uint64_t>("first");
    while (address) {
      if (!visited.insert(address).second || visited.size() > 1000000)
        Fail("invalid linked list");
      auto node = At(address, name);
      result.push_back(node);
      address = node.Value<uint64_t>("next");
    }
    return result;
  }
  Json Property(Node p, int depth = 0) const {
    if (depth > 32)
      Fail("property nesting exceeds limit");
    auto data = p.Field("data");
    switch (p.Value<uint8_t>("type")) {
    case 0:
      return String(Resolve(data.Value<uint64_t>("pointer")));
    case 1:
      return data.Value<int32_t>("val");
    case 2:
      return data.Value<float>("val");
    case 8: {
      auto value = data.Field("val");
      return Scalar<double>(
          data.bytes.subspan(value.bytes.data() - data.bytes.data()));
    }
    case 10:
      return data.Value<int32_t>("val") != 0;
    case 7:
      return Json{{"$id", data.Value<uint64_t>("pointer")}};
    case 6: {
      Json result = Json::object();
      for (auto child : List(data.Field("group"), "IDProperty"))
        result[child.Text("name")] = Property(child, depth + 1);
      return result;
    }
    case 5: {
      int count = p.Value<int32_t>("len");
      if (count < 0 || count > 1000000)
        Fail("invalid property array length");
      auto bytes = Resolve(data.Value<uint64_t>("pointer"));
      auto subtype = p.Value<uint8_t>("subtype");
      Json result = Json::array();
      for (int i = 0; i < count; ++i) {
        if (subtype == 8)
          result.push_back(Scalar<double>(bytes.subspan(size_t(i) * 8)));
        else if (subtype == 2)
          result.push_back(Scalar<float>(bytes.subspan(size_t(i) * 4)));
        else if (subtype == 1)
          result.push_back(Scalar<int32_t>(bytes.subspan(size_t(i) * 4)));
        else
          Fail("unsupported property array subtype");
      }
      return result;
    }
    // Blender UI collections (for example the add-on's take list) are not
    // simulation properties. Leave their contents out of the native model.
    default:
      return nullptr;
    }
  }
  Json Properties(Node n) const {
    auto address = n.Field("id").Value<uint64_t>("properties");
    return address ? Property(At(address, "IDProperty")) : Json::object();
  }
  void CheckAnimation(Node n) const {
    if (n.Field("id").Value<uint64_t>("lib"))
      Fail(n.Name() + ": linked datablocks must be made local before saving");
    if (n.Field("constraints").Value<uint64_t>("first"))
      Fail(n.Name() + ": apply constraints before saving for native loading");
    if (auto address = n.Value<uint64_t>("adt")) {
      auto adt = At(address, "AnimData");
      if (adt.Value<uint64_t>("action") ||
          adt.Field("drivers").Value<uint64_t>("first") ||
          adt.Field("nla_tracks").Value<uint64_t>("first"))
        Fail(n.Name() +
             ": bake and remove animation/drivers before native loading");
    }
  }
  glm::vec3 Vector(Node n, const char *field) const {
    auto a = n.Value<std::array<float, 3>>(field);
    return {a[0], a[1], a[2]};
  }
  glm::quat Rotation(Node n, bool delta = false) const {
    int mode = n.Value<int16_t>("rotmode");
    if (mode == 0) {
      auto q = n.Value<std::array<float, 4>>(delta ? "dquat" : "quat");
      return glm::normalize(glm::quat(q[0], q[1], q[2], q[3]));
    }
    if (mode == -1) {
      auto axis = Vector(n, delta ? "drotAxis" : "rotAxis");
      float angle = n.Value<float>(delta ? "drotAngle" : "rotAngle");
      return angle == 0 ? glm::quat(1, 0, 0, 0)
                        : glm::angleAxis(angle, glm::normalize(axis));
    }
    if (mode < 1 || mode > 6)
      Fail(n.Name() + ": unsupported rotation mode");
    const char *order[] = {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"};
    auto v = Vector(n, delta ? "drot" : "rot");
    glm::quat q(1, 0, 0, 0);
    for (int i = 0; i < 3; ++i) {
      int axis = order[mode - 1][i] - 'X';
      glm::vec3 direction(0);
      direction[axis] = 1;
      q = glm::angleAxis(v[axis], direction) * q;
    }
    return glm::normalize(q);
  }
  Matrix World(uint64_t address) {
    if (auto it = transforms.find(address); it != transforms.end())
      return it->second;
    if (!transforming.insert(address).second || transforming.size() > 1024)
      Fail("cyclic/excessive object parenting");
    auto n = At(address, "Object");
    CheckAnimation(n);
    auto matrix =
        glm::translate(Matrix(1), Vector(n, "loc") + Vector(n, "dloc")) *
        glm::mat4_cast(Rotation(n, true) * Rotation(n)) *
        glm::scale(Matrix(1), Vector(n, "size") * Vector(n, "dscale"));
    if (auto parent = n.Value<uint64_t>("parent")) {
      if ((n.Value<int16_t>("partype") & 15) != 0)
        Fail(n.Name() + ": only object parenting is supported");
      auto values = n.Value<std::array<float, 16>>("parentinv");
      Matrix inverse(1);
      std::memcpy(&inverse[0][0], values.data(), sizeof(inverse));
      matrix = World(parent) * inverse * matrix;
    }
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        if (!std::isfinite(matrix[i][j]))
          Fail(n.Name() + ": non-finite transform");
    if (std::abs(glm::determinant(matrix)) < 1e-12f)
      Fail(n.Name() + ": singular transform");
    transforming.erase(address);
    transforms.emplace(address, matrix);
    return matrix;
  }
  std::vector<uint64_t> Objects(uint64_t collection,
                                std::set<uint64_t> &visiting) const {
    if (!visiting.insert(collection).second)
      Fail("cyclic collection nesting");
    auto c = At(collection, "Collection");
    std::vector<uint64_t> out;
    for (auto member : List(c.Field("gobject"), "CollectionObject"))
      out.push_back(member.Value<uint64_t>("ob"));
    for (auto child : List(c.Field("children"), "CollectionChild")) {
      auto nested = Objects(child.Value<uint64_t>("collection"), visiting);
      out.insert(out.end(), nested.begin(), nested.end());
    }
    visiting.erase(collection);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  }
  uint64_t FindCollection(uint64_t address, const std::string &name,
                          std::set<uint64_t> &seen) const {
    if (!seen.insert(address).second)
      Fail("cyclic collection nesting");
    auto c = At(address, "Collection");
    if (c.Name() == name)
      return address;
    for (auto child : List(c.Field("children"), "CollectionChild"))
      if (auto result =
              FindCollection(child.Value<uint64_t>("collection"), name, seen))
        return result;
    seen.erase(address);
    return 0;
  }
  Bytes Layer(Node mesh, const char *domain, const char *name,
              int fallback = -1) const {
    auto cd = mesh.Field(domain);
    int count = cd.Value<int>("totlayer");
    if (count < 0 || count > 4096)
      Fail("invalid mesh attribute count");
    auto type = blend.GetType("CustomDataLayer");
    if (!type)
      Fail("missing CustomDataLayer DNA");
    auto bytes = count ? Resolve(cd.Value<uint64_t>("layers")) : Bytes{};
    if (size_t(count) > bytes.size() / type->GetSize())
      Fail("truncated mesh attributes");
    Bytes candidate;
    for (int i = 0; i < count; ++i) {
      Node layer{*type,
                 bytes.subspan(size_t(i) * type->GetSize(), type->GetSize())};
      if (layer.Text("name") == name)
        return Resolve(layer.Value<uint64_t>("data"));
      if (fallback >= 0 && layer.Value<int>("type") == fallback)
        candidate = Resolve(layer.Value<uint64_t>("data"));
    }
    return candidate;
  }
};
struct Object {
  uint64_t address;
  std::shared_ptr<Node> node;
  Json props;
  std::string id;
  Matrix world;
};
// Qhull creates the explicit convex proxy from saved mesh points. It uses a
// reentrant context so independent fuzz worlds can load safely in parallel.
std::vector<SimTriangle> convex_faces(const std::vector<SimVec3> &vertices) {
  std::vector<double> coordinates;
  for (auto p : vertices)
    coordinates.insert(coordinates.end(), {p.x, p.y, p.z});
  auto close = [](FILE *f) { std::fclose(f); };
  auto log = std::unique_ptr<FILE, decltype(close)>(std::tmpfile(), close);
  require(bool(log), "cannot create convex hull diagnostics");
  qhT context;
  qhT *qh = &context;
  qh_zero(qh, log.get());
  char flags[] = "qhull Qt";
  int error = qh_new_qhull(qh, 3, vertices.size(), coordinates.data(), 0, flags,
                           nullptr, log.get());
  std::vector<SimTriangle> result;
  if (!error) {
    facetT *facet;
    FORALLfacets {
      vertexT *vertex, **vertexp;
      std::vector<uint32_t> ids;
      FOREACHvertex_(facet->vertices)
          ids.push_back(qh_pointid(qh, vertex->point));
      if (ids.size() != 3) {
        error = 1;
        break;
      }
      auto a = vertices[ids[0]], b = vertices[ids[1]], c = vertices[ids[2]];
      SimVec3 normal{facet->normal[0], facet->normal[1], facet->normal[2]};
      if (dot(cross(b - a, c - a), normal) < 0)
        std::swap(ids[1], ids[2]);
      result.push_back({ids[0], ids[1], ids[2]});
    }
  }
  qh_freeqhull(qh, !qh_ALL);
  int current = 0, total = 0;
  qh_memfreeshort(qh, &current, &total);
  if (error || result.empty())
    Fail("degenerate convex collision mesh");
  return result;
}
void check_calibration(const RobotDefinition &definition,
                       const std::vector<SimVec3> &points,
                       const std::string &id, uint32_t kind, bool collisions) {
  auto references = std::span(definition.shapes).first(definition.shape_count);
  auto found = std::find_if(references.begin(), references.end(),
                            [&](const auto &part) { return part.id == id; });
  if (found == references.end() || found->kind != kind || found->collisions != collisions)
    Fail(id + ": vehicle geometry changed; recalibrate the physics profile");
  auto matches = [](const auto &a, const auto &b) {
    return std::all_of(a.begin(), a.end(), [&](auto p) {
      return std::any_of(b.begin(), b.end(), [&](auto q) { return norm(p-q) <= 1e-4; });
    });
  };
  if (!matches(points, found->points) || !matches(found->points, points))
    Fail(id + ": vehicle geometry changed; recalibrate the physics profile");
}
Collider read_collider(Reader &r, Main &scene, const Object &obj, Matrix world,
               Matrix body_world, const RobotDefinition *definition) {
  if (obj.node->Value<int16_t>("type") != 1)
    Fail(obj.node->Name() + ": collider must be a mesh");
  if (obj.node->Field("modifiers").Value<uint64_t>("first"))
    Fail(obj.node->Name() +
         ": apply modifiers before saving for native loading");
  auto mesh = r.At(obj.node->Value<uint64_t>("data"), "Mesh");
  if (mesh.Value<uint64_t>("key"))
    Fail(obj.node->Name() +
         ": apply shape keys before saving for native loading");
  int nv = mesh.Value<int>("totvert"), nf = mesh.Value<int>("totpoly"),
      nc = mesh.Value<int>("totloop");
  if (nv < 4 || nv > 1000000 || nf < 1 || nc < 3 || nc > 6000000)
    Fail(obj.node->Name() + ": empty/excessive mesh");
  auto positions = r.Layer(mesh, "vdata", "position", 48);
  auto corners = r.Layer(mesh, "ldata", ".corner_vert");
  auto offsets = r.Resolve(mesh.Value<uint64_t>("poly_offset_indices"));
  if (positions.size() / 12 < size_t(nv) || corners.size() / 4 < size_t(nc) ||
      offsets.size() / 4 < size_t(nf) + 1)
    Fail("truncated saved mesh attributes");
  Matrix local = glm::inverse(rigid(body_world)) * world;
  Matrix part = rigid(local), residual = glm::inverse(part) * local;
  std::vector<SimVec3> vertices, calibration;
  for (int i = 0; i < nv; ++i) {
    auto p = Scalar<std::array<float, 3>>(positions.subspan(size_t(i) * 12));
    glm::vec3 point(p[0], p[1], p[2]);
    vertices.push_back(ned(glm::vec3(residual * glm::vec4(point, 1))));
    calibration.push_back(ned(glm::vec3(local * glm::vec4(point, 1))));
    if (!finite(vertices.back()) || !finite(calibration.back()))
      Fail(obj.id + ": non-finite saved vertex");
  }
  ShapeRec shape;
  // Even empty primitive ranges use the current array cursor, matching the
  // canonical C ABI encoding used by resolved replay and read-only tools.
  shape.vertex_begin = scene.vertices.size();
  shape.triangle_begin = scene.triangles.size();
  shape.pose = pose(part);
  shape.collisions = obj.props.value("sim_collision", false);
  shape.friction = obj.props.value("sim_friction", 1.0);
  shape.restitution = obj.props.value("sim_restitution", 0.0);
  auto kind = obj.props.value("sim_shape", std::string("mesh"));
  if (kind != "box" && kind != "sphere" && kind != "cylinder" &&
      kind != "mesh" && kind != "convex")
    Fail(obj.id + ": unknown sim_shape");
  if (shape.collisions && kind == "mesh")
    Fail(obj.id + ": collisions require an explicit primitive or convex proxy");
  if (kind == "box" || kind == "sphere" || kind == "cylinder") {
    SimVec3 lo = vertices[0], hi = lo;
    for (auto p : vertices) {
      lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
      hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    shape.size = (hi - lo) * .5;
    shape.pose.position = point(shape.pose, (hi + lo) * .5);
    shape.kind = kind == "box"      ? SIM_BOX
                 : kind == "sphere" ? SIM_SPHERE
                                    : SIM_CYLINDER;
    if (std::min({shape.size.x, shape.size.y, shape.size.z}) <= 1e-7)
      Fail(obj.id + ": degenerate primitive");
    if (kind == "box")
      for (auto p : vertices) {
        if (std::min(std::abs(p.x - lo.x), std::abs(p.x - hi.x)) > 1e-5 ||
            std::min(std::abs(p.y - lo.y), std::abs(p.y - hi.y)) > 1e-5 ||
            std::min(std::abs(p.z - lo.z), std::abs(p.z - hi.z)) > 1e-5)
          Fail(obj.id + ": geometry is not a box; use a convex proxy");
      }
    if (kind == "sphere" &&
        std::max({shape.size.x, shape.size.y, shape.size.z}) -
                std::min({shape.size.x, shape.size.y, shape.size.z}) >
            1e-5)
      Fail(obj.id + ": nonuniform sphere scale");
    if (kind == "cylinder" && std::abs(shape.size.x - shape.size.y) > 1e-5)
      Fail(obj.id + ": elliptical cylinder");
  } else {
    shape.kind = SIM_MESH;
    std::vector<SimTriangle> triangles;
    if (kind == "convex")
      triangles = convex_faces(vertices);
    else {
      // Stored faces are triangulated locally. Collision-enabled concave meshes
      // are rejected above; these triangles describe observation geometry.
      for (int i = 0; i < nf; ++i) {
        int begin = Scalar<int>(offsets.subspan(size_t(i) * 4)),
            end = Scalar<int>(offsets.subspan(size_t(i + 1) * 4));
        if (begin < 0 || end > nc || end - begin < 3)
          Fail(obj.id + ": invalid polygon range");
        std::vector<uint32_t> face;
        for (int j = begin; j < end; ++j) {
          auto index = Scalar<uint32_t>(corners.subspan(size_t(j) * 4));
          if (index >= uint32_t(nv))
            Fail(obj.id + ": mesh index outside vertex buffer");
          face.push_back(index);
        }
        for (size_t j = 1; j + 1 < face.size(); ++j)
          triangles.push_back({face[0], face[j], face[j + 1]});
      }
      if (glm::determinant(glm::mat3(residual)) < 0)
        for (auto &t : triangles)
          std::swap(t.a, t.c);
    }
    shape.vertex_begin = scene.vertices.size();
    shape.vertex_count = vertices.size();
    shape.triangle_begin = scene.triangles.size();
    shape.triangle_count = triangles.size();
    scene.vertices.append(vertices);
    scene.triangles.append(triangles);
  }
  if (definition) {
    check_calibration(*definition, calibration, obj.id, shape.kind, shape.collisions);
  }
  return Collider{shape};
}
} // namespace
MainPtr Main::load_blend(const std::string &filename) {
  // Parse Blender's DNA schema and stored datablocks. This reconstructs static
  // transforms and geometry; it never executes Python, drivers or modifiers.
  std::string file_hash;
  auto bytes = ReadBytes(filename, &file_hash);
  if (bytes.size() < 12 || String(Bytes(bytes).first(7)) != "BLENDER")
    Fail("not a Blender file");
  if (bytes[7] != '-' || bytes[8] != 'v' || bytes[9] != '4')
    Fail("only little-endian, 64-bit Blender 4.x files are supported");
  auto file = cblend::Blend::Read(bytes);
  if (!file)
    Fail("CBlend could not parse the saved file");
  Reader r(*file);
  const char *requested = std::getenv("AUTONOMY_SIM_SOURCE_SCENE");
  std::string scene_name =
      requested && *requested ? requested : "AutonomySource";
  std::optional<Node> source;
  for (auto &block : file->GetBlocks(cblend::BLOCK_CODE_SC)) {
    auto n = r.At(block.header.address, "Scene");
    if (n.Name() == scene_name)
      source.emplace(n);
  }
  if (!source)
    Fail("authoring scene not found: " + scene_name);
  auto properties = r.Properties(*source);
  if (properties.contains("sim_robot")) {
    const auto name = properties.at("sim_robot").get<std::string>();
    const auto *robot = robot_definition(name);
    if (!robot)
      Fail("unknown fixed robot: " + name);
    if (file_hash != robot_asset_hash(name))
      Fail(name + ": robot .blend SHA-256 mismatch; use the registered asset "
                  "or rebuild its configuration and checksums together");
    if (properties.value("sim_robot_config_sha256", std::string{}) !=
            robot_configuration_hash(*robot))
      Fail(name + ": compiled robot configuration SHA-256 mismatch");
  }
  if (properties.value("sim_schema", 0) != 1 ||
      properties.value("sim_preview", false))
    Fail("scene has no simulation authoring contract");
  if (std::abs(source->Field("unit").Value<float>("scale_length") - 1) > 1e-8)
    Fail("scene must use metre units");
  std::set<uint64_t> seen;
  auto collection =
      r.FindCollection(source->Value<uint64_t>("master_collection"),
                       properties.at("sim_collection"), seen);
  if (!collection)
    Fail("missing simulation source collection");
  seen.clear();
  auto addresses = r.Objects(collection, seen);
  // Only members of the designated simulation collection become physical
  // entities. Saved preview scenes and decorative objects cannot add bodies.
  std::vector<Object> objects;
  std::set<std::string> identifiers;
  const std::set<std::string> roles = {"body",   "collider", "thruster",
                                       "sensor", "visual",   "instance"};
  for (auto address : addresses) {
    auto node = r.At(address, "Object");
    auto props = r.Properties(node);
    auto role = props.value("sim_role", std::string{});
    if (!roles.contains(role))
      Fail(node.Name() + ": unclassified object in simulation collection");
    if (role == "visual")
      continue;
    auto id = props.value("sim_id", std::string{});
    if (id.empty() || !identifiers.insert(id).second)
      Fail(node.Name() + ": missing or duplicate sim_id");
    if (role == "instance") {
      auto group = node.Value<uint64_t>("dup_group");
      if (!group)
        Fail(node.Name() + ": instance requires a collection");
      auto offset = r.Vector(r.At(group, "Collection"), "dupli_ofs");
      Matrix placement = r.World(address) * glm::translate(Matrix(1), -offset);
      std::set<uint64_t> nesting;
      for (auto prototype : r.Objects(group, nesting)) {
        auto member = r.At(prototype, "Object");
        auto attributes = r.Properties(member);
        if (attributes.value("sim_role", std::string{}) == "visual")
          continue;
        if (attributes.value("sim_role", std::string{}) != "body" ||
            member.Value<int16_t>("type") != 1 ||
            member.Value<uint64_t>("parent") ||
            attributes.contains("sim_profile") ||
            attributes.value("sim_motion", std::string("static")) != "static")
          Fail(node.Name() + ": instances require standalone static mesh "
                             "bodies; realize this instance before saving");
        auto child_id = id + "/" + attributes.at("sim_id").get<std::string>();
        if (!identifiers.insert(child_id).second)
          Fail("duplicate instance ID: " + child_id);
        objects.push_back({prototype, std::make_shared<Node>(member),
                           attributes, child_id,
                           placement * r.World(prototype)});
      }
      continue;
    }
    objects.push_back(
        {address, std::make_shared<Node>(node), props, id, r.World(address)});
  }
  std::sort(objects.begin(), objects.end(),
            [](auto &a, auto &b) { return a.id < b.id; });
  MainBuilder builder;
  auto &result = builder.draft();
  result.path = std::filesystem::absolute(filename).string();
  result.header.gravity = {0, 0, properties.value("sim_gravity", 9.81)};
  result.header.water_density = properties.value("sim_water_density", 1025.0);
  result.header.dt_denominator = count_property(properties, "sim_hz", 360);
  result.header.hydro_period =
      count_property(properties, "sim_hydro_period", 7);
  result.header.source_frame = 1;
  if (const char *frame = std::getenv("AUTONOMY_SIM_SOURCE_FRAME")) {
    uint32_t n = 0;
    auto end = frame + std::strlen(frame);
    auto [last, error] = std::from_chars(frame, end, n);
    if (error != std::errc() || last != end || n > 2147483647)
      Fail("invalid Blender source frame");
    result.header.source_frame = n;
  }
  copy_text(result.header.source_kind, sizeof(result.header.source_kind),
            "blender_native");
  copy_text(result.header.scene_name, sizeof(result.header.scene_name),
            scene_name.c_str());
  std::map<std::string, size_t> owners;
  std::map<std::string, const RobotDefinition *> profiles;
  // Build bodies first so attachments can resolve stable ownership IDs. The
  // profile supplies measured physical parameters, while the saved Blender
  // objects supply the complete body list, placements and collision geometry.
  for (auto &obj : objects) {
    if (obj.props.at("sim_role") != "body")
      continue;
    auto name = obj.props.value("sim_profile", std::string{});
    const auto *definition = name.empty() ? nullptr : robot_definition(name);
    if (!name.empty() && !definition)
      Fail("missing profile for unknown fixed robot " + name);
    if (definition) {
      if (result.robot.definition)
        Fail("only one fixed robot is supported per scene");
      result.robot.definition = definition;
    }
    auto motion = obj.props.value("sim_motion", std::string("static"));
    if (motion != "static" && motion != "dynamic" && motion != "kinematic")
      Fail(obj.id + ": invalid sim_motion");
    if (motion == "dynamic" && !definition)
      Fail(obj.id + ": dynamic body requires calibrated profile");
    BodyRec body = definition ? definition->body : BodyRec{};
    body.id = body_id(obj.id);
    auto label = obj.props.value("sim_name", obj.node->Name());
    copy_text(body.name, sizeof(body.name), label.c_str());
    body.motion = motion == "dynamic"     ? SIM_DYNAMIC
                  : motion == "kinematic" ? SIM_KINEMATIC
                                          : SIM_STATIC;
    body.cls = obj.props.value("sim_class", 0u);
    body.tracked = obj.props.value("sim_tracked", false);
    body.fuzz = obj.props.value("sim_fuzz", false);
    body.initial_pose = pose(obj.world);
    std::vector<Collider> colliders;
    if (obj.node->Value<int16_t>("type") == 1)
      colliders.push_back(read_collider(r, result, obj, obj.world, obj.world, definition));
    for (auto &part : objects)
      if (part.props.at("sim_role") == "collider" &&
          part.props.value("sim_body", std::string{}) == obj.id)
        colliders.push_back(read_collider(r, result, part, part.world, obj.world, definition));
    body.shape_count = colliders.size();
    if (definition && body.shape_count != definition->shape_count)
      Fail(obj.id +
           ": collider count changed; recalibrate the physics profile");
    auto index = result.objects.size();
    owners.emplace(obj.id, index);
    profiles.emplace(obj.id, definition);
    auto &object = result.add_object(body, colliders, definition != nullptr);
    if (definition)
      for (const auto &h : std::span(definition->hulls).first(definition->hull_count)) {
        HullRec hull = h.physics;
        hull.body = index;
        hull.vertex_begin = result.hull_vertices.size();
        if (!h.positions.empty())
          result.hull_vertices.append(h.positions);
        else {
          if (h.collider >= body.shape_count) Fail("invalid fixed robot hull collider");
          auto &shape = object.colliders[h.collider].data;
          auto faces = surface(shape.kind, shape.size,
                               shape.vertex_count ? result.vertices.data() + shape.vertex_begin : nullptr,
                               shape.triangle_count ? result.triangles.data() + shape.triangle_begin : nullptr,
                               shape.vertex_count, shape.triangle_count);
          for (const auto &face : faces)
            result.hull_vertices.append(face);
        }
        hull.vertex_count = result.hull_vertices.size() - hull.vertex_begin;
        result.robot.add_hull(hull);
      }
  }
  std::array<std::pair<std::pair<size_t, int>, ThrusterRec>, Robot::max_thrusters> thrusters;
  size_t thruster_count = 0;
  size_t cameras = 0;
  for (auto &obj : objects) {
    auto role = obj.props.at("sim_role").get<std::string>();
    if (role != "collider" && role != "thruster" && role != "sensor")
      continue;
    auto owner = obj.props.value("sim_body", std::string{});
    if (!owners.contains(owner))
      Fail(obj.id + ": missing owning body " + owner);
    auto index = owners.at(owner);
    if (role == "collider")
      continue;
    auto root = std::find_if(objects.begin(), objects.end(),
                             [&](auto &b) { return b.id == owner; });
    Matrix local = glm::inverse(rigid(root->world)) * obj.world;
    auto mount = pose(local);
    if (role == "thruster") {
      int slot = obj.props.value("sim_index", -1);
      const auto *definition = profiles.at(owner);
      if (!definition || slot < 0 || size_t(slot) >= definition->thruster_count)
        Fail(obj.id + ": missing thruster calibration/index");
      ThrusterRec thruster = definition->thrusters[slot];
      thruster.body = index;
      if (!properties.contains("sim_robot")) {
        // The editable pool preserves its saved attachment transforms; the
        // checksum-bound standalone assets use the fixed C++ mount values.
        thruster.position = mount.position;
        thruster.direction = rotate(mount.orientation, {1, 0, 0});
        thruster.direction = thruster.direction / norm(thruster.direction);
      }
      if (thruster_count == thrusters.size()) Fail("too many fixed robot thrusters");
      thrusters[thruster_count++] = {{index, slot}, thruster};
    } else {
      uint64_t address = obj.address;
      if (obj.node->Value<int16_t>("type") != 11) {
        address = 0;
        for (auto &block : file->GetBlocks(cblend::BLOCK_CODE_OB)) {
          auto child = r.At(block.header.address, "Object");
          if (child.Value<uint64_t>("parent") == obj.address &&
              child.Value<int16_t>("type") == 11) {
            address = block.header.address;
            break;
          }
        }
      }
      if (!address)
        Fail(obj.id + ": sensor requires a camera datablock or child");
      auto camera_obj = r.At(address, "Object");
      auto camera = r.At(camera_obj.Value<uint64_t>("data"), "Camera");
      if (camera.Value<uint8_t>("type") != 0)
        Fail(obj.id + ": simulation camera must be perspective");
      Matrix optical = glm::inverse(rigid(root->world)) *
                       rigid(r.World(address)) *
                       glm::rotate(Matrix(1), float(std::acos(-1.0) / 2),
                                   glm::vec3(0, 0, 1));
      result.header.camera.body = index;
      result.header.camera.pose = pose(optical);
      result.header.camera.width = count_property(obj.props, "sim_width", 800);
      result.header.camera.height =
          count_property(obj.props, "sim_height", 600);
      result.header.camera.horizontal_fov =
          2 *
          std::atan(double(camera.Value<float>("sensor_x")) /
                    (2 * camera.Value<float>("lens"))) *
          180 / std::acos(-1.0);
      if (properties.contains("sim_robot")) {
        const auto *definition = profiles.at(owner);
        if (!definition) Fail("camera has no fixed robot definition");
        result.header.camera = definition->camera;
        result.header.camera.body = index;
      }
      ++cameras;
    }
  }
  if (cameras != 1)
    Fail("exactly one simulation camera is required");
  auto active_thrusters = std::span(thrusters).first(thruster_count);
  std::sort(active_thrusters.begin(), active_thrusters.end(),
            [](auto &a, auto &b) { return a.first < b.first; });
  for (auto &[owner, index] : owners) {
    const auto *definition = profiles.at(owner);
    auto expected = definition ? definition->thruster_count : 0;
    int slot = 0;
    for (auto &t : active_thrusters)
      if (t.first.first == index) {
        if (t.first.second != slot++)
          Fail(owner + ": missing/duplicate thruster mounts");
        result.robot.add_thruster(t.second);
      }
    if (size_t(slot) != expected)
      Fail(owner + ": missing thruster mounts");
  }
  auto result_owner = builder.finish();
  // Hash the constructed model, not its filesystem path or unrelated previews.
  // Relocating the same saved source leaves its physical identity unchanged.
  auto packed = result.binary();
  auto identity = sha256(packed.data(), packed.size());
  std::memcpy(result.header.content_hash, identity.data(), identity.size());
  return result_owner;
}
} // namespace sim
