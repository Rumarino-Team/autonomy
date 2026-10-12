#include "blend_scene.hpp"

#include <cblend.hpp>
#include <yaml-cpp/yaml.h>
#include <zlib.h>
#include <zstd.h>
#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace blender_stonefish_cpp {
namespace {
using Bytes = cblend::MemorySpan;
[[noreturn]] void Fail(const std::string& message) { throw std::runtime_error("[blender_cpp] " + message); }
template<class T> T Scalar(Bytes bytes) {
    if (bytes.size() < sizeof(T)) Fail("truncated field or array");
    T value;
    std::memcpy(&value, bytes.data(), sizeof(T));
    return value;
}
std::string String(Bytes bytes) {
    if (bytes.empty()) return {};
    auto end = std::find(bytes.begin(), bytes.end(), 0);
    return {reinterpret_cast<const char*>(bytes.data()), static_cast<size_t>(end - bytes.begin())};
}
struct Node {
    cblend::BlendType type;
    Bytes bytes;
    Node Field(std::string_view name) const {
        auto field = type.GetField(name);
        if (!field) Fail("missing DNA field: " + std::string(name));
        auto data = field->GetData(bytes);
        if (data.empty()) Fail("invalid DNA field: " + std::string(name));
        return {field->GetFieldType(), data};
    }
    template<class T> T Value(std::string_view name) const { return Scalar<T>(Field(name).bytes); }
    std::string Text(std::string_view name) const { return String(Field(name).bytes); }
};
class Reader {
public:
    explicit Reader(cblend::Blend& blend) : blend(blend) {
        for (auto code : {cblend::BLOCK_CODE_DATA, cblend::BLOCK_CODE_OB, cblend::BLOCK_CODE_ME})
            for (const auto& block : blend.GetBlocks(code))
                if (block.header.address) blocks.emplace(block.header.address, &block);
    }
    Bytes Resolve(uint64_t address) const {
        if (!address) return {};
        auto it = blocks.upper_bound(address);
        if (it == blocks.begin()) Fail("unresolved Blender pointer");
        --it;
        const auto offset = address - it->first;
        if (offset >= it->second->body.size()) Fail("Blender pointer outside its data block");
        return Bytes(it->second->body).subspan(offset);
    }
    Node At(uint64_t address, std::string_view type_name) const {
        auto type = blend.GetType(type_name);
        if (!type) Fail("missing DNA type: " + std::string(type_name));
        auto bytes = Resolve(address);
        if (bytes.size() < type->GetSize()) Fail("truncated " + std::string(type_name));
        return {*type, bytes.first(type->GetSize())};
    }
    std::unordered_map<std::string, std::string> Properties(const Node& object) const {
        std::unordered_map<std::string, std::string> result;
        auto root = object.Field("id").Value<uint64_t>("properties");
        if (!root) return result;
        auto group = At(root, "IDProperty");
        if (group.Value<uint8_t>("type") != 6) Fail("object properties are not an IDProperty group");
        uint64_t ptr = group.Field("data").Field("group").Value<uint64_t>("first");
        std::set<uint64_t> visited;
        while (ptr) {
            if (!visited.insert(ptr).second || visited.size() > 4096) Fail("invalid IDProperty list");
            auto prop = At(ptr, "IDProperty");
            auto data = prop.Field("data");
            std::string value;
            switch (prop.Value<uint8_t>("type")) {
            case 0: value = String(Resolve(data.Value<uint64_t>("pointer"))); break;
            case 1: case 10: value = std::to_string(data.Value<int32_t>("val")); break;
            case 2: value = std::to_string(Scalar<float>(data.Field("val").bytes)); break;
            case 8: {
                auto val = data.Field("val").bytes;
                value = std::to_string(Scalar<double>(data.bytes.subspan(val.data() - data.bytes.data())));
                break;
            }
            default: break;
            }
            result[prop.Text("name")] = value;
            ptr = prop.Value<uint64_t>("next");
        }
        return result;
    }
    Bytes Layer(const Node& mesh, std::string_view domain, std::string_view name, int fallback_type = -1) const {
        auto cd = mesh.Field(domain);
        int count = cd.Value<int>("totlayer");
        auto ptr = cd.Value<uint64_t>("layers");
        if (count < 0 || count > 4096) Fail("invalid CustomData layer count");
        if (!count) return {};
        auto type = blend.GetType("CustomDataLayer");
        if (!type) Fail("missing CustomDataLayer");
        auto bytes = Resolve(ptr);
        if (static_cast<size_t>(count) > bytes.size() / type->GetSize()) Fail("truncated CustomData layers");
        Bytes fallback;
        for (int i = 0; i < count; ++i) {
            Node layer{*type, bytes.subspan(i * type->GetSize(), type->GetSize())};
            if (layer.Text("name") == name) return Resolve(layer.Value<uint64_t>("data"));
            if (fallback_type >= 0 && layer.Value<int>("type") == fallback_type && fallback.empty())
                fallback = Resolve(layer.Value<uint64_t>("data"));
        }
        return fallback;
    }
    Mesh ReadMesh(uint64_t address, const Vec3& scale) const {
        Node mesh = At(address, "Mesh");
        const int nv = mesh.Value<int>("totvert"), nf = mesh.Value<int>("totpoly"), nc = mesh.Value<int>("totloop");
        if (nv <= 0 || nf <= 0 || nc <= 0) Fail("mesh has no vertices or faces");
        auto positions = Layer(mesh, "vdata", "position", 48);
        auto corners = Layer(mesh, "ldata", ".corner_vert");
        auto uvs = Layer(mesh, "ldata", "UVMap", 49);
        auto offsets = Resolve(mesh.Value<uint64_t>("poly_offset_indices"));
        if (positions.size() / 12 < static_cast<size_t>(nv) || corners.size() / 4 < static_cast<size_t>(nc)
            || offsets.size() / 4 < static_cast<size_t>(nf) + 1) Fail("unsupported or truncated Blender 4.x mesh attributes");
        if (!uvs.empty() && uvs.size() / 8 < static_cast<size_t>(nc)) Fail("truncated UV attribute");
        Mesh out;
        out.vertices.reserve(nv);
        for (int i = 0; i < nv; ++i) {
            auto v = Scalar<std::array<float, 3>>(positions.subspan(i * 12));
            Vec3 p{v[0] * scale[0], v[1] * scale[1], -v[2] * scale[2]};
            for (double component : p) if (!std::isfinite(component)) Fail("non-finite mesh vertex");
            out.vertices.push_back(p);
        }
        if (Scalar<int>(offsets) != 0 || Scalar<int>(offsets.subspan(nf * 4)) != nc) Fail("invalid polygon offsets");
        for (int i = 0; i < nf; ++i) {
            const int first = Scalar<int>(offsets.subspan(i * 4)), last = Scalar<int>(offsets.subspan((i + 1) * 4));
            if (first < 0 || last > nc || last - first < 3) Fail("invalid polygon corner range");
            std::vector<uint32_t> face;
            std::vector<Vec2> face_uv;
            for (int j = first; j < last; ++j) {
                uint32_t vertex = Scalar<uint32_t>(corners.subspan(j * 4));
                if (vertex >= static_cast<uint32_t>(nv)) Fail("polygon references a missing vertex");
                face.push_back(vertex);
                if (!uvs.empty()) {
                    auto uv = Scalar<std::array<float, 2>>(uvs.subspan(j * 8));
                    face_uv.push_back({uv[0], uv[1]});
                }
            }
            if (-scale[0] * scale[1] * scale[2] < 0) {
                std::reverse(face.begin(), face.end());
                std::reverse(face_uv.begin(), face_uv.end());
            }
            out.faces.push_back(std::move(face));
            out.face_uvs.push_back(std::move(face_uv));
        }
        return out;
    }
    cblend::Blend& blend;
    std::map<uint64_t, const cblend::Block*> blocks;
};
struct Known { const char* name; const char* material; const char* look; };
const std::unordered_map<std::string, Known> known = {
    {"pool_tile", {"PoolShell", "ceramic", "pool_tile"}}, {"pool_deck", {"PoolDeck", "concrete", "deck"}},
    {"pool_stainless", {"PoolFittings", "steel", "stainless"}}, {"pool_band", {"PoolBand", "ceramic", "pool_band"}},
    {"pool_coping", {"PoolCoping", "stone", "coping"}}, {"pool_light", {"PoolLights", "glass", "light"}},
    {"pool_glass", {"PoolWindow", "glass", "glass"}}, {"pool_hall", {"PoolHall", "concrete", "hall"}}
};
std::vector<uint8_t> ReadBytes(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);

    if (!file || file.tellg() <= 0) Fail("cannot read " + path.string());
    std::vector<uint8_t> raw(static_cast<size_t>(file.tellg()));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(raw.data()), raw.size())) Fail("cannot read complete blend file");
    if (raw.size() >= 2 && raw[0] == 0x1f && raw[1] == 0x8b) {
        gzFile gzip = gzopen(path.c_str(), "rb");
        if (!gzip) Fail("cannot open gzip blend");
        std::vector<uint8_t> out;
        std::array<uint8_t, 65536> buffer;
        int n;
        while ((n = gzread(gzip, buffer.data(), buffer.size())) > 0) out.insert(out.end(), buffer.begin(), buffer.begin() + n);
        int status = gzclose(gzip);
        if (n < 0 || status != Z_OK) Fail("invalid gzip blend");
        return out;
    }
    if (raw.size() >= 4 && Scalar<uint32_t>(raw) == ZSTD_MAGICNUMBER) {
        auto* stream = ZSTD_createDStream();
        if (!stream) Fail("cannot allocate zstd decoder");
        std::unique_ptr<ZSTD_DStream, decltype(&ZSTD_freeDStream)> guard(stream, ZSTD_freeDStream);
        ZSTD_initDStream(stream);
        ZSTD_inBuffer input{raw.data(), raw.size(), 0};
        std::vector<uint8_t> out;
        std::array<uint8_t, 65536> buffer;
        size_t remaining = 1;
        while (input.pos < input.size || remaining) {
            ZSTD_outBuffer output{buffer.data(), buffer.size(), 0};
            size_t before = input.pos;
            remaining = ZSTD_decompressStream(stream, &output, &input);
            if (ZSTD_isError(remaining)) Fail("invalid zstd blend");
            out.insert(out.end(), buffer.begin(), buffer.begin() + output.pos);
            if (input.pos == before && !output.pos) Fail("truncated zstd blend");
        }
        return out;
    }
    return raw;
}
bool Truth(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
    return value == "true" || value == "1";
}
} // namespace

std::string MeshFingerprint(const Mesh& mesh) {
    std::string key;
    auto append = [&](const auto& value) { key.append(reinterpret_cast<const char*>(&value), sizeof(value)); };
    append(mesh.vertices.size());
    for (auto vertex : mesh.vertices) append(vertex);
    append(mesh.faces.size());
    for (const auto& face : mesh.faces) { append(face.size()); for (auto index : face) append(index); }
    for (const auto& face : mesh.face_uvs) { append(face.size()); for (auto uv : face) append(uv); }
    return key;
}

Scene ReadScene(const std::filesystem::path& filename, const std::filesystem::path& config_path) {
    auto config = YAML::LoadFile(config_path.string());
    auto bytes = ReadBytes(filename);
    //Magic Layout Explanation:
    // 0-6: "BLENDER"
    // 7: '-'
    // 8: 'v'
    // 9-11: version
    // 12-15: header size
    // 16-19: footer size
    // 20-23: total size
    // 24-27: data offset
    if (bytes.size() < 12 || String(Bytes(bytes).first(7)) != "BLENDER") Fail("not a blender file");
    const std::string version(reinterpret_cast<const char*>(bytes.data() + 9), 3);
    if (version[0] != '4')
        Fail("this importer currently supports Blender 4.x mesh layouts; file version is " + version);

    auto blend = cblend::Blend::Read(bytes);
    if (!blend) Fail("CBlend could not parse the file (error category " + std::to_string(blend.error().index()) + ")");
    Reader reader(*blend);
    Scene scene;
    std::set<std::string> entity_names;
    std::set<std::string> entity_ids;
    std::map<std::tuple<uint64_t, double, double, double>, std::shared_ptr<const Mesh>> meshes;
    std::unordered_map<std::string, std::shared_ptr<const Mesh>> geometry;
    const double mesh_scale = config["meshes"]["scale"].as<double>(1.0);
    if (!std::isfinite(mesh_scale) || mesh_scale <= 0) Fail("meshes.scale must be finite and positive");
    for (const auto& block : blend->GetBlocks(cblend::BLOCK_CODE_OB)) {
        auto block_type = blend->GetBlockType(block);
        if (!block_type) Fail("invalid Object block type");
        Node node{*block_type, block.body};
        const auto type = node.Value<int16_t>("type");
        auto name = node.Field("id").Text("name").substr(2);
        if (type == 11) {
            if (node.Value<uint64_t>("parent") != 0 || node.Value<int16_t>("rotmode") != 1) {
                std::cerr << "[blender_cpp] skipping camera " << name
                          << ": unparent it and use XYZ Euler rotation\n";
                continue;
            }
            ViewCamera camera;
            camera.name = name;
            auto loc = Scalar<std::array<float, 3>>(node.Field("loc").bytes);
            auto rot = Scalar<std::array<float, 3>>(node.Field("rot").bytes);
            camera.location = {loc[0], loc[1], loc[2]};
            camera.rotation = {rot[0], rot[1], rot[2]};
            scene.cameras.push_back(std::move(camera));
            continue;
        }
        if (type != 1) continue;
        auto props = reader.Properties(node);
        auto flag = props["stonefish"];
        std::transform(flag.begin(), flag.end(), flag.begin(), [](unsigned char c) { return std::tolower(c); });
        // Every mesh is a static. stonefish set to 0 or false leaves an object out.
        if (flag == "0" || flag == "false") continue;
        if (node.Value<uint64_t>("parent") != 0) Fail(name + ": apply the parent transform before importing");
        if (node.Value<int16_t>("rotmode") != 1) Fail(name + ": use XYZ Euler rotation for this importer");
        if (node.Field("modifiers").Value<uint64_t>("first")) Fail(name + ": apply modifiers before importing stored mesh data");
        Object object;
        object.blender_name = name;
        auto get = [&](const char* key, const std::string& fallback) { return props[key].empty() ? fallback : props[key]; };
        object.name = get("stonefish_name", name);
        object.material = get("material", config["defaults"]["material"].as<std::string>("steel"));
        object.look = get("look", config["defaults"]["look"].as<std::string>("gray"));
        object.cls = props["cls"];
        object.convex = Truth(props["convex"]);
        if (known.contains(name) && props["stonefish_name"].empty()) {
            const auto& fallback = known.at(name);
            object.name = fallback.name; object.material = fallback.material; object.look = fallback.look; object.cls = "scenery";
        }
        if (object.name.starts_with("__hot_reload_")) Fail("reserved entity name: " + object.name);
        if (!entity_names.insert(object.name).second) Fail("duplicate entity name: " + object.name);
        object.id = get("stonefish_id", object.name);
        if (!entity_ids.insert(object.id).second) Fail("duplicate stonefish_id: " + object.id);
        auto loc = Scalar<std::array<float, 3>>(node.Field("loc").bytes);
        auto rot = Scalar<std::array<float, 3>>(node.Field("rot").bytes);
        auto size = Scalar<std::array<float, 3>>(node.Field("size").bytes);
        object.position = {-loc[0], loc[1], -loc[2]};
        object.rotation = {-rot[0], rot[1], -rot[2]};
        for (size_t i = 0; i < 3; ++i) {
            object.scale[i] = size[i] * mesh_scale;
            if (!std::isfinite(object.position[i]) || !std::isfinite(object.rotation[i])
                || !std::isfinite(object.scale[i]) || object.scale[i] == 0) Fail(name + ": invalid object transform");
        }
        const auto address = node.Value<uint64_t>("data");
        auto key = std::tuple(address, object.scale[0], object.scale[1], object.scale[2]);
        auto it = meshes.find(key);
        if (it == meshes.end()) {
            auto candidate = std::make_shared<Mesh>(reader.ReadMesh(address, object.scale));
            auto [shared, inserted] = geometry.emplace(MeshFingerprint(*candidate), candidate);
            it = meshes.emplace(key, shared->second).first;
        }
        object.mesh = it->second;
        scene.objects.push_back(std::move(object));
    }
    return scene;
}

void WriteObj(const Mesh& mesh, const std::filesystem::path& filename) {
    std::filesystem::create_directories(filename.parent_path());
    std::ofstream file(filename);
    if (!file) Fail("cannot write " + filename.string());
    file << "# Exported by blender_stonefish_cpp\n" << std::fixed << std::setprecision(6);
    for (auto p : mesh.vertices) file << "v " << p[0] << ' ' << p[1] << ' ' << p[2] << '\n';
    bool textured = mesh.face_uvs.size() == mesh.faces.size();
    for (size_t i = 0; i < mesh.faces.size() && textured; ++i) textured = mesh.face_uvs[i].size() == mesh.faces[i].size();
    if (textured) for (const auto& face : mesh.face_uvs) for (auto uv : face) file << "vt " << uv[0] << ' ' << uv[1] << '\n';
    for (const auto& face : mesh.faces) {
        auto a = mesh.vertices.at(face[0]), b = mesh.vertices.at(face[1]), c = mesh.vertices.at(face[2]);
        Vec3 ab{b[0]-a[0], b[1]-a[1], b[2]-a[2]}, ac{c[0]-a[0], c[1]-a[1], c[2]-a[2]};
        Vec3 normal{ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]};
        double length = std::hypot(normal[0], normal[1], normal[2]);
        if (length < 1e-12) normal = {0, 0, 1}; else for (double& x : normal) x /= length;
        file << "vn " << normal[0] << ' ' << normal[1] << ' ' << normal[2] << '\n';
    }
    size_t uv = 1;
    for (size_t f = 0; f < mesh.faces.size(); ++f) {
        file << 'f';
        for (auto v : mesh.faces[f]) {
            file << ' ' << (v + 1) << '/';
            if (textured) file << uv++;
            file << '/' << f + 1;
        }
        file << '\n';
    }
    if (!file) Fail("failed writing " + filename.string());
}
} // namespace blender_stonefish_cpp
