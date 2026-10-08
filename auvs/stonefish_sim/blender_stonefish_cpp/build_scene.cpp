#include "build_scene.hpp"
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/MaterialManager.h>
#include <Stonefish/core/NED.h>
#include <Stonefish/entities/statics/Obstacle.h>
#include <Stonefish/entities/forcefields/Ocean.h>
#include <Stonefish/entities/forcefields/Atmosphere.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/graphics/OpenGLContent.h>
#include <Stonefish/utils/GeometryFileUtil.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace blender_stonefish_cpp {
namespace {
struct Point2 { double x, y; };
double Orient(Point2 a, Point2 b, Point2 c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
bool Contains(Point2 p, Point2 a, Point2 b, Point2 c) {
    constexpr double eps = 1e-12;
    return Orient(a, b, p) >= -eps && Orient(b, c, p) >= -eps && Orient(c, a, p) >= -eps;
}
Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double Dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec3 Normalized(Vec3 v) {
    double length = std::hypot(v[0], v[1], v[2]);
    if (length < 1e-12) return {0, 0, 1};
    for (double& component : v) component /= length;
    return v;
}
glm::vec3 FaceNormal(const Mesh& mesh, const std::vector<uint32_t>& face) {
    const auto& a = mesh.vertices[face[0]];
    const auto& b = mesh.vertices[face[1]];
    const auto& c = mesh.vertices[face[2]];
    Vec3 normal = Normalized(Cross({b[0] - a[0], b[1] - a[1], b[2] - a[2]}, {c[0] - a[0], c[1] - a[1], c[2] - a[2]}));
    return {float(normal[0]), float(normal[1]), float(normal[2])};
}
// Quads use the shorter diagonal, matching rapidobj::Triangulate in Stonefish's OBJ loader.
std::vector<std::array<int, 3>> Triangulate(const Mesh& mesh, const std::vector<uint32_t>& face) {
    const int count = static_cast<int>(face.size());
    std::vector<std::array<int, 3>> tris;
    if (count == 3) {
        tris.push_back({0, 1, 2});
        return tris;
    }
    if (count == 4) {
        auto dist2 = [&](int i, int j) {
            const auto& p = mesh.vertices[face[i]];
            const auto& q = mesh.vertices[face[j]];
            double dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
            return dx * dx + dy * dy + dz * dz;
        };
        if (dist2(0, 2) < dist2(1, 3)) {
            tris.push_back({0, 1, 2});
            tris.push_back({0, 2, 3});
        } else {
            tris.push_back({0, 1, 3});
            tris.push_back({1, 2, 3});
        }
        return tris;
    }
    const auto& origin = mesh.vertices[face[0]];
    Vec3 normal = Normalized(Cross(
        {mesh.vertices[face[1]][0] - origin[0], mesh.vertices[face[1]][1] - origin[1], mesh.vertices[face[1]][2] - origin[2]},
        {mesh.vertices[face[2]][0] - origin[0], mesh.vertices[face[2]][1] - origin[1], mesh.vertices[face[2]][2] - origin[2]}));
    Vec3 u = Normalized(Cross(normal, std::abs(normal[0]) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0}));
    Vec3 v = Cross(normal, u);
    std::vector<Point2> polygon(count);
    for (int i = 0; i < count; ++i) {
        const auto& p = mesh.vertices[face[i]];
        polygon[i] = {Dot({p[0], p[1], p[2]}, u), Dot({p[0], p[1], p[2]}, v)};
    }
    std::vector<int> ring(count);
    for (int i = 0; i < count; ++i) ring[i] = i;
    while (ring.size() > 3) {
        bool clipped = false;
        for (size_t i = 0; i < ring.size(); ++i) {
            int prev = ring[(i + ring.size() - 1) % ring.size()], curr = ring[i], next = ring[(i + 1) % ring.size()];
            if (Orient(polygon[prev], polygon[curr], polygon[next]) <= 1e-12) continue;
            bool blocked = false;
            for (int other : ring) {
                if (other == prev || other == curr || other == next) continue;
                if (Contains(polygon[other], polygon[prev], polygon[curr], polygon[next])) { blocked = true; break; }
            }
            if (blocked) continue;
            tris.push_back({prev, curr, next});
            ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            for (size_t i = 1; i + 1 < ring.size(); ++i) tris.push_back({ring[0], ring[i], ring[i + 1]});
            return tris;
        }
    }
    tris.push_back({ring[0], ring[1], ring[2]});
    return tris;
}
bool Textured(const Mesh& mesh) {
    if (mesh.face_uvs.size() != mesh.faces.size()) return false;
    for (size_t i = 0; i < mesh.faces.size(); ++i)
        if (mesh.face_uvs[i].size() != mesh.faces[i].size()) return false;
    return true;
}
sf::Mesh* CloneStonefishMesh(const sf::Mesh* source) {
    if (source->isTexturable()) {
        const auto* textured = static_cast<const sf::TexturableMesh*>(source);
        auto* copy = new sf::TexturableMesh;
        copy->vertices = textured->vertices;
        copy->faces = textured->faces;
        return copy;
    }
    const auto* plain = static_cast<const sf::PlainMesh*>(source);
    auto* copy = new sf::PlainMesh;
    copy->vertices = plain->vertices;
    copy->faces = plain->faces;
    return copy;
}
sf::Mesh* StonefishMesh(const Mesh& mesh) {
    const size_t positions = mesh.vertices.size();
    if (Textured(mesh)) {
        auto* out = new sf::TexturableMesh;
        out->vertices.resize(positions);
        for (size_t i = 0; i < positions; ++i)
            out->vertices[i].pos = glm::vec3(mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]);
        std::unordered_set<sf::IndexedTexturableVertex, sf::IndexedTexturableVertexHash> generated;
        GLuint extra = 0;
        for (size_t f = 0; f < mesh.faces.size(); ++f) {
            glm::vec3 normal = FaceNormal(mesh, mesh.faces[f]);
            for (auto corner : Triangulate(mesh, mesh.faces[f])) {
                sf::Face face;
                for (int k = 0; k < 3; ++k) {
                    auto& slot = out->vertices[mesh.faces[f][corner[k]]];
                    glm::vec2 uv(mesh.face_uvs[f][corner[k]][0], mesh.face_uvs[f][corner[k]][1]);
                    if (glm::length2(slot.normal) == 0.f) {
                        slot.normal = normal;
                        slot.uv = uv;
                        face.vertexID[k] = mesh.faces[f][corner[k]];
                    } else if (slot.normal.x == normal.x && slot.normal.y == normal.y && slot.normal.z == normal.z
                               && slot.uv.x == uv.x && slot.uv.y == uv.y) {
                        face.vertexID[k] = mesh.faces[f][corner[k]];
                    } else {
                        sf::TexturableVertex vertex = slot;
                        vertex.normal = normal;
                        vertex.uv = uv;
                        vertex.tangent = glm::vec3(0.f);
                        auto found = generated.find({0, vertex});
                        if (found != generated.end())
                            face.vertexID[k] = found->index + static_cast<GLuint>(positions);
                        else {
                            generated.insert({extra, vertex});
                            face.vertexID[k] = extra + static_cast<GLuint>(positions);
                            ++extra;
                        }
                    }
                }
                out->faces.push_back(face);
            }
        }
        std::vector<sf::IndexedTexturableVertex> sorted(generated.begin(), generated.end());
        std::sort(sorted.begin(), sorted.end());
        out->vertices.resize(positions + sorted.size());
        for (size_t i = 0; i < sorted.size(); ++i) out->vertices[positions + i] = sorted[i].vertex;
        sf::OpenGLContent::CheckAndRepairFaceVertexOrder(out);
        sf::OpenGLContent::ComputeTangents(out);
        return out;
    }
    auto* out = new sf::PlainMesh;
    out->vertices.resize(positions);
    for (size_t i = 0; i < positions; ++i)
        out->vertices[i].pos = glm::vec3(mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]);
    std::unordered_set<sf::IndexedVertex, sf::IndexedVertexHash> generated;
    GLuint extra = 0;
    for (const auto& polygon : mesh.faces) {
        glm::vec3 normal = FaceNormal(mesh, polygon);
        for (auto corner : Triangulate(mesh, polygon)) {
            sf::Face face;
            for (int k = 0; k < 3; ++k) {
                auto& slot = out->vertices[polygon[corner[k]]];
                if (glm::length2(slot.normal) == 0.f) {
                    slot.normal = normal;
                    face.vertexID[k] = polygon[corner[k]];
                } else if (slot.normal.x == normal.x && slot.normal.y == normal.y && slot.normal.z == normal.z) {
                    face.vertexID[k] = polygon[corner[k]];
                } else {
                    sf::Vertex vertex = slot;
                    vertex.normal = normal;
                    auto found = generated.find({0, vertex});
                    if (found != generated.end())
                        face.vertexID[k] = found->index + static_cast<GLuint>(positions);
                    else {
                        generated.insert({extra, vertex});
                        face.vertexID[k] = extra + static_cast<GLuint>(positions);
                        ++extra;
                    }
                }
            }
            out->faces.push_back(face);
        }
    }
    std::vector<sf::IndexedVertex> sorted(generated.begin(), generated.end());
    std::sort(sorted.begin(), sorted.end());
    out->vertices.resize(positions + sorted.size());
    for (size_t i = 0; i < sorted.size(); ++i) out->vertices[positions + i] = sorted[i].vertex;
    sf::OpenGLContent::CheckAndRepairFaceVertexOrder(out);
    return out;
}
// Converted meshes survive scenario restarts. A full restart still uploads new
// GPU buffers; this skips the triangulation work for geometry that did not change.
std::unordered_map<std::string, std::unique_ptr<sf::Mesh>>& StonefishMeshCache() {
    static std::unordered_map<std::string, std::unique_ptr<sf::Mesh>> cache;
    return cache;
}
const sf::Mesh* CachedStonefishMesh(const Mesh& mesh) {
    auto& cache = StonefishMeshCache();
    const std::string key = MeshFingerprint(mesh);
    auto it = cache.find(key);
    if (it == cache.end())
        it = cache.emplace(key, std::unique_ptr<sf::Mesh>(StonefishMesh(mesh))).first;
    return it->second.get();
}
} // namespace
std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const Scene& scene,
    const std::filesystem::path& config_path, const std::filesystem::path& data,
    const std::filesystem::path& blend) {
    auto config = YAML::LoadFile(config_path.string());
    for (const auto& object : scene.objects) {
        if (!config["materials"][object.material] || !config["looks"][object.look])
            throw std::runtime_error("[blender_cpp] " + object.name + ": unknown material or look");
    }
    auto path = [&](const YAML::Node& value) -> std::string {
        if (!value || value.as<std::string>().empty()) return {};
        return (data / value.as<std::string>()).string();
    };
    std::unordered_map<std::string, std::string> materials, looks, classes;
    for (const auto& entry : config["materials"]) {
        auto name = entry.first.as<std::string>();
        auto props = entry.second;
        materials[name] = manager.getMaterialManager()->CreateMaterial(
            name, props["density"].as<double>(), props["restitution"].as<double>(), props["magnetic"].as<double>(0));
    }
    for (const auto& entry : config["looks"]) {
        auto name = entry.first.as<std::string>();
        auto props = entry.second;
        auto color = props["color"];
        looks[name] = manager.CreateLook(name,
            sf::Color(color[0].as<float>(), color[1].as<float>(), color[2].as<float>()),
            props["roughness"].as<float>(0.5f), props["metalness"].as<float>(0), props["reflectivity"].as<float>(0.5f),
            path(props["texture"]), path(props["normal_map"]));
    }
    for (const auto& pair : config["friction"]["pairs"]) {
        auto a = pair[0].as<std::string>(), b = pair[1].as<std::string>();
        if (!materials.contains(a) || !materials.contains(b)
            || !manager.SetMaterialsInteraction(materials.at(a), materials.at(b), pair[2].as<double>(), pair[3].as<double>()))
            throw std::runtime_error("[blender_cpp] invalid friction pair: " + a + "/" + b);
    }
    const auto env = config["environment"], ocean = env["ocean"], sun = env["sun"], ned = env["ned"];
    manager.getNED()->Init(ned["latitude"].as<double>(), ned["longitude"].as<double>(), 0);
    if (ocean["enabled"].as<bool>(true)) {
        auto water = manager.getMaterialManager()->CreateFluid("Water", ocean["water_density"].as<double>(), 1.308e-3, 1.55);
        manager.EnableOcean(ocean["wave_height"].as<double>(), manager.getMaterialManager()->getFluid(water));
        auto* medium = manager.getOcean();
        medium->setWaterType(ocean["jerlov"].as<double>());
        medium->SetConditions(env["atmosphere"]["temperature"].as<double>());
        medium->setParticles(ocean["particles"].as<bool>(true));
        auto current = ocean["current"]["velocity"];
        medium->AddVelocityField(new sf::Uniform(sf::Vector3(current[0].as<double>(), current[1].as<double>(), current[2].as<double>())));
    }
    manager.EnableAtmosphere();
    manager.getAtmosphere()->SetSunPosition(sun["azimuth"].as<double>(), sun["elevation"].as<double>());
    std::map<const Mesh*, const sf::Mesh*> meshes;
    size_t reused = 0;
    for (const auto& object : scene.objects) {
        auto it = meshes.find(object.mesh.get());
        if (it == meshes.end()) {
            const std::string key = MeshFingerprint(*object.mesh);
            const bool hit = StonefishMeshCache().contains(key);
            it = meshes.emplace(object.mesh.get(), CachedStonefishMesh(*object.mesh)).first;
            if (hit) ++reused;
        }
        auto* obstacle = new sf::Obstacle(object.name, CloneStonefishMesh(it->second),
                                          object.convex, materials.at(object.material), looks.at(object.look));
        const auto& p = object.position;
        const auto& r = object.rotation;
        manager.AddStaticEntity(obstacle, sf::Transform(sf::Quaternion(r[2], r[1], r[0]), sf::Vector3(p[0], p[1], p[2])));
        if (!object.cls.empty() && object.cls != "scenery") classes.emplace(obstacle->getName(), object.cls);
    }
    std::cout << "[blender_cpp] built " << scene.objects.size() << " static entities, " << meshes.size()
              << " unique meshes (" << reused << " from cache), " << classes.size()
              << " tracked objects from " << blend << '\n';
    return classes;
}
std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const std::filesystem::path& blend,
    const std::filesystem::path& config_path, const std::filesystem::path& data) {
    return BuildScene(manager, ReadScene(blend, config_path), config_path, data, blend);
}
}
