#include "build_scene.hpp"
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/MaterialManager.h>
#include <Stonefish/core/NED.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/graphics/OpenGLPipeline.h>
#include <Stonefish/entities/StaticEntity.h>
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
#include <set>
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

sf::Transform Pose(const Object& object) {
    return sf::Transform(sf::Quaternion(object.rotation[2], object.rotation[1], object.rotation[0]),
                         sf::Vector3(object.position[0], object.position[1], object.position[2]));
}
std::string Identity(const Object& object) { return object.id.empty() ? object.name : object.id; }
sf::OpenGLContent* Content() {
    auto* app = sf::SimulationApp::getApp();
    return app->hasGraphics() ? static_cast<sf::GraphicalSimulationApp*>(app)->getGLPipeline()->getContent() : nullptr;
}
struct DetachedObstacleDeleter {
    void operator()(sf::Obstacle* obstacle) const {
        if(obstacle != nullptr) { obstacle->ReleasePhysics(); delete obstacle; }
    }
};
using DetachedObstacle = std::unique_ptr<sf::Obstacle, DetachedObstacleDeleter>;
struct PreparedLook {
    sf::Look value;
    ~PreparedLook() {
        if(value.albedoTexture) glDeleteTextures(1, &value.albedoTexture);
        if(value.normalMap) glDeleteTextures(1, &value.normalMap);
    }
};
void ApplyEnvironment(sf::SimulationManager& manager, const EnvironmentSettings& env) {
    manager.getNED()->Init(env.latitude, env.longitude, 0);
    if(env.enabled) {
        sf::Fluid fluid;
        fluid.name = "Water"; fluid.density = env.density; fluid.viscosity = 1.308e-3; fluid.IOR = 1.55;
        manager.UpdateOcean(env.waves, fluid);
        auto* ocean = manager.getOcean();
        ocean->setWaterType(env.jerlov);
        ocean->SetConditions(env.temperature);
        ocean->setParticles(env.particles);
        auto* current = dynamic_cast<sf::Uniform*>(ocean->getVelocityField(0));
        const sf::Vector3 velocity(env.current[0], env.current[1], env.current[2]);
        if(current) current->setVelocity(velocity);
        else ocean->AddVelocityField(new sf::Uniform(velocity));
        ocean->EnableCurrents();
        manager.getDynamicsWorld()->updateSingleAabb(ocean->getGhost());
    } else manager.DisableOcean();
    manager.getAtmosphere()->SetSunPosition(env.azimuth, env.elevation);
}
} // namespace

SceneSettings ReadSceneSettings(const std::filesystem::path& path, const std::filesystem::path& data) {
    const auto config = YAML::LoadFile(path.string());
    SceneSettings out;
    auto number = [](const YAML::Node& node, const std::string& name, double lo, double hi) {
        const double v = node.as<double>();
        if(!std::isfinite(v) || v < lo || v > hi)
            throw std::runtime_error("[blender_cpp] invalid " + name);
        return v;
    };
    auto texture = [&](const YAML::Node& node) {
        const auto value = node.as<std::string>("");
        if(!value.empty() && !std::filesystem::is_regular_file(data / value))
            throw std::runtime_error("[blender_cpp] texture missing: " + (data / value).string());
        return value;
    };
    if(!config["materials"].IsMap() || !config["looks"].IsMap())
        throw std::runtime_error("[blender_cpp] materials and looks must be maps");
    for(const auto& entry : config["materials"]) {
        const auto name = entry.first.as<std::string>(); const auto v = entry.second;
        if(name.empty()) throw std::runtime_error("[blender_cpp] empty material name");
        out.materials.emplace(name, MaterialSettings{
            number(v["density"], "density", 1e-9, 1e12), number(v["restitution"], "restitution", 0, 1),
            number(v["magnetic"] ? v["magnetic"] : YAML::Node(0), "magnetic", -1e12, 1e12)});
    }
    for(const auto& entry : config["looks"]) {
        const auto name = entry.first.as<std::string>(); const auto v = entry.second;
        if(name.empty() || name == "__Default__") throw std::runtime_error("[blender_cpp] reserved/empty look name");
        if(!v["color"].IsSequence() || v["color"].size() != 3)
            throw std::runtime_error("[blender_cpp] color must have three components");
        LookSettings look;
        for(size_t i = 0; i < 3; ++i) look.color[i] = number(v["color"][i], "color", 0, 1);
        look.roughness = number(v["roughness"] ? v["roughness"] : YAML::Node(0.5), "roughness", 0, 1);
        look.metalness = number(v["metalness"] ? v["metalness"] : YAML::Node(0), "metalness", 0, 1);
        look.reflectivity = number(v["reflectivity"] ? v["reflectivity"] : YAML::Node(0.5), "reflectivity", 0, 1);
        look.texture = texture(v["texture"]); look.normal_map = texture(v["normal_map"]);
        out.looks.emplace(name, std::move(look));
    }
    if(out.materials.empty() || out.looks.empty()) throw std::runtime_error("[blender_cpp] empty materials/looks");
    std::set<std::pair<std::string, std::string>> pairs;
    for(const auto& pair : config["friction"]["pairs"]) {
        if(!pair.IsSequence() || pair.size() != 4) throw std::runtime_error("[blender_cpp] invalid friction pair");
        auto a = pair[0].as<std::string>(), b = pair[1].as<std::string>();
        if(!out.materials.contains(a) || !out.materials.contains(b) || !pairs.emplace(std::min(a,b), std::max(a,b)).second)
            throw std::runtime_error("[blender_cpp] unknown/duplicate friction pair: " + a + "/" + b);
        out.friction.push_back({a, b, number(pair[2], "static friction", 0, 1e6), number(pair[3], "dynamic friction", 0, 1e6)});
    }
    const auto env = config["environment"], ocean = env["ocean"], sun = env["sun"], ned = env["ned"];
    auto& e = out.environment;
    e.enabled = ocean["enabled"].as<bool>(true); e.particles = ocean["particles"].as<bool>(true);
    e.density = number(ocean["water_density"], "water_density", 1e-9, 1e9);
    e.jerlov = number(ocean["jerlov"], "jerlov", 0, 1);
    e.waves = number(ocean["wave_height"], "wave_height", 0, 2);
    e.temperature = number(env["atmosphere"]["temperature"], "temperature", -100, 200);
    e.azimuth = number(sun["azimuth"], "sun azimuth", -1e9, 1e9);
    e.elevation = number(sun["elevation"], "sun elevation", -90, 90);
    e.latitude = number(ned["latitude"], "latitude", -90, 90);
    e.longitude = number(ned["longitude"], "longitude", -180, 180);
    const auto current = ocean["current"]["velocity"];
    if(!current.IsSequence() || current.size() != 3) throw std::runtime_error("[blender_cpp] current velocity must have three components");
    for(size_t i = 0; i < 3; ++i) e.current[i] = number(current[i], "current velocity", -1e6, 1e6);
    return out;
}

void UpdateScene(sf::SimulationManager& manager, const Scene& previous, const Scene& next,
                 const SceneSettings& previous_settings, const SceneSettings& settings,
                 const std::filesystem::path& data, bool reload_textures) {
    std::unordered_map<std::string, const Object*> old;
    std::unordered_map<std::string, sf::StaticEntity*> live;
    std::unordered_set<std::string> old_names, next_ids, next_names;
    for(const auto& object : previous.objects) {
        auto* entity = dynamic_cast<sf::StaticEntity*>(manager.getEntity(object.name));
        if(!entity || !old.emplace(Identity(object), &object).second)
            throw std::runtime_error("[blender_cpp] missing/duplicate live static: " + object.name);
        live.emplace(Identity(object), entity); old_names.insert(object.name);
    }
    for(const auto& object : next.objects) {
        if(object.name.empty() || object.name.starts_with("__hot_reload_"))
            throw std::runtime_error("[blender_cpp] empty/reserved entity name: " + object.name);
        if(!object.mesh || !next_ids.insert(Identity(object)).second || !next_names.insert(object.name).second)
            throw std::runtime_error("[blender_cpp] missing mesh or duplicate identity/name");
        if(!settings.materials.contains(object.material) || !settings.looks.contains(object.look))
            throw std::runtime_error("[blender_cpp] unknown material/look for " + object.name);
        if(manager.getNameManager()->HasName(object.name) && !old_names.contains(object.name))
            throw std::runtime_error("[blender_cpp] name conflicts with a robot/sensor/entity: " + object.name);
    }

    // Stage every fallible decode/upload before detaching or moving live bodies.
    std::vector<std::unique_ptr<PreparedLook>> prepared_looks;
    auto* content = Content();
    if(content) for(const auto& [name, spec] : settings.looks) {
        const auto prior = previous_settings.looks.find(name);
        if(!reload_textures && prior != previous_settings.looks.end() && prior->second == spec) continue;
        auto prepared = std::make_unique<PreparedLook>(); auto& look = prepared->value;
        look.name = name; look.type = sf::LookType::PHYSICAL;
        look.color = sf::Color(spec.color[0], spec.color[1], spec.color[2]);
        look.reflectivity = spec.reflectivity; look.params = {float(spec.roughness), float(spec.metalness)};
        if(!spec.texture.empty()) {
            look.albedoTexture = content->LoadTexture((data / spec.texture).string(), true);
            if(!look.albedoTexture) throw std::runtime_error("[blender_cpp] cannot decode texture: " + spec.texture);
        }
        if(!spec.normal_map.empty()) {
            look.normalMap = content->LoadTexture((data / spec.normal_map).string(), false);
            if(!look.normalMap) throw std::runtime_error("[blender_cpp] cannot decode normal map: " + spec.normal_map);
        }
        prepared_looks.push_back(std::move(prepared));
    }
    std::unordered_map<std::string, DetachedObstacle> replacements;
    const auto placeholder_material = manager.getMaterialManager()->GetMaterialsList().front();
    for(const auto& object : next.objects) {
        auto prior = old.find(Identity(object));
        if(prior != old.end() && prior->second->convex == object.convex
           && MeshFingerprint(*prior->second->mesh) == MeshFingerprint(*object.mesh)) continue;
        replacements.emplace(Identity(object), DetachedObstacle(new sf::Obstacle(
            "__hot_reload_staged__", CloneStonefishMesh(CachedStonefishMesh(*object.mesh)), object.convex,
            placeholder_material, "")));
    }

    auto* materials = manager.getMaterialManager();
    for(const auto& [name, spec] : settings.materials) {
        sf::Material value{name, spec.density, spec.restitution, spec.magnetic};
        if(!materials->UpdateMaterial(value)) materials->CreateMaterial(name, spec.density, spec.restitution, spec.magnetic);
    }
    for(const auto& pair : previous_settings.friction)
        materials->SetMaterialsInteraction(pair.a, pair.b, 1, 1);
    for(const auto& pair : settings.friction)
        materials->SetMaterialsInteraction(pair.a, pair.b, pair.stat, pair.dynamic);
    if(content) for(auto& prepared : prepared_looks) {
        content->SetLook(std::move(prepared->value));
        prepared->value.albedoTexture = prepared->value.normalMap = 0; // ownership transferred
    }
    // Vacate the old names first, allowing swaps/renames in one Blender save.
    for(auto& [id, entity] : live)
        if(!next_ids.contains(id) || replacements.contains(id)) {
            manager.RemoveStaticEntity(entity); entity->ReleasePhysics(); delete entity; entity = nullptr;
        } else entity->Rename("__hot_reload_live__");
    size_t added = 0, replaced = 0, removed = 0, moved = 0;
    for(const auto& [id, prior] : old) if(!next_ids.contains(id)) ++removed;
    for(const auto& object : next.objects) {
        const auto id = Identity(object);
        auto replacement = replacements.find(id);
        sf::StaticEntity* entity = nullptr;
        if(replacement != replacements.end()) {
            entity = replacement->second.get(); entity->Rename(object.name);
            manager.AddStaticEntity(entity, Pose(object)); replacement->second.release();
            old.contains(id) ? ++replaced : ++added;
        } else {
            entity = live.at(id); entity->Rename(object.name);
            const auto* prior = old.at(id);
            if(prior->position != object.position || prior->rotation != object.rotation) {
                entity->setTransform(Pose(object));
                manager.getDynamicsWorld()->updateSingleAabb(entity->getRigidBody());
                ++moved;
            }
        }
        entity->setSurface(materials->getMaterial(object.material), content ? content->getLookId(object.look) : -1);
    }
    if(previous_settings.environment != settings.environment) ApplyEnvironment(manager, settings.environment);
    if(content) {
        // A frame may already be queued when a save arrives. Replace it before
        // the renderer can read deleted/reused object IDs.
        auto* pipeline = static_cast<sf::GraphicalSimulationApp*>(sf::SimulationApp::getApp())->getGLPipeline();
        SDL_LockMutex(pipeline->getDrawingQueueMutex());
        pipeline->PurgeDrawingQueue();
        pipeline->PurgeSelectedDrawingQueue();
        manager.UpdateDrawingQueue();
        SDL_UnlockMutex(pipeline->getDrawingQueueMutex());
    }
    // Drop converted meshes no longer in the scene to bound memory during editing.
    std::unordered_set<std::string> used;
    for(const auto& object : next.objects) used.insert(MeshFingerprint(*object.mesh));
    auto& cache = StonefishMeshCache();
    for(auto it = cache.begin(); it != cache.end();) it = used.contains(it->first) ? std::next(it) : cache.erase(it);
    std::cout << "[blender_cpp] live scene: " << added << " added, " << replaced << " replaced, " << removed
              << " removed, " << moved << " moved, " << prepared_looks.size() << " looks updated; clock="
              << manager.getSimulationTime() << " (world/robot preserved)\n";
}

std::unordered_map<std::string, std::string> BuildScene(
    sf::SimulationManager& manager, const Scene& scene,
    const std::filesystem::path& config_path, const std::filesystem::path& data,
    const std::filesystem::path& blend) {
    const auto validated = ReadSceneSettings(config_path, data);
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
        medium->EnableCurrents();
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
