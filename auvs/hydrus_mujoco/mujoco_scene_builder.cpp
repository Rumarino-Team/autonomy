#include "mujoco_scene_builder.hpp"

#include "../blender_scene_cpp/blend_scene.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace mujoco_scene {
namespace {

using blender_scene_cpp::Mesh;
using blender_scene_cpp::Object;
using blender_scene_cpp::Placement;
using blender_scene_cpp::Scene;
using blender_scene_cpp::Vec3;

void Fail(std::string& error, const std::string& message) { error = message; }

std::vector<int> Triangulate(const Mesh& mesh) {
    std::vector<int> triangles;
    for (const auto& face : mesh.faces) {
        if (face.size() < 3) continue;
        for (size_t offset = 1; offset + 1 < face.size(); ++offset) {
            triangles.push_back(static_cast<int>(face[0]));
            triangles.push_back(static_cast<int>(face[offset]));
            triangles.push_back(static_cast<int>(face[offset + 1]));
        }
    }
    return triangles;
}

bool FillMeshAsset(mjsMesh* mesh, const Mesh& source, std::string& error) {
    if (mesh == nullptr) {
        Fail(error, "missing mesh asset");
        return false;
    }
    const std::vector<int> triangles = Triangulate(source);
    if (triangles.empty()) {
        Fail(error, "mesh has no triangles");
        return false;
    }
    std::vector<float> vertices;
    vertices.reserve(source.vertices.size() * 3);
    for (const auto& vertex : source.vertices) {
        vertices.push_back(static_cast<float>(vertex[0]));
        vertices.push_back(static_cast<float>(vertex[1]));
        vertices.push_back(static_cast<float>(vertex[2]));
    }
    mjs_setString(mesh->file, "");
    mjs_setFloat(mesh->uservert, vertices.data(), static_cast<int>(vertices.size()));
    mjs_setInt(mesh->userface, triangles.data(), static_cast<int>(triangles.size()));
    return true;
}

mjsMesh* FindOrAddMesh(mjSpec* spec, const std::string& name) {
    if (mjsElement* element = mjs_findElement(spec, mjOBJ_MESH, name.c_str())) {
        return mjs_asMesh(element);
    }
    mjsMesh* mesh = mjs_addMesh(spec, nullptr);
    mjs_setName(mesh->element, name.c_str());
    return mesh;
}

void SetQuat(double quat[4], const double source[4]) {
    for (int i = 0; i < 4; ++i) quat[i] = source[i];
}

void SetPos(double pos[3], const Vec3& source) {
    pos[0] = source[0];
    pos[1] = source[1];
    pos[2] = source[2];
}

void PoolBounds(const Scene& scene, Vec3& minimum, Vec3& maximum, bool& found) {
    minimum = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity()};
    maximum = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
               -std::numeric_limits<double>::infinity()};
    found = false;
    for (const Object& object : scene.objects) {
        if (object.placement != Placement::world_static) continue;
        if (object.name != "PoolShell" && object.blender_name != "pool_tile") continue;
        for (const auto& vertex : object.mesh->vertices) {
            Vec3 world{
                vertex[0] + object.position[0],
                vertex[1] + object.position[1],
                vertex[2] + object.position[2],
            };
            for (int axis = 0; axis < 3; ++axis) {
                minimum[axis] = std::min(minimum[axis], world[axis]);
                maximum[axis] = std::max(maximum[axis], world[axis]);
                found = true;
            }
        }
    }
    if (!found) {
        for (const Object& object : scene.objects) {
            if (object.placement != Placement::world_static) continue;
            for (const auto& vertex : object.mesh->vertices) {
                Vec3 world{
                    vertex[0] + object.position[0],
                    vertex[1] + object.position[1],
                    vertex[2] + object.position[2],
                };
                for (int axis = 0; axis < 3; ++axis) {
                    minimum[axis] = std::min(minimum[axis], world[axis]);
                    maximum[axis] = std::max(maximum[axis], world[axis]);
                    found = true;
                }
            }
        }
    }
}

void AddPoolCollision(mjSpec* spec, const Vec3& minimum, const Vec3& maximum) {
    mjsBody* world = mjs_findBody(spec, "world");
    if (world == nullptr) return;
    const Vec3 center{
        0.5 * (minimum[0] + maximum[0]),
        0.5 * (minimum[1] + maximum[1]),
        0.5 * (minimum[2] + maximum[2]),
    };
    const Vec3 half{
        0.5 * (maximum[0] - minimum[0]),
        0.5 * (maximum[1] - minimum[1]),
        0.5 * (maximum[2] - minimum[2]),
    };
    const double bottom = maximum[2];
    const double thickness = 0.3;

    auto add_box = [&](const char* name, const Vec3& pos, const Vec3& size) {
        mjsGeom* geom = mjs_addGeom(world, nullptr);
        mjs_setName(geom->element, name);
        geom->type = mjGEOM_BOX;
        SetPos(geom->pos, pos);
        geom->size[0] = size[0];
        geom->size[1] = size[1];
        geom->size[2] = size[2];
        geom->rgba[0] = geom->rgba[1] = geom->rgba[2] = 0.55f;
        geom->rgba[3] = 1.f;
    };

    mjsGeom* floor = mjs_addGeom(world, nullptr);
    mjs_setName(floor->element, "pool_floor");
    floor->type = mjGEOM_PLANE;
    SetPos(floor->pos, {center[0], center[1], bottom});
    floor->quat[0] = 0;
    floor->quat[1] = 1;
    floor->quat[2] = 0;
    floor->quat[3] = 0;
    floor->size[0] = half[0];
    floor->size[1] = half[1];
    floor->size[2] = 0.1;
    floor->rgba[0] = floor->rgba[1] = floor->rgba[2] = 0.45f;
    floor->rgba[3] = 1.f;

    add_box("pool_wall_nx", {minimum[0] - thickness * 0.5, center[1], bottom * 0.5},
            {thickness * 0.5, half[1], bottom * 0.5});
    add_box("pool_wall_px", {maximum[0] + thickness * 0.5, center[1], bottom * 0.5},
            {thickness * 0.5, half[1], bottom * 0.5});
    add_box("pool_wall_ny", {center[0], minimum[1] - thickness * 0.5, bottom * 0.5},
            {half[0], thickness * 0.5, bottom * 0.5});
    add_box("pool_wall_py", {center[0], maximum[1] + thickness * 0.5, bottom * 0.5},
            {half[0], thickness * 0.5, bottom * 0.5});
}

void EnsureMeshPlaceholders(mjSpec* spec) {
    for (mjsElement* element = mjs_firstElement(spec, mjOBJ_MESH); element != nullptr;
         element = mjs_nextElement(spec, element)) {
        mjsMesh* mesh = mjs_asMesh(element);
        if (mesh == nullptr || mesh->uservert == nullptr || !mesh->uservert->empty()) continue;
        const float vertices[] = {
            0, 0, 0, 0.01f, 0, 0, 0, 0.01f, 0, 0, 0, 0.01f,
        };
        const int faces[] = {0, 1, 2, 0, 2, 3};
        mjs_setFloat(mesh->uservert, vertices, 12);
        mjs_setInt(mesh->userface, faces, 6);
        mjs_setString(mesh->file, "");
    }
}

std::string FuzzData(const Object& object) {
    std::string data = object.fuzz;
    if (object.cls == "cube" || object.cls == "rect" || object.cls == "gate") {
        if (!data.empty()) data.push_back(' ');
        data += object.cls;
    }
    return data;
}

}  // namespace

void ApplyBlendToSpec(mjSpec* spec, const Scene& scene, std::string& error) {
    if (spec == nullptr) {
        Fail(error, "null mjSpec");
        return;
    }
    mjsBody* world = mjs_findBody(spec, "world");
    if (world == nullptr) {
        Fail(error, "mjSpec missing world body");
        return;
    }

    bool has_pool = false;
    Vec3 minimum{}, maximum{};
    PoolBounds(scene, minimum, maximum, has_pool);
    if (has_pool) AddPoolCollision(spec, minimum, maximum);

    for (const Object& object : scene.objects) {
        if (object.mesh == nullptr) continue;
        const std::string mesh_name =
            object.placement == Placement::mesh_asset ? object.name : object.name + "_mesh";
        mjsMesh* mesh = FindOrAddMesh(spec, mesh_name);
        if (!FillMeshAsset(mesh, *object.mesh, error)) return;

        if (object.placement == Placement::mesh_asset) {
            if (mjsElement* geom_element = mjs_findElement(spec, mjOBJ_GEOM, object.name.c_str())) {
                mjsGeom* geom = mjs_asGeom(geom_element);
                if (geom != nullptr) {
                    geom->type = mjGEOM_MESH;
                    mjs_setString(geom->meshname, mesh_name.c_str());
                }
            }
            continue;
        }

        if (object.placement == Placement::mocap_fuzz) {
            mjsBody* body = mjs_findBody(spec, object.name.c_str());
            if (body == nullptr) {
                body = mjs_addBody(world, nullptr);
                mjs_setName(body->element, object.name.c_str());
            }
            body->mocap = 1;
            SetPos(body->pos, object.position);
            SetQuat(body->quat, object.quat);

            mjsGeom* geom = nullptr;
            if (mjsElement* geom_element = mjs_findElement(spec, mjOBJ_GEOM, object.name.c_str())) {
                geom = mjs_asGeom(geom_element);
            } else {
                geom = mjs_addGeom(body, nullptr);
                mjs_setName(geom->element, object.name.c_str());
            }
            geom->type = mjGEOM_MESH;
            mjs_setString(geom->meshname, mesh_name.c_str());
            geom->contype = 0;
            geom->conaffinity = 0;
            geom->rgba[0] = geom->rgba[1] = geom->rgba[2] = 0.75f;
            geom->rgba[3] = 1.f;

            const std::string text_name = "fuzz:" + object.name;
            mjsText* text = nullptr;
            if (mjsElement* text_element = mjs_findElement(spec, mjOBJ_TEXT, text_name.c_str())) {
                text = mjs_asText(text_element);
            } else {
                text = mjs_addText(spec);
                mjs_setName(text->element, text_name.c_str());
            }
            mjs_setString(text->data, FuzzData(object).c_str());
            continue;
        }

        mjsGeom* geom = mjs_addGeom(world, nullptr);
        mjs_setName(geom->element, object.name.c_str());
        geom->type = mjGEOM_MESH;
        mjs_setString(geom->meshname, mesh_name.c_str());
        SetPos(geom->pos, object.position);
        SetQuat(geom->quat, object.quat);
        geom->contype = object.convex ? 1 : 0;
        geom->conaffinity = object.convex ? 1 : 0;
        geom->rgba[0] = geom->rgba[1] = geom->rgba[2] = 0.75f;
        geom->rgba[3] = 1.f;
    }
}

mjModel* ComposeModel(
    const std::filesystem::path& skeleton_xml,
    const std::filesystem::path& blend_file,
    const std::filesystem::path& stonefish_config,
    std::string& error) {
    char buffer[1024] = {};
    mjSpec* spec = mj_parseXML(skeleton_xml.string().c_str(), nullptr, buffer, sizeof(buffer));
    if (spec == nullptr) {
        error = buffer[0] != '\0' ? buffer : "mj_parseXML failed";
        return nullptr;
    }

    try {
        const Scene scene = blender_scene_cpp::ReadScene(blend_file, stonefish_config);
        ApplyBlendToSpec(spec, scene, error);
        if (!error.empty()) {
            mj_deleteSpec(spec);
            return nullptr;
        }
        EnsureMeshPlaceholders(spec);
    } catch (const std::exception& exception) {
        error = exception.what();
        mj_deleteSpec(spec);
        return nullptr;
    }

    mjModel* model = mj_compile(spec, nullptr);
    if (model == nullptr) {
        const char* spec_error = mjs_getError(spec);
        error = spec_error != nullptr && spec_error[0] != '\0' ? spec_error : "mj_compile failed";
        mj_deleteSpec(spec);
        return nullptr;
    }
    mj_deleteSpec(spec);
    return model;
}

}  // namespace mujoco_scene
