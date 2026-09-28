#include "hydrus_camera.h"

#include "hydrus_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace hydrus {

namespace {

constexpr int kW = HYDRUS_CAM_W;
constexpr int kH = HYDRUS_CAM_H;
constexpr int kCell = kW / HYDRUS_CAM_GRID_X;
constexpr uint8_t kShadeFloor = 70;
constexpr uint8_t kShadeGate = 180;
constexpr uint8_t kShadeMarker = 240;
constexpr int kClsCube = 0;
constexpr int kClsGate = 2;

struct ScreenPoint {
    float u = 0;
    float v = 0;
    float z = 0;
    bool ok = false;
};

struct CamPoint {
    float x = 0;
    float y = 0;
    float z = 0;
};

// Same camera as auv.cpp projectAabb: site +X right, +Y up, +Z forward, then Y is flipped
// so image +Y is down.
ScreenPoint project(const mjtNum* cam_pos, const mjtNum* cam_mat, const float point[3], double focal) {
    const mjtNum world[3] = {point[0] - cam_pos[0], point[1] - cam_pos[1], point[2] - cam_pos[2]};
    mjtNum cam[3];
    mju_mulMatTVec3(cam, cam_mat, world);
    cam[1] = -cam[1];
    ScreenPoint out;
    out.z = static_cast<float>(cam[2]);
    if (out.z <= 0.1f || out.z >= 50.f)
        return out;
    out.u = static_cast<float>(focal * cam[0] / cam[2] + kW * 0.5);
    out.v = static_cast<float>(focal * cam[1] / cam[2] + kH * 0.5);
    out.ok = true;
    return out;
}

CamPoint toCamera(const mjtNum* cam_pos, const mjtNum* cam_mat, const float point[3]) {
    const mjtNum world[3] = {point[0] - cam_pos[0], point[1] - cam_pos[1], point[2] - cam_pos[2]};
    mjtNum cam[3];
    mju_mulMatTVec3(cam, cam_mat, world);
    return {static_cast<float>(cam[0]), static_cast<float>(cam[1]), static_cast<float>(cam[2])};
}

ScreenPoint projectCam(const CamPoint& cam, double focal) {
    ScreenPoint out;
    out.z = cam.z;
    if (out.z <= 0.1f)
        return out;
    out.u = static_cast<float>(focal * cam.x / cam.z + kW * 0.5);
    out.v = static_cast<float>(focal * -cam.y / cam.z + kH * 0.5);
    out.ok = true;
    return out;
}

CamPoint lerpCam(const CamPoint& a, const CamPoint& b, float t) {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

// Keeps the part of the polygon in front of the near plane. Output can hold n_in + 1 points.
int clipNear(const CamPoint* in, int n_in, CamPoint* out, float near_z) {
    int n = 0;
    for (int i = 0; i < n_in; ++i) {
        const CamPoint& a = in[i];
        const CamPoint& b = in[(i + 1) % n_in];
        const bool ina = a.z >= near_z;
        const bool inb = b.z >= near_z;
        if (ina && inb) {
            out[n++] = b;
        } else if (ina && !inb) {
            out[n++] = lerpCam(a, b, (near_z - a.z) / (b.z - a.z));
        } else if (!ina && inb) {
            out[n++] = lerpCam(a, b, (near_z - a.z) / (b.z - a.z));
            out[n++] = b;
        }
    }
    return n;
}

void pushTri(std::vector<CameraModel::Tri>& tris, int geom, uint8_t shade, const float a[3], const float b[3], const float c[3]) {
    CameraModel::Tri tri;
    tri.geom = geom;
    tri.shade = shade;
    std::memcpy(tri.v[0], a, sizeof(float) * 3);
    std::memcpy(tri.v[1], b, sizeof(float) * 3);
    std::memcpy(tri.v[2], c, sizeof(float) * 3);
    tris.push_back(tri);
}

void addMesh(std::vector<CameraModel::Tri>& tris, const mjModel* model, int geom, uint8_t shade) {
    const int mesh = model->geom_dataid[geom];
    if (mesh < 0)
        return;
    const int vert_adr = model->mesh_vertadr[mesh];
    const int face_adr = model->mesh_faceadr[mesh];
    const int nface = model->mesh_facenum[mesh];
    for (int i = 0; i < nface; ++i) {
        const int* face = model->mesh_face + 3 * (face_adr + i);
        float v[3][3];
        for (int k = 0; k < 3; ++k) {
            const float* vert = model->mesh_vert + 3 * (vert_adr + face[k]);
            v[k][0] = vert[0];
            v[k][1] = vert[1];
            v[k][2] = vert[2];
        }
        pushTri(tris, geom, shade, v[0], v[1], v[2]);
    }
}

void addCylinder(std::vector<CameraModel::Tri>& tris, int geom, uint8_t shade, float radius, float half_height) {
    constexpr int kSlices = 10;
    constexpr float kPi = 3.14159265f;
    float top[kSlices][3];
    float bottom[kSlices][3];
    for (int i = 0; i < kSlices; ++i) {
        const float a = static_cast<float>(i) / kSlices * 2.f * kPi;
        const float x = std::sin(a) * radius;
        const float y = std::cos(a) * radius;
        top[i][0] = bottom[i][0] = x;
        top[i][1] = bottom[i][1] = y;
        top[i][2] = half_height;
        bottom[i][2] = -half_height;
    }
    const float top_center[3] = {0, 0, half_height};
    const float bottom_center[3] = {0, 0, -half_height};
    for (int i = 0; i < kSlices; ++i) {
        const int n = (i + 1) % kSlices;
        pushTri(tris, geom, shade, top[i], bottom[i], top[n]);
        pushTri(tris, geom, shade, bottom[i], bottom[n], top[n]);
        pushTri(tris, geom, shade, top_center, top[i], top[n]);
        pushTri(tris, geom, shade, bottom_center, bottom[n], bottom[i]);
    }
}

void addPlane(std::vector<CameraModel::Tri>& tris, int geom, uint8_t shade, float hx, float hy) {
    const float a[3] = {-hx, -hy, 0};
    const float b[3] = {hx, -hy, 0};
    const float c[3] = {hx, hy, 0};
    const float d[3] = {-hx, hy, 0};
    pushTri(tris, geom, shade, a, b, c);
    pushTri(tris, geom, shade, a, c, d);
}

void localAabb(const mjModel* model, int geom, float min[3], float max[3]) {
    bool any = false;
    auto add = [&](float x, float y, float z) {
        const float p[3] = {x, y, z};
        if (!any) {
            for (int i = 0; i < 3; ++i)
                min[i] = max[i] = p[i];
            any = true;
            return;
        }
        for (int i = 0; i < 3; ++i) {
            min[i] = std::min(min[i], p[i]);
            max[i] = std::max(max[i], p[i]);
        }
    };
    const mjtNum* size = model->geom_size + 3 * geom;
    if (model->geom_type[geom] == mjGEOM_MESH && model->geom_dataid[geom] >= 0) {
        const int mesh = model->geom_dataid[geom];
        const int vert_adr = model->mesh_vertadr[mesh];
        const int nvert = model->mesh_vertnum[mesh];
        for (int i = 0; i < nvert; ++i) {
            const float* vert = model->mesh_vert + 3 * (vert_adr + i);
            add(vert[0], vert[1], vert[2]);
        }
    } else if (model->geom_type[geom] == mjGEOM_CYLINDER) {
        const float r = static_cast<float>(size[0]);
        const float h = static_cast<float>(size[1]);
        add(-r, -r, -h);
        add(r, r, h);
    } else {
        add(0, 0, 0);
    }
}

void worldPoint(float out[3], const mjtNum* pos, const mjtNum* mat, const float local[3]) {
    out[0] = static_cast<float>(pos[0] + mat[0] * local[0] + mat[1] * local[1] + mat[2] * local[2]);
    out[1] = static_cast<float>(pos[1] + mat[3] * local[0] + mat[4] * local[1] + mat[5] * local[2]);
    out[2] = static_cast<float>(pos[2] + mat[6] * local[0] + mat[7] * local[1] + mat[8] * local[2]);
}

void rasterTriangle(uint8_t* image, float* depth, const ScreenPoint& a, const ScreenPoint& b, const ScreenPoint& c, uint8_t shade) {
    const float min_u = std::min(a.u, std::min(b.u, c.u));
    const float max_u = std::max(a.u, std::max(b.u, c.u));
    const float min_v = std::min(a.v, std::min(b.v, c.v));
    const float max_v = std::max(a.v, std::max(b.v, c.v));
    const int u0 = std::max(0, static_cast<int>(std::floor(min_u)));
    const int u1 = std::min(kW - 1, static_cast<int>(std::ceil(max_u)));
    const int v0 = std::max(0, static_cast<int>(std::floor(min_v)));
    const int v1 = std::min(kH - 1, static_cast<int>(std::ceil(max_v)));
    const float area = (b.u - a.u) * (c.v - a.v) - (c.u - a.u) * (b.v - a.v);
    if (std::abs(area) < 1e-4f || u0 > u1 || v0 > v1)
        return;
    const float inv_area = 1.f / area;
    const float iz_a = 1.f / a.z;
    const float iz_b = 1.f / b.z;
    const float iz_c = 1.f / c.z;
    for (int v = v0; v <= v1; ++v) {
        for (int u = u0; u <= u1; ++u) {
            const float pu = static_cast<float>(u) + 0.5f;
            const float pv = static_cast<float>(v) + 0.5f;
            const float w0 = ((b.u - pu) * (c.v - pv) - (c.u - pu) * (b.v - pv)) * inv_area;
            const float w1 = ((c.u - pu) * (a.v - pv) - (a.u - pu) * (c.v - pv)) * inv_area;
            const float w2 = 1.f - w0 - w1;
            if (w0 < 0.f || w1 < 0.f || w2 < 0.f)
                continue;
            const float iz = w0 * iz_a + w1 * iz_b + w2 * iz_c;
            float& slot = depth[v * kW + u];
            if (iz > slot) {
                slot = iz;
                image[v * kW + u] = shade;
            }
        }
    }
}

float sobelAt(const uint8_t* image, int u, int v) {
    auto at = [&](int x, int y) { return static_cast<float>(image[y * kW + x]); };
    const float gx = -at(u - 1, v - 1) + at(u + 1, v - 1) - 2.f * at(u - 1, v) + 2.f * at(u + 1, v) -
        at(u - 1, v + 1) + at(u + 1, v + 1);
    const float gy = -at(u - 1, v - 1) - 2.f * at(u, v - 1) - at(u + 1, v - 1) + at(u - 1, v + 1) +
        2.f * at(u, v + 1) + at(u + 1, v + 1);
    return std::abs(gx) + std::abs(gy);
}

int patchSad(const uint8_t* current, const uint8_t* previous, int u, int v, int du, int dv) {
    int sad = 0;
    for (int y = 0; y < kCell; ++y) {
        for (int x = 0; x < kCell; ++x) {
            const int pu = u + du + x;
            const int pv = v + dv + y;
            if (pu < 0 || pv < 0 || pu >= kW || pv >= kH)
                sad += 255;
            else
                sad += std::abs(current[(v + y) * kW + (u + x)] - previous[pv * kW + pu]);
        }
    }
    return sad;
}

}  // namespace

CameraModel CameraModel::build(const mjModel* model) {
    CameraModel camera;
    camera.site = mj_name2id(model, mjOBJ_SITE, "camera");
    camera.fov_h_deg = numericOr(model, "camera_spec", 2, 60);
    const int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
    const int gate = mj_name2id(model, mjOBJ_GEOM, "gate");
    const int marker = mj_name2id(model, mjOBJ_GEOM, "marker");
    if (floor >= 0)
        addPlane(camera.tris, floor, kShadeFloor, static_cast<float>(model->geom_size[3 * floor]), static_cast<float>(model->geom_size[3 * floor + 1]));
    if (gate >= 0) {
        addMesh(camera.tris, model, gate, kShadeGate);
        CameraModel::Box box;
        box.geom = gate;
        box.cls = kClsGate;
        localAabb(model, gate, box.local_min, box.local_max);
        camera.boxes.push_back(box);
    }
    if (marker >= 0) {
        const mjtNum* size = model->geom_size + 3 * marker;
        addCylinder(camera.tris, marker, kShadeMarker, static_cast<float>(size[0]), static_cast<float>(size[1]));
        CameraModel::Box box;
        box.geom = marker;
        box.cls = kClsCube;
        localAabb(model, marker, box.local_min, box.local_max);
        camera.boxes.push_back(box);
    }
    return camera;
}

CameraView::CameraView() : image(kW* kH), previous(kW* kH), depth(kW* kH) {}

void CameraView::clear() {
    has_previous = false;
    std::fill(image.begin(), image.end(), 0);
    std::fill(previous.begin(), previous.end(), 0);
}

void CameraView::render(const mjModel*, const mjData* data, const CameraModel& camera, float* features) {
    std::fill(image.begin(), image.end(), 0);
    std::fill(depth.begin(), depth.end(), 0.f);
    std::fill(features, features + HYDRUS_CAM_FEATURE_SIZE, 0.f);
    if (camera.site < 0) {
        std::fprintf(stderr, "[hydrus_camera] missing camera site\n");
        return;
    }
    const mjtNum* cam_pos = data->site_xpos + 3 * camera.site;
    const mjtNum* cam_mat = data->site_xmat + 9 * camera.site;
    const double focal = (kW * 0.5) / std::tan(camera.fov_h_deg * 3.141592653589793 / 360.0);

    for (const CameraModel::Tri& tri : camera.tris) {
        const mjtNum* pos = data->geom_xpos + 3 * tri.geom;
        const mjtNum* mat = data->geom_xmat + 9 * tri.geom;
        CamPoint cam_pts[3];
        for (int k = 0; k < 3; ++k) {
            float world[3];
            worldPoint(world, pos, mat, tri.v[k]);
            cam_pts[k] = toCamera(cam_pos, cam_mat, world);
        }
        CamPoint clipped[4];
        const int nclip = clipNear(cam_pts, 3, clipped, 0.15f);
        if (nclip < 3)
            continue;
        ScreenPoint screen[4];
        bool ok = true;
        for (int k = 0; k < nclip; ++k) {
            screen[k] = projectCam(clipped[k], focal);
            ok = ok && screen[k].ok;
        }
        if (!ok)
            continue;
        for (int k = 1; k < nclip - 1; ++k)
            rasterTriangle(image.data(), depth.data(), screen[0], screen[k], screen[k + 1], tri.shade);
    }

    struct Seen {
        int cls = 0;
        float cx = 0;
        float cy = 0;
        float w = 0;
        float h = 0;
    };
    Seen seen[8];
    int seen_n = 0;
    for (const CameraModel::Box& box : camera.boxes) {
        const mjtNum* pos = data->geom_xpos + 3 * box.geom;
        const mjtNum* mat = data->geom_xmat + 9 * box.geom;
        float u_min = 0;
        float u_max = 0;
        float v_min = 0;
        float v_max = 0;
        bool any = false;
        const float xs[2] = {box.local_min[0], box.local_max[0]};
        const float ys[2] = {box.local_min[1], box.local_max[1]};
        const float zs[2] = {box.local_min[2], box.local_max[2]};
        for (float x : xs) {
            for (float y : ys) {
                for (float z : zs) {
                    const float local[3] = {x, y, z};
                    float world[3];
                    worldPoint(world, pos, mat, local);
                    const ScreenPoint p = project(cam_pos, cam_mat, world, focal);
                    if (!p.ok)
                        continue;
                    if (!any) {
                        u_min = u_max = p.u;
                        v_min = v_max = p.v;
                        any = true;
                    } else {
                        u_min = std::min(u_min, p.u);
                        u_max = std::max(u_max, p.u);
                        v_min = std::min(v_min, p.v);
                        v_max = std::max(v_max, p.v);
                    }
                }
            }
        }
        if (!any || u_max <= u_min || v_max <= v_min || seen_n >= 8)
            continue;
        Seen& slot = seen[seen_n++];
        slot.cls = box.cls;
        slot.cx = 0.5f * (u_min + u_max) / kW - 0.5f;
        slot.cy = 0.5f * (v_min + v_max) / kH - 0.5f;
        slot.w = (u_max - u_min) / kW;
        slot.h = (v_max - v_min) / kH;
    }
    for (int i = 1; i < seen_n; ++i) {
        const Seen key = seen[i];
        int j = i;
        while (j > 0) {
            const bool key_gate = key.cls == kClsGate;
            const bool prev_gate = seen[j - 1].cls == kClsGate;
            const bool before = (key_gate && !prev_gate) || (key_gate == prev_gate && key.h > seen[j - 1].h);
            if (!before)
                break;
            seen[j] = seen[j - 1];
            --j;
        }
        seen[j] = key;
    }
    const int nwrite = std::min(seen_n, static_cast<int>(HYDRUS_CAM_BOXES));
    for (int i = 0; i < nwrite; ++i) {
        float* dst = features + HYDRUS_CAM_BOX_STRIDE * i;
        if (seen[i].cls >= 0 && seen[i].cls < 3)
            dst[seen[i].cls] = 1.f;
        dst[3] = seen[i].cx;
        dst[4] = seen[i].cy;
        dst[5] = seen[i].w;
        dst[6] = seen[i].h;
    }

    float* edges = features + HYDRUS_CAM_BOXES * HYDRUS_CAM_BOX_STRIDE;
    for (int gy = 0; gy < HYDRUS_CAM_GRID_Y; ++gy) {
        for (int gx = 0; gx < HYDRUS_CAM_GRID_X; ++gx) {
            float sum = 0;
            const int u0 = gx * kCell;
            const int v0 = gy * kCell;
            for (int y = 0; y < kCell; ++y) {
                for (int x = 0; x < kCell; ++x) {
                    const int u = u0 + x;
                    const int v = v0 + y;
                    if (u < 1 || v < 1 || u >= kW - 1 || v >= kH - 1)
                        continue;
                    sum += sobelAt(image.data(), u, v);
                }
            }
            edges[gy * HYDRUS_CAM_GRID_X + gx] = sum / (static_cast<float>(kCell * kCell) * 1020.f);
        }
    }

    float* flow = edges + HYDRUS_CAM_EDGES;
    if (has_previous) {
        constexpr int kSearch = 8;
        for (int gy = 0; gy < HYDRUS_CAM_GRID_Y; ++gy) {
            for (int gx = 0; gx < HYDRUS_CAM_GRID_X; ++gx) {
                const int u0 = gx * kCell;
                const int v0 = gy * kCell;
                int best = patchSad(image.data(), previous.data(), u0, v0, 0, 0);
                int best_du = 0;
                int best_dv = 0;
                for (int dv = -kSearch; dv <= kSearch; dv += 2) {
                    for (int du = -kSearch; du <= kSearch; du += 2) {
                        const int sad = patchSad(image.data(), previous.data(), u0, v0, du, dv);
                        if (sad < best) {
                            best = sad;
                            best_du = du;
                            best_dv = dv;
                        }
                    }
                }
                const int cell = gy * HYDRUS_CAM_GRID_X + gx;
                flow[2 * cell] = static_cast<float>(best_du) / kCell;
                flow[2 * cell + 1] = static_cast<float>(best_dv) / kCell;
            }
        }
    }
    previous.swap(image);
    has_previous = true;
}

}  // namespace hydrus
