#include "blender_link.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace blender_link {
namespace {

constexpr char kMagic[4] = {'A', 'M', 'J', 'B'};
constexpr std::uint16_t kVersion = 1;
constexpr int kDefaultPort = 9760;
constexpr double kPeriod = 1.0 / 30.0;

#pragma pack(push, 1)
struct Packet {
    char magic[4];
    std::uint16_t version;
    std::uint16_t count;
    double sim_time;
    Pose pose[kMaxPoses];
};
#pragma pack(pop)

static_assert(sizeof(Pose) == 88, "Blender pose must stay 32 + 7 doubles");
static_assert(sizeof(Packet) == 16 + 88 * kMaxPoses, "Blender packet layout drifted");

// pool_scene.blend uses the Stonefish map. M = diag(-1, 1, -1) sends NED
// (north, east, down) to Blender (-north, east, -down), and M is its own inverse.
void nedToBlenderPos(const mjtNum ned[3], double out[3]) {
    out[0] = static_cast<double>(-ned[0]);
    out[1] = static_cast<double>(ned[1]);
    out[2] = static_cast<double>(-ned[2]);
}

void nedToBlenderQuat(const mjtNum rotation[9], double out[4]) {
    // Rb = M * R * M. Negate rows 0 and 2, then negate columns 0 and 2.
    mjtNum blender[9];
    blender[0] = rotation[0];
    blender[1] = -rotation[1];
    blender[2] = rotation[2];
    blender[3] = -rotation[3];
    blender[4] = rotation[4];
    blender[5] = -rotation[5];
    blender[6] = rotation[6];
    blender[7] = -rotation[7];
    blender[8] = rotation[8];

    mjtNum quat[4];
    mju_mat2Quat(quat, blender);
    for (int i = 0; i < 4; ++i)
        out[i] = static_cast<double>(quat[i]);
}

void writeName(char dst[32], const char* name) {
    std::memset(dst, 0, 32);
    if (name == nullptr)
        return;
    std::snprintf(dst, 32, "%s", name);
}

bool addBody(const mjModel* model, const mjData* data, int body, Pose* out, int& count) {
    if (body < 0 || count >= kMaxPoses)
        return false;
    const char* name = mj_id2name(model, mjOBJ_BODY, body);
    if (name == nullptr || name[0] == '\0')
        return false;
    Pose& pose = out[count++];
    writeName(pose.name, name);
    nedToBlenderPos(data->xpos + 3 * body, pose.pos);
    nedToBlenderQuat(data->xmat + 9 * body, pose.quat);
    return true;
}

bool addCamera(const mjModel* model, const mjData* data, int site, Pose* out, int& count) {
    if (site < 0 || count >= kMaxPoses)
        return false;
    if (mj_id2name(model, mjOBJ_SITE, site) == nullptr)
        return false;
    Pose& pose = out[count++];
    writeName(pose.name, "camera");
    nedToBlenderPos(data->site_xpos + 3 * site, pose.pos);
    nedToBlenderQuat(data->site_xmat + 9 * site, pose.quat);
    return true;
}

int portFromEnv() {
    const char* env = std::getenv("AUV_BLENDER_PORT");
    if (env == nullptr || env[0] == '\0')
        return kDefaultPort;
    char* end = nullptr;
    const long value = std::strtol(env, &end, 10);
    if (end == env || *end != '\0' || value < 1 || value > 65535) {
        std::fprintf(stderr, "[auv_mujoco] AUV_BLENDER_PORT '%s' is not a port\n", env);
        return -1;
    }
    return static_cast<int>(value);
}

}  // namespace

int collectPoses(const mjModel* model, const mjData* data, int vehicle_body, int camera_site, Pose out[kMaxPoses]) {
    int count = 0;
    if (model == nullptr || data == nullptr)
        return 0;
    addBody(model, data, vehicle_body, out, count);
    for (int text = 0; text < model->ntext; ++text) {
        const char* text_name = mj_id2name(model, mjOBJ_TEXT, text);
        if (text_name == nullptr || std::strncmp(text_name, "fuzz:", 5) != 0 || text_name[5] == '\0')
            continue;
        addBody(model, data, mj_name2id(model, mjOBJ_BODY, text_name + 5), out, count);
    }
    addCamera(model, data, camera_site, out, count);
    return count;
}

Publisher* Publisher::open() {
    const int port = portFromEnv();
    if (port < 0)
        return nullptr;
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::perror("[auv_mujoco] blender socket");
        return nullptr;
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::perror("[auv_mujoco] blender socket");
        ::close(fd);
        return nullptr;
    }
    std::fprintf(stdout, "[auv_mujoco] blender udp://127.0.0.1:%d\n", port);
    std::fflush(stdout);
    return new Publisher(fd, port);
}

Publisher::Publisher(int fd, int port) : fd_(fd), port_(port) {}

Publisher::~Publisher() {
    if (fd_ >= 0)
        ::close(fd_);
}

void Publisher::publish(const mjModel* model, const mjData* data, int vehicle_body, int camera_site) {
    if (model == nullptr || data == nullptr || fd_ < 0)
        return;
    const double time = data->time;
    // First sample, the next 30 Hz slot, or a rewind that lands before this slot.
    if (started_ && time + 1e-9 < next_time_ && time >= next_time_ - kPeriod - 1e-6)
        return;

    Packet packet{};
    std::memcpy(packet.magic, kMagic, 4);
    packet.version = kVersion;
    packet.sim_time = time;
    packet.count = static_cast<std::uint16_t>(collectPoses(model, data, vehicle_body, camera_site, packet.pose));
    if (packet.count == 0)
        return;

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(static_cast<std::uint16_t>(port_));
    dest.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const std::size_t bytes = 16 + sizeof(Pose) * packet.count;
    const ssize_t sent = ::sendto(
        fd_,
        &packet,
        bytes,
        MSG_DONTWAIT,
        reinterpret_cast<sockaddr*>(&dest),
        sizeof(dest));
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != ECONNREFUSED && errno != ENOBUFS)
        std::perror("[auv_mujoco] blender send");

    started_ = true;
    next_time_ = time + kPeriod;
}

}  // namespace blender_link
