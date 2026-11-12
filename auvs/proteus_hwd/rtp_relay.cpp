#include "rtp_relay.hpp"
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
struct Packet { std::vector<unsigned char> bytes; bool keyframe; };
constexpr size_t max_packets = 4;
constexpr size_t max_packet_bytes = 4 * 1024 * 1024;
constexpr guint64 max_appsrc_bytes = 2 * 1024 * 1024;
std::thread worker;
std::mutex mutex;
std::condition_variable changed;
std::deque<Packet> packets;
std::atomic<bool> running{false};
std::atomic<bool> resync{false};
std::atomic<bool> reset_pipeline{false};
bool waiting_for_keyframe = true;
std::string host;
unsigned int port = 0;

bool parse_destination(const char *destination) {
  const std::string text(destination ? destination : "");
  const auto colon = text.rfind(':');
  if (colon == std::string::npos) return false;
  const auto ip = text.substr(0, colon);
  in_addr address{};
  if (inet_pton(AF_INET, ip.c_str(), &address) != 1) return false;
  const auto digits = text.substr(colon + 1);
  if (digits.empty() || digits.find_first_not_of("0123456789") != std::string::npos) return false;
  try {
    const auto n = std::stoul(digits);
    if (n == 0 || n > 65535) return false;
    host = ip; port = static_cast<unsigned int>(n); return true;
  } catch (...) { return false; }
}
void run() {
  const std::string desc =
      "appsrc name=encoded is-live=true format=time do-timestamp=true block=false "
      "caps=video/x-h264,stream-format=byte-stream ! h264parse ! "
      "rtph264pay pt=96 config-interval=-1 mtu=1200 ! udpsink host=" + host +
      " port=" + std::to_string(port) + " sync=false async=false";
  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(desc.c_str(), &error);
  if (error || !pipeline) {
    std::fprintf(stderr, "[rtp] cannot create relay: %s\n", error ? error->message : "unknown error");
    if (error) g_error_free(error);
    if (pipeline) gst_object_unref(pipeline);
    running = false; return;
  }
  GstElement *source = gst_bin_get_by_name(GST_BIN(pipeline), "encoded");
  GstBus *bus = gst_element_get_bus(pipeline);
  if (!source || gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    std::fputs("[rtp] cannot start relay\n", stderr); running = false;
    if (source) gst_object_unref(source);
    gst_object_unref(bus); gst_element_set_state(pipeline, GST_STATE_NULL); gst_object_unref(pipeline); return;
  }
  std::fprintf(stderr, "[rtp] relaying encoded stereo H264 to %s:%u\n", host.c_str(), port);
  while (running) {
    GstMessage *message = gst_bus_pop_filtered(bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (message) {
      GError *err = nullptr; gchar *debug = nullptr;
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) gst_message_parse_error(message, &err, &debug);
      std::fprintf(stderr, "[rtp] relay stopped: %s\n", err ? err->message : "end of stream");
      if (err) g_error_free(err);
      g_free(debug); gst_message_unref(message); running = false; break;
    }
    Packet packet;
    {
      std::unique_lock<std::mutex> lock(mutex);
      changed.wait_for(lock, std::chrono::milliseconds(100), [] { return !running || !packets.empty(); });
      if (!running) break;
      if (packets.empty()) continue;
      packet = std::move(packets.front()); packets.pop_front();
    }
    if (resync.load()) continue;
    if (reset_pipeline.exchange(false)) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        std::fputs("[rtp] relay restart failed\n", stderr); running = false; break;
      }
      if (!packet.keyframe) { resync = true; continue; }
    }
    if (gst_app_src_get_current_level_bytes(GST_APP_SRC(source)) > max_appsrc_bytes) {
      resync = true;
      std::lock_guard<std::mutex> lock(mutex);
      packets.clear(); waiting_for_keyframe = true;
      continue;
    }
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, packet.bytes.size(), nullptr);
    if (!buffer) { resync = true; continue; }
    gst_buffer_fill(buffer, 0, packet.bytes.data(), packet.bytes.size());
    if (!packet.keyframe) GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
    if (gst_app_src_push_buffer(GST_APP_SRC(source), buffer) != GST_FLOW_OK) resync = true;
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(source); gst_object_unref(bus); gst_object_unref(pipeline);
}
} // namespace

bool rtp_relay_start(const char *destination) {
  rtp_relay_stop();
  if (!parse_destination(destination)) { std::fputs("[rtp] destination must be IPv4:port\n", stderr); return false; }
  gst_init(nullptr, nullptr);
  waiting_for_keyframe = true; resync = false; reset_pipeline = false; running = true;
  try {
    worker = std::thread([] {
      try { run(); }
      catch (const std::exception &error) { std::fprintf(stderr, "[rtp] relay stopped: %s\n", error.what()); running = false; }
      catch (...) { std::fputs("[rtp] relay stopped unexpectedly\n", stderr); running = false; }
    });
  } catch (...) { running = false; return false; }
  return true;
}
void rtp_relay_submit(const unsigned char *data, size_t size, bool keyframe) {
  if (!running || !data || size == 0) return;
  if (size > max_packet_bytes) { resync = true; return; }
  std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
  if (!lock.owns_lock()) { resync = true; return; }
  if (resync.exchange(false) || packets.size() >= max_packets) {
    packets.clear(); waiting_for_keyframe = true; reset_pipeline = true;
  }
  if (waiting_for_keyframe && !keyframe) return;
  try {
    packets.push_back(Packet{std::vector<unsigned char>(data, data + size), keyframe});
    waiting_for_keyframe = false;
  } catch (...) { resync = true; return; }
  lock.unlock(); changed.notify_one();
}
void rtp_relay_stop() {
  running = false; changed.notify_all();
  if (worker.joinable()) worker.join();
  std::lock_guard<std::mutex> lock(mutex);
  packets.clear(); waiting_for_keyframe = true; resync = false; reset_pipeline = false;
}
