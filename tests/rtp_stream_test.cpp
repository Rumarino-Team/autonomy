#include "../auvs/proteus_hwd/rtp_relay.hpp"
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <thread>
#include <cstdio>
#include <string>
#include <vector>

extern "C" bool video_open(const char *, int *, int *);
extern "C" bool video_get_size(int *, int *);
extern "C" bool video_frame(unsigned char *, size_t, int, int);
extern "C" void video_close();

static unsigned int free_port() {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  assert(fd >= 0);
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  assert(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
  socklen_t size = sizeof(address);
  assert(getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size) == 0);
  const unsigned int result = ntohs(address.sin_port); close(fd); return result;
}

static bool feed_until_frame(int full_width, int height, int expected_width) {
  const std::string pipeline_text =
      "videotestsrc is-live=true pattern=smpte ! video/x-raw,width=" + std::to_string(full_width) +
      ",height=" + std::to_string(height) + ",framerate=30/1 ! "
      "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=15 ! "
      "video/x-h264,stream-format=byte-stream,alignment=au ! appsink name=encoded sync=false";
  GError *error = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_text.c_str(), &error);
  assert(!error && pipeline);
  GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "encoded");
  assert(sink && gst_element_set_state(pipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE);
  bool got_frame = false;
  for (int i = 0; i < 180 && !got_frame; ++i) {
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 100 * GST_MSECOND);
    if (sample) {
      GstBuffer *buffer = gst_sample_get_buffer(sample);
      GstMapInfo map{};
      if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        rtp_relay_submit(map.data, map.size, !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT));
        gst_buffer_unmap(buffer, &map);
      }
      gst_sample_unref(sample);
    }
    int width = 0, frame_height = 0;
    if (video_get_size(&width, &frame_height) && width == expected_width && frame_height == height) {
      std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 4);
      assert(!video_frame(pixels.data(), pixels.size() - 1, width, height));
      assert(!video_frame(pixels.data(), pixels.size(), width - 1, height));
      got_frame = video_frame(pixels.data(), pixels.size(), width, height);
      if (got_frame) {
        bool has_color = false;
        for (size_t j = 0; j + 3 < pixels.size(); j += 4)
          if (pixels[j] || pixels[j + 1] || pixels[j + 2]) { has_color = true; break; }
        assert(has_color);
      }
    }
  }
  gst_element_set_state(pipeline, GST_STATE_NULL); gst_object_unref(sink); gst_object_unref(pipeline);
  return got_frame;
}

int main() {
  gst_init(nullptr, nullptr);
  GstElementFactory *encoder = gst_element_factory_find("x264enc");
  if (!encoder) { std::puts("SKIP: RTP integration test needs the optional x264enc plugin"); return 0; }
  gst_object_unref(encoder);
  const unsigned int port = free_port();
  const auto dest = std::string("127.0.0.1:") + std::to_string(port);
  const auto source = std::string("rtp://") + dest;
  int width = 0, height = 0;
  assert(!video_open("invalid-source", &width, &height));
  assert(video_open(source.c_str(), &width, &height));
  assert(rtp_relay_start(dest.c_str()));
  assert(feed_until_frame(640, 240, 320));
  // Exceed the bounded queue's packet limit: delivery must resume at a keyframe.
  std::vector<unsigned char> oversized(4 * 1024 * 1024 + 1);
  rtp_relay_submit(oversized.data(), oversized.size(), true);
  assert(feed_until_frame(800, 300, 400));
  rtp_relay_stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  assert(rtp_relay_start(dest.c_str()));
  assert(feed_until_frame(640, 240, 320));
  rtp_relay_stop(); video_close();
  std::puts("RTP relay, drop recovery, stereo crop, restart, and resolution changes passed");
}
