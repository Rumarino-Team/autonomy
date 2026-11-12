#pragma once
#include <cstddef>

// The relay copies encoded packets and never waits for the network on submit.
bool rtp_relay_start(const char *destination);
void rtp_relay_submit(const unsigned char *data, size_t size, bool keyframe);
void rtp_relay_stop();
