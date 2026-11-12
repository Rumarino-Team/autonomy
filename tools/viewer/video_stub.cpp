#include <cstddef>
extern "C" const char *video_default_source() { return "127.0.0.1:5610"; }
extern "C" bool video_open(const char *, int *, int *) { return false; }
extern "C" bool video_get_size(int *, int *) { return false; }
extern "C" bool video_frame(unsigned char *, size_t, int, int) { return false; }
extern "C" bool video_has_error() { return false; }
extern "C" void video_close() {}
