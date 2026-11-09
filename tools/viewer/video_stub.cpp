extern "C" bool video_open(const char *, int *, int *) { return false; }

extern "C" bool video_frame(unsigned char *) { return false; }

extern "C" void video_close() {}
