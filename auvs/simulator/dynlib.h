#pragma once

#include <stddef.h>

#if defined(__GNUC__)
#define DYNLIB_API __attribute__((visibility("default")))
#else
#define DYNLIB_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

DYNLIB_API void *dynlib_open(const char *path);
DYNLIB_API void *dynlib_symbol(void *library, const char *name);
DYNLIB_API void dynlib_close(void *library);
/* UTF-8 path of the module containing address. Returns 1 on success. */
DYNLIB_API int dynlib_module_path(void *address, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
