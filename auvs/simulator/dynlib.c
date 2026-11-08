#if !defined(_WIN32) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

#include "dynlib.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

void *dynlib_open(const char *path) {
    wchar_t wide[32768];
    wchar_t full[32768];
    int converted;
    DWORD length;
    if (!path)
        return 0;
    converted = MultiByteToWideChar(CP_UTF8, 0, path, -1, wide, 32768);
    if (converted <= 0)
        return 0;
    length = GetFullPathNameW(wide, 32768, full, 0);
    return (void *)LoadLibraryExW(
        length > 0 && length < 32768 ? full : wide,
        0,
        LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS
    );
}

void *dynlib_symbol(void *library, const char *name) {
    if (!library || !name)
        return 0;
    return (void *)GetProcAddress((HMODULE)library, name);
}

void dynlib_close(void *library) {
    if (library)
        FreeLibrary((HMODULE)library);
}

int dynlib_module_path(void *address, char *out, size_t out_len) {
    HMODULE module = 0;
    wchar_t wide[32768];
    DWORD length;
    int wrote;
    if (!address || !out || out_len < 2)
        return 0;
    if (!GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            (LPCWSTR)address,
            &module
        ))
        return 0;
    length = GetModuleFileNameW(module, wide, 32768);
    if (length == 0 || length >= 32768)
        return 0;
    wrote = WideCharToMultiByte(CP_UTF8, 0, wide, (int)length, out, (int)out_len - 1, 0, 0);
    if (wrote <= 0)
        return 0;
    out[wrote] = 0;
    return 1;
}

#else
#include <dlfcn.h>
#include <string.h>

void *dynlib_open(const char *path) {
    if (!path)
        return 0;
    return dlopen(path, RTLD_NOW | RTLD_LOCAL);
}

void *dynlib_symbol(void *library, const char *name) {
    if (!library || !name)
        return 0;
    return dlsym(library, name);
}

void dynlib_close(void *library) {
    if (library)
        dlclose(library);
}

int dynlib_module_path(void *address, char *out, size_t out_len) {
    Dl_info info;
    size_t length;
    if (!address || !out || out_len < 2)
        return 0;
    if (!dladdr(address, &info) || !info.dli_fname)
        return 0;
    length = strlen(info.dli_fname);
    if (length + 1 > out_len)
        return 0;
    memcpy(out, info.dli_fname, length + 1);
    return 1;
}
#endif
