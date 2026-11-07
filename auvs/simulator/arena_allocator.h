#ifndef SIM_ARENA_ALLOCATOR_H
#define SIM_ARENA_ALLOCATOR_H

#include <stddef.h>
#include <stdint.h>
#ifndef SIM_API
#if defined(__GNUC__)
#define SIM_API __attribute__((visibility("default")))
#else
#define SIM_API
#endif
#endif
#ifdef __cplusplus
extern "C" {
#endif
/* Borrowed backing memory. Reset invalidates every allocation from this arena.
 */
typedef struct ArenaAllocator {
    uint8_t *data;
    size_t capacity;
    size_t offset;
} ArenaAllocator;

typedef struct PoolAllocator {
    ArenaAllocator arena;
    size_t element_size;
    size_t alignment;
    size_t stride;
    uint32_t capacity;
    uint32_t count;
} PoolAllocator;

SIM_API void arena_init(ArenaAllocator *, void *, size_t);
SIM_API void *arena_alloc(ArenaAllocator *, size_t, size_t);
SIM_API void arena_reset(ArenaAllocator *);
SIM_API int pool_init(PoolAllocator *, void *, size_t, size_t, size_t, uint32_t);
SIM_API void *pool_alloc(PoolAllocator *);
SIM_API void pool_reset(PoolAllocator *);

#ifdef __cplusplus
}
#endif

#endif
