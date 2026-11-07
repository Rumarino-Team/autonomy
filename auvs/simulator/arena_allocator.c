#include "arena_allocator.h"
#include <string.h>

void arena_init(ArenaAllocator *a, void *p, size_t n) {
    if (a) {
        a->data = p;
        a->capacity = p ? n : 0;
        a->offset = 0;
    }
}

void *arena_alloc(ArenaAllocator *a, size_t n, size_t alignment) {
    if (!a || !a->data || !n || !alignment || (alignment & (alignment - 1)) ||
        a->offset > a->capacity)
        return NULL;
    uintptr_t base = (uintptr_t)a->data;
    if (a->offset > UINTPTR_MAX - base)
        return NULL;
    uintptr_t current = base + a->offset;
    size_t padding = (size_t)(-current & (alignment - 1));
    size_t remaining = a->capacity - a->offset;
    if (padding > remaining || n > remaining - padding)
        return NULL;
    if (padding > UINTPTR_MAX - current || n > UINTPTR_MAX - current - padding)
        return NULL;
    void *result = a->data + a->offset + padding;
    a->offset += padding + n;
    return result;
}

void arena_reset(ArenaAllocator *a) {
    if (a)
        a->offset = 0;
}

int pool_init(
    PoolAllocator *p, void *memory, size_t bytes, size_t size, size_t alignment, uint32_t count
) {
    if (!p)
        return 0;
    memset(p, 0, sizeof(*p));
    if (!size || !alignment || (alignment & (alignment - 1)) || size > SIZE_MAX - (alignment - 1))
        return 0;
    size_t stride = (size + alignment - 1) & ~(alignment - 1);
    if ((count && !memory) || count > SIZE_MAX / stride || bytes < count * stride ||
        ((uintptr_t)memory & (alignment - 1)))
        return 0;
    arena_init(&p->arena, memory, bytes);
    p->element_size = size;
    p->alignment = alignment;
    p->stride = stride;
    p->capacity = count;
    return 1;
}

void *pool_alloc(PoolAllocator *p) {
    if (!p || p->count == p->capacity)
        return NULL;
    void *v = arena_alloc(&p->arena, p->stride, p->alignment);
    if (v) {
        memset(v, 0, p->element_size);
        ++p->count;
    }
    return v;
}

void pool_reset(PoolAllocator *p) {
    if (p) {
        arena_reset(&p->arena);
        p->count = 0;
    }
}
