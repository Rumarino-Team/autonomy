#pragma once
#include "../internal.hpp"

/* Private typed wrappers over the same C bump pool used by model records. */
template <class T> struct BindingPool {
    core::Buffer<T> records;
    PoolAllocator pool{};

    explicit BindingPool(SimAllocator a = {})
        : records(a) {
    }

    void init(size_t count) {
        core::require(count <= UINT32_MAX, "binding count");
        records.reserve(count);
        core::require(
            pool_init(
                &pool,
                records.p,
                core::bytes(records.cap, sizeof(T)),
                sizeof(T),
                alignof(T),
                uint32_t(count)
            ),
            "binding pool init"
        );
    }

    void push_back(decltype(T::data) value) {
        auto *p = static_cast<T *>(pool_alloc(&pool));
        core::require(p, "binding capacity", SIM_CAPACITY);
        p->data = value;
        records.n = pool.count;
    }

    size_t size() const {
        return records.size();
    }

    auto &operator[](size_t i) {
        return records[i].data;
    }

    const auto &operator[](size_t i) const {
        return records[i].data;
    }

    T &block(size_t i) {
        return records[i];
    }
};
