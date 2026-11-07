#include "../backends/wire.hpp"
#include "../internal.hpp"
#include <iostream>
using namespace core;

namespace {
void test(bool b, const char *message) {
    if (!b)
        throw std::runtime_error(message);
}

struct Owner {
    Simulation m{};

    ~Owner() {
        simulation_destroy(&m);
    }
};

struct Counter {
    size_t calls = 0, live = 0, fail = SIZE_MAX;

    static void *alloc(void *u, size_t n, size_t a) {
        auto &c = *static_cast<Counter *>(u);
        if (c.calls++ == c.fail)
            return nullptr;
        auto *p = default_alloc(nullptr, n, a);
        if (p)
            ++c.live;
        return p;
    }

    static void free(void *u, void *p, size_t n, size_t a) {
        if (p) {
            --static_cast<Counter *>(u)->live;
            default_free(nullptr, p, n, a);
        }
    }

    SimAllocator api() {
        return {this, alloc, free};
    }
};

ObjectBlock *
add(Simulation &m, uint32_t id, const MeshData &mesh = {}, MathVector3f position = v()) {
    UnderwaterBody b;
    underwater_body_default(&b);
    b.id = id;
    b.initial_pose.pos = position;
    std::snprintf(b.name, 64, "body%u", id);
    Collider c;
    collider_default(&c);
    if (mesh.positions) {
        c.data.kind = SIM_MESH;
        c.geometry = mesh;
    }
    ObjectBlock *out = nullptr;
    char e[256];
    check(simulation_add_object(&m, &b, &c, 1, 0, &out, e, sizeof e), e);
    return out;
}
} // namespace

int main() {
    try {
        alignas(32) uint8_t memory[128];
        ArenaAllocator arena;
        arena_init(&arena, memory + 1, 127);
        auto *p = arena_alloc(&arena, 32, 32);
        test(p && uintptr_t(p) % 32 == 0, "arena alignment");
        size_t offset = arena.offset;
        test(!arena_alloc(&arena, SIZE_MAX, 16) && arena.offset == offset, "overflow atomicity");
        test(!arena_alloc(&arena, 1, 3), "invalid alignment");
        arena_reset(&arena);
        test(arena.offset == 0, "scratch reset");
        PoolAllocator pool;
        test(pool_init(&pool, memory, 128, 16, 16, 8), "pool init");
        void *first = pool_alloc(&pool);
        for (int i = 1; i < 8; ++i)
            test(pool_alloc(&pool), "pool slots");
        test(!pool_alloc(&pool), "pool capacity");
        pool_reset(&pool);
        test(first == pool_alloc(&pool), "pool reuse after reset");
        ListBase list{};
        IntrusiveDataNode a{}, b{};
        test(
            sim_list_append(&list, &a) == SIM_OK && sim_list_append(&list, &b) == SIM_OK, "append"
        );
        test(list.last->prev == &a.link && list.first->next == &b.link, "reverse traversal");
        test(sim_list_append(&list, &a) == SIM_INVALID, "duplicate membership");
        test(
            sim_list_unlink(&list, &a) == SIM_OK && list.first == &b.link &&
                list.first->prev == nullptr,
            "unlink"
        );
        const void *cast = nullptr;
        a.owner = &list;
        a.kind = SIM_DATA_OBJECT;
        test(
            sim_node_cast(&a, SIM_DATA_OBJECT, &list, &cast) == SIM_OK && cast == &a, "checked cast"
        );
        test(
            sim_node_cast(&a, SIM_DATA_FAULT, &list, &cast) == SIM_INVALID && cast == nullptr,
            "type rejection"
        );
        for (size_t failure = 0; failure < 14; ++failure) {
            Counter counter;
            counter.fail = failure;
            SimulationConfig c;
            simulation_config_default(&c);
            c.allocator = counter.api();
            {
                Owner owner;
                char e[256];
                auto result = simulation_init(&owner.m, &c, e, sizeof e);
                test(result == SIM_NO_MEMORY, "injected allocation failure");
            }
            test(counter.live == 0, "partial init leak");
        }
        Counter counter;
        {
            Owner owner;
            auto &m = owner.m;
            SimulationConfig config;
            simulation_config_default(&config);
            config.allocator = counter.api();
            char e[256];
            check(simulation_init(&m, &config, e, sizeof e), e);
            MathVector3f positions[] = {v(0, 0, 0), v(1, 0, 0), v(0, 1, 0), v(0, 0, 1)};
            SimTriangle triangles[] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
            MeshData geometry{positions, 4, triangles, 4};
            auto *one = add(m, 10, geometry);
            for (auto &position : positions)
                reinterpret_cast<unsigned char *>(&position)[12] = 0x7f;
            positions[0].buf[0] = -0.0f;
            auto *two = add(m, 20, geometry, v(4, 0, 0));
            test(
                one->colliders[0].geometry.positions == two->colliders[0].geometry.positions,
                "shared geometry"
            );
            test(m.geometry_count == 1, "geometry count");
            positions[1].buf[0] = 2;
            auto *three = add(m, 30, geometry);
            test(
                three->colliders[0].geometry.positions != one->colliders[0].geometry.positions,
                "scaled geometry"
            );
            auto *primitive = add(m, 40);
            test(!primitive->colliders[0].geometry.positions, "primitive geometry");
            for (uint32_t i = 0; i < 20; ++i)
                add(m, 100 + i);
            test(m.object_table[0] == one, "stable addresses");
            check(simulation_finalize(&m, e, sizeof e), e);
            test(
                simulation_add_module(&m, "thrusters", e, sizeof e) == SIM_INVALID,
                "frozen rejection"
            );
            Owner clone;
            check(simulation_clone(&m, &clone.m, e, sizeof e), e);
            test(!clone.m.frozen, "clone is draft");
            check(simulation_finalize(&clone.m, e, sizeof e), e);
            auto *p1 = clone.m.object_table[0]->colliders[0].geometry.positions;
            test(
                p1 != one->colliders[0].geometry.positions &&
                    p1 == clone.m.object_table[1]->colliders[0].geometry.positions,
                "clone sharing/isolation"
            );
            WireBuffer encoded;
            encode_scene(m, encoded);
            auto decoded = decode_scene({encoded.data(), encoded.size()});
            test(
                decoded->value.object_count == m.object_count && decoded->value.geometry_count == 2,
                "worker scene round trip"
            );
            test(
                decoded->value.object_table[0]->colliders[0].geometry.positions ==
                    decoded->value.object_table[1]->colliders[0].geometry.positions,
                "wire sharing"
            );
            auto last = encoded[encoded.size() - 1];
            encoded[encoded.size() - 1] = 0;
            bool bad_reference = false;
            try {
                decode_scene({encoded.data(), encoded.size()});
            } catch (const Error &) {
                bad_reference = true;
            }
            require(bad_reference, "primitive mesh reference rejection");
            encoded[encoded.size() - 1] = last;
            encoded[0] = 'X';
            bool rejected = false;
            try {
                decode_scene({encoded.data(), encoded.size()});
            } catch (const Error &) {
                rejected = true;
            }
            test(rejected, "version rejection");
            encoded[0] = 'A';
            rejected = false;
            try {
                decode_scene({encoded.data(), encoded.size() - 1});
            } catch (const Error &) {
                rejected = true;
            }
            test(rejected, "truncation rejection");
            auto *mutable_one = const_cast<ObjectBlock *>(m.object_table[0]);
            auto saved = mutable_one->id.owner;
            mutable_one->id.owner = &clone.m;
            test(simulation_validate(&m, e, sizeof e) == SIM_INVALID, "foreign owner rejection");
            mutable_one->id.owner = saved;
            for (unsigned i = 0; i < 64; ++i) {
                Owner repeated;
                check(simulation_clone(&m, &repeated.m, e, sizeof e), e);
                check(simulation_finalize(&repeated.m, e, sizeof e), e);
            }
        }
        test(counter.live == 0, "model destruction leak");
        std::cout << "allocator, intrusive database, sharing, clone, and worker "
                     "codec checks passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
