#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/world.h"
#include "tests/check.h"

static Sim_Config make_small_config(void)
{
    Sim_Config config = get_default_config();
    config.terrain_size[0] = 40.0f;
    config.terrain_size[1] = 30.0f;
    config.crater_count = 20;
    return config;
}

// The terrain the renderer and sensors read must be the surface physics collides with.
// Also pins down the height-field orientation: row 0 is north (+y), heights are +z.
static void test_terrain_matches_physics(Linear_Allocator *allocator)
{
    Sim_Config config = make_small_config();
    static World world;
    CHECK(create_world(&world, allocator, &config));

    v3 down = {0.0f, 0.0f, -1.0f};
    u32 state = 12345;
    for (u32 i = 0; i < 200; i++) {
        state = state * 1664525u + 1013904223u;
        f32 x = ((f32)(state >> 8) / 16777216.0f - 0.5f) * 38.0f;
        state = state * 1664525u + 1013904223u;
        f32 y = ((f32)(state >> 8) / 16777216.0f - 0.5f) * 28.0f;
        v3 origin = {x, y, world.terrain.max_height + 10.0f};
        f32 distance = cast_ray(&world, origin, down, 100.0f);
        CHECK(distance > 0.0f);
        CHECK_NEAR(origin.z - distance, get_terrain_height(&world.terrain, x, y), 1e-3);
    }
    destroy_world(&world);
}

// On flat ground a dropped box comes to rest exactly one half-extent above it.
static void test_box_comes_to_rest(Linear_Allocator *allocator)
{
    Sim_Config config = make_small_config();
    config.terrain_relief = 0.0f;
    config.crater_count = 0;
    static World world;
    CHECK(create_world(&world, allocator, &config));

    v3 half_extents = {0.5f, 0.3f, 0.2f};
    v3 position = {1.0f, 2.0f, 3.0f};
    CHECK(add_box(&world, position, half_extents, 500.0f, 0xeb9632));
    for (u32 i = 0; i < 3 * config.physics_hz; i++) {
        step_world(&world);
    }
    b3Transform pose = world.props[0].current;
    CHECK_NEAR(pose.p.x, 1.0f, 1e-2);
    CHECK_NEAR(pose.p.y, 2.0f, 1e-2);
    CHECK_NEAR(pose.p.z, get_terrain_height(&world.terrain, pose.p.x, pose.p.y) + 0.2f, 5e-3);
    CHECK(b3Length(b3Body_GetLinearVelocity(world.props[0].body)) < 1e-3f);
    CHECK_NEAR(get_sim_time_ns(&world), 3 * NS_PER_S, 1);
    destroy_world(&world);
}

static u64 run_scenario(Linear_Allocator *allocator)
{
    Sim_Config config = make_small_config();
    static World world;
    CHECK(create_world(&world, allocator, &config));
    for (u32 i = 0; i < 12; i++) {
        v3 half_extents = {0.3f + 0.05f * (f32)(i % 3), 0.3f, 0.25f};
        f32 x = -4.0f + 0.7f * (f32)i;
        f32 y = 0.3f * (f32)(i % 4);
        v3 position = {x, y, get_terrain_height(&world.terrain, x, y) + 1.0f + 0.8f * (f32)i};
        CHECK(add_box(&world, position, half_extents, 400.0f, 0xeb9632));
    }
    for (u32 i = 0; i < 2000; i++) {
        step_world(&world);
    }
    u64 hash = hash_world_state(&world);
    destroy_world(&world);
    return hash;
}

// Same inputs, same bits: the foundation of record/replay and reproducible CI runs.
static void test_determinism(Linear_Allocator *allocator)
{
    u64 first = run_scenario(allocator);
    u64 second = run_scenario(allocator);
    CHECK(first == second);
}

static void write_file(const char *path, const void *data, u64 size)
{
    FILE *file = fopen(path, "wb");
    CHECK(file != NULL);
    if (file) {
        fwrite(data, 1, size, file);
        fclose(file);
    }
}

// A 16-bit PGM: 3 columns by 2 rows, big-endian samples, a comment in the header.
static void test_heightmap(Linear_Allocator *allocator)
{
    const char *tmp = getenv("TMPDIR"); // the Nix build sandbox sets it; /tmp may not exist
    char path[256];
    snprintf(path, sizeof(path), "%s/regolith-test-XXXXXX", tmp && tmp[0] ? tmp : "/tmp");
    i32 fd = mkstemp(path);
    CHECK(fd >= 0);
    close(fd);

    const char header[] = "P5\n# test terrain\n3 2\n65535\n";
    u16 samples[6] = {0, 32768, 65535, 13107, 26214, 39321};
    u8 file[sizeof(header) - 1 + sizeof(samples)];
    memcpy(file, header, sizeof(header) - 1);
    for (u32 i = 0; i < 6; i++) {
        file[sizeof(header) - 1 + i * 2] = (u8)(samples[i] >> 8);
        file[sizeof(header) - 1 + i * 2 + 1] = (u8)(samples[i] & 0xff);
    }
    write_file(path, file, sizeof(file));

    Sim_Config config = get_default_config();
    snprintf(config.terrain_heightmap, sizeof(config.terrain_heightmap), "%s", path);
    config.terrain_height = 10.0f;
    config.terrain_spacing = 2.0f;
    Terrain terrain;
    char error[256] = "";
    CHECK(load_heightmap(&terrain, allocator, &config, error, sizeof(error)));
    CHECK(terrain.cols == 3 && terrain.rows == 2);
    CHECK_NEAR(terrain.origin_x, -2.0, 1e-6); // centred: x from -2 to 2
    CHECK_NEAR(terrain.origin_y, 1.0, 1e-6); // row 0 is the north edge, y = 1
    CHECK_NEAR(terrain.heights[1], 10.0 * 32768.0 / 65535.0, 1e-4);
    CHECK_NEAR(terrain.max_height, 10.0, 1e-4);
    CHECK_NEAR(get_terrain_height(&terrain, 2.0f, 1.0f), 10.0, 1e-4); // north-east sample
    CHECK_NEAR(get_terrain_height(&terrain, -2.0f, -1.0f), 2.0, 1e-4); // south-west: 13107

    // The same heights through the whole world: physics and the loader agree.
    static World world;
    config.terrain_spacing = 2.0f;
    CHECK(create_world(&world, allocator, &config));
    v3 origin = {0.7f, 0.3f, 50.0f}; // inside a cell: the grid's edges are its boundary
    v3 down = {0.0f, 0.0f, -1.0f};
    CHECK_NEAR(50.0f - cast_ray(&world, origin, down, 100.0f),
               get_terrain_height(&world.terrain, 0.7f, 0.3f), 2e-3);
    destroy_world(&world);

    const char truncated[] = "P5 3 2 65535\n\x01\x02";
    write_file(path, truncated, sizeof(truncated) - 1);
    CHECK(!load_heightmap(&terrain, allocator, &config, error, sizeof(error)));
    CHECK(strstr(error, "truncated") != NULL);
    const char ascii[] = "P2 3 2 255\n1 2 3 4 5 6\n";
    write_file(path, ascii, sizeof(ascii) - 1);
    CHECK(!load_heightmap(&terrain, allocator, &config, error, sizeof(error)));
    unlink(path);
}

int main(void)
{
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    test_terrain_matches_physics(&allocator);
    reset_allocator(&allocator);
    test_box_comes_to_rest(&allocator);
    reset_allocator(&allocator);
    test_determinism(&allocator);
    reset_allocator(&allocator);
    test_heightmap(&allocator);
    destroy_allocator(&allocator);
    return report_checks("world");
}
