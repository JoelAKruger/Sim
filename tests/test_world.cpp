#include <math.h>
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
    config.boulder_count = 0; // these tests are about the bare ground
    config.soil_enabled = false; // and the rigid ground, apart from test_soil
    return config;
}

// The terrain the renderer and sensors read must be the surface physics collides with.
// Also pins down the ground's orientation: row 0 is north (+y), heights are +z.
static void test_terrain_matches_physics(Linear_Allocator *allocator)
{
    Sim_Config config = make_small_config();
    static World world;
    CHECK(create_world(&world, allocator, &config));
    step_world(&world); // the physics sets up its collision shapes on the first step

    v3 down = {0.0f, 0.0f, -1.0f};
    u32 state = 12345;
    for (u32 i = 0; i < 200; i++) {
        state = state * 1664525u + 1013904223u;
        f32 x = ((f32)(state >> 8) / 16777216.0f - 0.5f) * 38.0f;
        state = state * 1664525u + 1013904223u;
        f32 y = ((f32)(state >> 8) / 16777216.0f - 0.5f) * 28.0f;
        v3 origin = {x, y, world.terrain.max_height + 10.0f};
        f32 height = get_terrain_height(&world.terrain, x, y);
        f32 distance = cast_ray(&world, origin, down, 100.0f);
        CHECK(distance > 0.0f);
        CHECK_NEAR(origin.z - distance, height, 1e-3);
        f32 physics_distance = cast_physics_ray(world.physics, origin, down, 100.0f);
        CHECK(physics_distance > 0.0f);
        CHECK_NEAR(origin.z - physics_distance, height, 1e-3);
    }
    destroy_world(&world);
}

// On flat ground a box dropped from 3 m comes to rest exactly one half-extent above it,
// near where it fell.
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
    Pose settled = {};
    for (u32 i = 0; i < 3 * config.physics_hz; i++) {
        step_world(&world);
        if (i + 1 == 5 * config.physics_hz / 2) {
            settled = world.props[0].current;
        }
    }
    Pose pose = world.props[0].current;
    CHECK_NEAR(pose.p.x, 1.0f, 0.1);
    CHECK_NEAR(pose.p.y, 2.0f, 0.1);
    CHECK_NEAR(pose.p.z, get_terrain_height(&world.terrain, pose.p.x, pose.p.y) + 0.2f, 5e-3);
    // Still: it hasn't moved in the last half second. The solver leaves a few mm/s of noise
    // in a resting body's velocity, which never adds up to motion.
    CHECK(get_length(pose.p - settled.p) < 5e-4f);
    CHECK(get_length(get_body_velocity(world.physics, world.props[0].body)) < 1e-2f);
    CHECK_NEAR(get_sim_time_ns(&world), 3 * NS_PER_S, 1);
    destroy_world(&world);
}

// A heavy box on the soil sinks into it, as far as Bekker's law says, and leaves a dent
// that the ground height and the sensors' rays both see.
static void test_soil(Linear_Allocator *allocator)
{
    Sim_Config config = make_small_config();
    config.terrain_relief = 0.0f;
    config.crater_count = 0;
    config.soil_enabled = true;
    config.soil_size[0] = config.soil_size[1] = 6.0f;
    config.soil_bulldozing = false;
    static World world;
    CHECK(create_world(&world, allocator, &config));
    CHECK(world.has_soil);
    f32 x = 1.0f, y = 1.0f;
    f32 surface = get_world_ground_height(&world, x, y);
    CHECK_NEAR(surface, get_terrain_height(&world.terrain, x, y), 1e-4);

    // 128 kg on 0.16 m²: 7.8 kPa. Set down on the soil, it sinks at least as far as Bekker's
    // law, (p / kphi)^(1/n), and no further than if the soil were undamped, where the
    // weight's work equals the work of pressing the soil in: ((n + 1) p / kphi)^(1/n).
    // Plastic soil keeps the deepest point.
    v3 half_extents = {0.2f, 0.2f, 0.2f};
    CHECK(add_box(&world, v3{x, y, surface + 0.201f}, half_extents, 2000.0f, 0xeb9632));
    for (u32 i = 0; i < 2 * config.physics_hz; i++) {
        step_world(&world);
    }
    Pose pose = world.props[0].current;
    f32 pressure = 128.0f * -config.gravity.z / 0.16f;
    f32 n = config.soil_bekker_n;
    f32 least = powf(pressure / config.soil_bekker_kphi, 1.0f / n);
    f32 most = powf((n + 1.0f) * pressure / config.soil_bekker_kphi, 1.0f / n);
    f32 sinkage = surface + 0.2f - pose.p.z;
    printf("  soil: sank %.1f mm (Bekker: %.1f to %.1f mm)\n", (f64)(sinkage * 1000.0f),
           (f64)(least * 1000.0f), (f64)(most * 1000.0f));
    CHECK(sinkage > 0.95f * least && sinkage < most);
    CHECK(world.soil.changes > 0);
    f32 dent = get_world_ground_height(&world, x, y);
    CHECK_NEAR(dent, pose.p.z - 0.2f, 2e-3);

    // Just beside the box the rays see the soil where the ground height says it is.
    for (u32 i = 0; i < 8; i++) {
        f32 angle = (f32)i * 0.25f * PI_F32;
        f32 px = x + 0.3f * cosf(angle), py = y + 0.3f * sinf(angle);
        v3 origin = {px, py, surface + 2.0f};
        f32 distance = cast_ray(&world, origin, v3{0.0f, 0.0f, -1.0f}, 10.0f);
        CHECK_NEAR(origin.z - distance, get_world_ground_height(&world, px, py), 1e-3);
    }
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
    config.soil_enabled = false; // a 3x2 map has no room for it
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

// Boulders: clear of the spawn, the same for the same seed, solid to rays, and drawn with
// outward-facing triangles.
static u64 hash_boulders(Linear_Allocator *allocator, u32 seed, u32 count, u32 *placed)
{
    Sim_Config config = make_small_config();
    config.terrain_size[0] = config.terrain_size[1] = 60.0f;
    config.terrain_seed = seed;
    config.boulder_count = count;
    static World world;
    u64 hash = HASH_SEED;
    CHECK(create_world(&world, allocator, &config));
    const Boulder_Field *field = &world.boulders;
    *placed = field->count;
    for (u32 i = 0; i < field->count; i++) {
        const Boulder *boulder = &field->boulders[i];
        hash = hash_bytes(hash, boulder, sizeof(Boulder));
        f32 dx = boulder->center.x - config.robot_spawn.x;
        f32 dy = boulder->center.y - config.robot_spawn.y;
        CHECK(sqrtf(dx * dx + dy * dy) >= BOULDER_SPAWN_CLEARANCE);
        CHECK(boulder->radius <= 0.5f * config.boulder_size[1] * 1.2f * 1.25f + 1e-4f);
    }
    // Every rock's triangles face outwards (they're checked against their own rock).
    bool outward = true;
    for (u32 i = 0; i < field->count; i++) {
        const Boulder *boulder = &field->boulders[i];
        for (u32 v = boulder->first_vertex; v < boulder->first_vertex + boulder->vertex_count;
             v += 3) {
            v3 middle = (1.0f / 3.0f) *
                        (field->vertices[v] + field->vertices[v + 1] + field->vertices[v + 2]);
            outward = outward && dot(field->normals[v], middle - boulder->center) > 0.0f;
        }
    }
    CHECK(outward);
    if (field->count > 0) {
        // Straight down onto the biggest rock's centre: it is hit well above the ground.
        const Boulder *biggest = &field->boulders[0];
        for (u32 i = 1; i < field->count; i++) {
            if (field->boulders[i].radius > biggest->radius) {
                biggest = &field->boulders[i];
            }
        }
        v3 origin = {biggest->center.x, biggest->center.y, biggest->center.z + 5.0f};
        f32 distance = cast_ray(&world, origin, v3{0.0f, 0.0f, -1.0f}, 20.0f);
        f32 ground = get_terrain_height(&world.terrain, origin.x, origin.y);
        CHECK(distance > 0.0f && origin.z - distance > ground + 0.05f);
    }
    destroy_world(&world);
    reset_allocator(allocator);
    return hash;
}

static void test_boulders(Linear_Allocator *allocator)
{
    u32 placed_a, placed_b, placed_c, none;
    u64 a = hash_boulders(allocator, 7, 80, &placed_a);
    u64 b = hash_boulders(allocator, 7, 80, &placed_b);
    u64 c = hash_boulders(allocator, 8, 80, &placed_c);
    hash_boulders(allocator, 7, 0, &none);
    printf("  boulders: %u of 80 placed\n", placed_a);
    CHECK(placed_a >= 70 && placed_a <= 80);
    CHECK(a == b && placed_a == placed_b);
    CHECK(a != c);
    CHECK(none == 0);
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
    reset_allocator(&allocator);
    test_boulders(&allocator);
    reset_allocator(&allocator);
    test_soil(&allocator);
    destroy_allocator(&allocator);
    return report_checks("world");
}
