#include <string.h>

#include "core/config.h"
#include "tests/check.h"

static void test_schema(void)
{
    Sim_Config config = get_default_config();
    char error[256];
    CHECK(validate_config(&config, error, sizeof(error)));
    for (u32 i = 0; i < config_field_count; i++) {
        const Config_Field *field = &config_fields[i];
        // Keys are "section.name" or "section.subsection.name", with no empty parts;
        // format_config and the YAML nesting rely on it.
        const char *dot = strchr(field->key, '.');
        CHECK(dot && dot != field->key && strstr(field->key, "..") == NULL);
        CHECK(field->key[strlen(field->key) - 1] != '.');
        CHECK(field->count >= 1 && field->count <= CONFIG_MAX_COUNT);
        CHECK(find_config_field(field->key) == field);
        // Every default must pass its own checks.
        if (field->type == CONFIG_STRING) {
            CHECK(field->count == 1 && field->text_default != NULL);
            CHECK(set_config_text(&config, field->key, field->text_default, error, sizeof(error)));
        } else {
            CHECK(set_config(&config, field->key, field->defaults, field->count, error,
                             sizeof(error)));
        }
    }
    // Sections and subsections must be contiguous, or --dump-config would repeat a header.
    for (u32 i = 1; i < config_field_count; i++) {
        const char *key = config_fields[i].key;
        for (const char *dot = strchr(key, '.'); dot; dot = strchr(dot + 1, '.')) {
            u32 length = (u32)(dot - key) + 1; // the prefix, with its dot
            bool same_as_previous = strncmp(config_fields[i - 1].key, key, length) == 0;
            for (u32 j = 0; j + 1 < i && !same_as_previous; j++) {
                CHECK(strncmp(config_fields[j].key, key, length) != 0);
            }
        }
    }
}

// Settings that are fine alone but not together.
static void test_validate(void)
{
    char error[256];
    Sim_Config config = get_default_config();
    config.lidar.range[0] = 50.0f;
    CHECK(!validate_config(&config, error, sizeof(error)));
    CHECK(strstr(error, "lidar.range_m") != NULL);
    config = get_default_config();
    config.imu.rate = 1000.0f;
    CHECK(!validate_config(&config, error, sizeof(error)));
    CHECK(strstr(error, "imu.rate_hz") != NULL);
    config = get_default_config();
    config.terrain_heightmap[0] = 'm'; // a heightmap skips the size check, not the rest
    config.camera.depth_range[1] = 0.05f;
    CHECK(!validate_config(&config, error, sizeof(error)));
    CHECK(strstr(error, "camera.depth.range_m") != NULL);
}

static void test_text(void)
{
    Sim_Config config = get_default_config();
    char error[256];
    CHECK(config.robot_urdf[0] == 0);
    CHECK(set_config_text(&config, "robot.urdf", "/tmp/rover.urdf", error, sizeof(error)));
    CHECK(strcmp(config.robot_urdf, "/tmp/rover.urdf") == 0);
    CHECK(!set_config_text(&config, "world.physics_hz", "fast", error, sizeof(error)));
    CHECK(strstr(error, "takes numbers") != NULL);
    f64 number = 1.0;
    CHECK(!set_config(&config, "robot.urdf", &number, 1, error, sizeof(error)));
    CHECK(strstr(error, "takes text") != NULL);
    char too_long[CONFIG_STRING_SIZE + 1];
    memset(too_long, 'a', sizeof(too_long) - 1);
    too_long[sizeof(too_long) - 1] = 0;
    CHECK(!set_config_text(&config, "robot.urdf", too_long, error, sizeof(error)));
    CHECK(strcmp(config.robot_urdf, "/tmp/rover.urdf") == 0);

    const char *file = "robot:\n"
                       "  urdf: /data/rover#2.urdf   # a # inside a word is kept\n"
                       "terrain:\n"
                       "  heightmap: \"maps/\\\"site\\\" #1.pgm\"  # quotes and escapes\n";
    CHECK(load_config_text(&config, file, "text", NULL, error, sizeof(error)));
    CHECK(strcmp(config.robot_urdf, "/data/rover#2.urdf") == 0);
    CHECK(strcmp(config.terrain_heightmap, "maps/\"site\" #1.pgm") == 0);

    CHECK(load_config_text(&config, "robot:\n  urdf: 'it''s.urdf'\n", "text", NULL, error,
                           sizeof(error)));
    CHECK(strcmp(config.robot_urdf, "it's.urdf") == 0);

    CHECK(
        !load_config_text(&config, "robot:\n  urdf: \"open\n", "bad", NULL, error, sizeof(error)));
    CHECK(strstr(error, "bad:2: robot.urdf: expected text") != NULL);
    CHECK(
        !load_config_text(&config, "robot:\n  urdf: [1, 2]\n", "bad", NULL, error, sizeof(error)));
}

static void test_set(void)
{
    Sim_Config config = get_default_config();
    char error[256];
    f64 hz = 500.0;
    CHECK(set_config(&config, "world.physics_hz", &hz, 1, error, sizeof(error)));
    CHECK(config.physics_hz == 500);

    CHECK(!set_config(&config, "world.physics_hzz", &hz, 1, error, sizeof(error)));
    CHECK(strstr(error, "not a setting") != NULL);

    f64 too_fast = 20000.0;
    CHECK(!set_config(&config, "world.physics_hz", &too_fast, 1, error, sizeof(error)));
    CHECK(strstr(error, "within") != NULL);

    f64 fraction = 240.5;
    CHECK(!set_config(&config, "world.physics_hz", &fraction, 1, error, sizeof(error)));
    CHECK(strstr(error, "whole number") != NULL);

    f64 gravity[2] = {0.0, -9.81};
    CHECK(!set_config(&config, "world.gravity", gravity, 2, error, sizeof(error)));
    CHECK(strstr(error, "takes 3 values") != NULL);

    // A failed set changes nothing, not even the values that were in range.
    f64 bad_gravity[3] = {0.0, 0.0, 1000.0};
    CHECK(!set_config(&config, "world.gravity", bad_gravity, 3, error, sizeof(error)));
    CHECK(config.gravity.z == -9.81f);
    CHECK(config.physics_hz == 500);
}

static void test_load_text(void)
{
    const char *ros_file = "# a ROS 2 parameters file\n"
                           "regolith:\n"
                           "  ros__parameters:\n"
                           "    use_sim_time: true   # a standard ROS parameter, skipped\n"
                           "    world:\n"
                           "      physics_hz: 500\n"
                           "      gravity: [0, 0, -1.62]   # integers are fine for floats\n"
                           "\n"
                           "    terrain:\n"
                           "      relief_m: 1\n"
                           "      bogus: 3\n";
    Sim_Config config = get_default_config();
    u32 unknown = 99;
    char error[256];
    CHECK(load_config_text(&config, ros_file, "ros", &unknown, error, sizeof(error)));
    CHECK(config.physics_hz == 500);
    CHECK(config.gravity.z == -1.62f);
    CHECK(config.terrain_relief == 1.0f);
    CHECK(!config.use_sim_time); // the default; ROS's own use_sim_time is a different setting
    CHECK(unknown == 1);

    const char *flags = "lidar:\n  enabled: true\nimu:\n  acceleration_in_g: False\n"
                        "ros:\n  use_sim_time: true\n";
    CHECK(load_config_text(&config, flags, "flags", &unknown, error, sizeof(error)));
    CHECK(config.lidar.enabled && !config.imu.acceleration_in_g && config.use_sim_time);

    const char *plain_file = "world:\n  substeps: 8\n";
    CHECK(load_config_text(&config, plain_file, "plain", &unknown, error, sizeof(error)));
    CHECK(config.substeps == 8);
    CHECK(unknown == 0);

    struct Bad_File {
        const char *text;
        const char *expected;
    };
    Bad_File bad_files[] = {
        {"world:\n\tsubsteps: 8\n", "bad:2: indent with spaces"},
        {"world:\n  gravity:\n    - 0\n", "bad:3: write lists inline"},
        {"world:\n  gravity: [0, 0,\n    -9.81]\n", "bad:2: world.gravity: expected a number"},
        {"world:\n  physics_hz: fast\n", "bad:2: world.physics_hz: expected a number"},
        {"# ok\nworld:\n  physics_hz: 5\n", "bad:3: world.physics_hz must be within"},
        {"just some words\n", "bad:1: expected \"key: value\""},
        {"lidar:\n  enabled: 1\n", "bad:2: lidar.enabled: expected true or false"},
        {"camera:\n  color:\n    enabled: yes please\n",
         "bad:3: camera.color.enabled: expected true or false"},
    };
    for (u32 i = 0; i < ARRAY_COUNT(bad_files); i++) {
        Sim_Config scratch = get_default_config();
        bool loaded =
            load_config_text(&scratch, bad_files[i].text, "bad", NULL, error, sizeof(error));
        CHECK(!loaded);
        if (!loaded && strstr(error, bad_files[i].expected) == NULL) {
            fprintf(stderr, "  got \"%s\", expected \"%s\"\n", error, bad_files[i].expected);
            check_failures++;
        }
    }
}

// --dump-config output must load back to exactly the same settings.
static void test_round_trip(void)
{
    Sim_Config original = get_default_config();
    original.gravity.z = -1.62f;
    original.physics_hz = 1000;
    original.terrain_spacing = 0.1f;
    original.terrain_seed = 4000000000u;
    snprintf(original.robot_urdf, sizeof(original.robot_urdf), "/home/nova/a \"quoted\" #path");
    original.lidar.enabled = true;
    original.imu.acceleration_in_g = false;
    original.camera.color.resolution[0] = 1280; // nested sections round-trip too
    original.camera.depth.enabled = true;
    char error_text[256];
    CHECK(set_config_text(&original, "camera.depth.topic", "/d415/depth", error_text,
                          sizeof(error_text)));

    static char text[16384];
    u32 length = format_config(&original, text, sizeof(text));
    CHECK(length < sizeof(text));

    Sim_Config loaded = get_default_config();
    u32 unknown = 99;
    char error[256];
    CHECK(load_config_text(&loaded, text, "dump", &unknown, error, sizeof(error)));
    CHECK(unknown == 0);
    CHECK(memcmp(&loaded, &original, sizeof(Sim_Config)) == 0);

    // A buffer that is too small is truncated, not overrun, and reports what it needed.
    char small[32];
    CHECK(format_config(&original, small, sizeof(small)) == length);
    CHECK(strlen(small) == sizeof(small) - 1);
}

// The shipped config/sim.yaml must stay in step with the schema.
static void test_shipped_file(const char *path)
{
    Sim_Config config = get_default_config();
    u32 unknown = 99;
    char error[256];
    bool loaded = load_config_file(&config, path, &unknown, error, sizeof(error));
    CHECK(loaded);
    if (!loaded) {
        fprintf(stderr, "  %s\n", error);
    }
    CHECK(unknown == 0);
    CHECK(validate_config(&config, error, sizeof(error)));
}

int main(int argc, char **argv)
{
    test_schema();
    test_set();
    test_text();
    test_validate();
    test_load_text();
    test_round_trip();
    if (argc > 1) {
        test_shipped_file(argv[1]);
    }
    return report_checks("config");
}
