#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/shared_state.h"
#include "core/world.h"
#include "render/sensor_camera.h"
#include "render/viewer.h"
#include "can/can_bridge.h"
#include "ros/ros_bridge.h"

// Beyond this many steps in one frame the sim is not keeping up. It then runs slower
// than real time rather than spiralling ever further behind.
#define MAX_STEPS_PER_FRAME 64

struct Args {
    bool headless;
    bool print_hash;
    bool dump_config;
    u64 max_steps; // 0 runs until quit
    f64 realtime_factor; // 0 runs as fast as possible
    const char *config_path;
    const char *screenshot_path; // save the final frame here
    const char *urdf_path; // overrides robot.urdf
};

static void print_usage(void)
{
    fprintf(stderr, "usage: regolith [options] [--ros-args ...]\n"
                    "  --config FILE  load settings (see config/sim.yaml)\n"
                    "  --urdf FILE    load this robot (overrides robot.urdf)\n"
                    "  --dump-config  print the effective settings as YAML and exit\n"
                    "  --headless     run without a window\n"
                    "  --rtf X        real-time factor; 0 runs as fast as possible (default 1)\n"
                    "  --steps N      quit after N physics steps\n"
                    "  --hash         print the final state hash (for determinism checks)\n"
                    "  --screenshot F save the final frame as a PNG (with --steps)\n"
                    "Settings apply in order: defaults, --config, then ROS 2 parameters\n"
                    "(--ros-args --params-file / -p) when built with ROS 2.\n");
}

static bool parse_args(i32 argc, char **argv, Args *args)
{
    *args = {.headless = false,
             .print_hash = false,
             .dump_config = false,
             .max_steps = 0,
             .realtime_factor = 1.0,
             .config_path = NULL,
             .screenshot_path = NULL,
             .urdf_path = NULL};
    bool in_ros_args = false;
    for (i32 i = 1; i < argc; i++) {
        const char *arg = argv[i];
        if (strcmp(arg, "--ros-args") == 0) {
            in_ros_args = true;
        } else if (in_ros_args) {
            in_ros_args = strcmp(arg, "--") != 0;
        } else if (strcmp(arg, "--headless") == 0) {
            args->headless = true;
        } else if (strcmp(arg, "--hash") == 0) {
            args->print_hash = true;
        } else if (strcmp(arg, "--dump-config") == 0) {
            args->dump_config = true;
        } else if (strcmp(arg, "--config") == 0 && i + 1 < argc) {
            args->config_path = argv[++i];
        } else if (strcmp(arg, "--urdf") == 0 && i + 1 < argc) {
            args->urdf_path = argv[++i];
        } else if (strcmp(arg, "--steps") == 0 && i + 1 < argc) {
            args->max_steps = strtoull(argv[++i], NULL, 10);
        } else if (strcmp(arg, "--rtf") == 0 && i + 1 < argc) {
            args->realtime_factor = strtod(argv[++i], NULL);
        } else if (strcmp(arg, "--screenshot") == 0 && i + 1 < argc) {
            args->screenshot_path = argv[++i];
        } else {
            if (strcmp(arg, "--help") != 0 && strcmp(arg, "-h") != 0) {
                log_error("unknown argument: %s", arg);
            }
            print_usage();
            return false;
        }
    }
    return true;
}

// Rolling measurements for the overlay and the headless progress log.
struct Pace_Stats {
    u64 window_start_ns;
    u64 window_start_sim_ns;
    u64 window_steps;
    u64 window_step_ns;
    Frame_Stats frame;
};

static void update_pace_stats(Pace_Stats *pace, u64 now_ns, u64 sim_ns)
{
    u64 window_ns = now_ns - pace->window_start_ns;
    if (window_ns < NS_PER_S / 2) {
        return;
    }
    pace->frame.realtime_factor = (f64)(sim_ns - pace->window_start_sim_ns) / (f64)window_ns;
    pace->frame.step_ms =
        pace->window_steps ? (f64)pace->window_step_ns / (f64)pace->window_steps * 1e-6 : 0.0;
    pace->window_start_ns = now_ns;
    pace->window_start_sim_ns = sim_ns;
    pace->window_steps = 0;
    pace->window_step_ns = 0;
}

// Ctrl-C and SIGTERM belong to the app, not to rclcpp, so shutdown works the same with
// or without ROS 2. A second signal a while after the first gives up on a clean
// shutdown. One that follows immediately is a duplicate: Ctrl-C under ros2 launch
// arrives from the terminal and then again from launch, and must not kill the sim.
#define SIGNAL_REPEAT_NS (NS_PER_S / 2)

static Shared_Global_State *signal_shared;
static u64 first_signal_ns;

static void handle_signal(int signal_number)
{
    (void)signal_number;
    u64 now = get_time_ns(); // clock_gettime is async-signal-safe
    if (!is_quit_requested(signal_shared)) {
        first_signal_ns = now;
        request_quit(signal_shared);
    } else if (now - first_signal_ns > SIGNAL_REPEAT_NS) {
        _exit(130);
    }
}

static void install_signal_handlers(Shared_Global_State *shared)
{
    signal_shared = shared;
    struct sigaction action = {};
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
}

// Defaults, then --config, then ROS 2 parameters (a no-op when built without ROS 2).
static bool load_settings(const Args *args, i32 argc, char **argv, Sim_Config *config)
{
    *config = get_default_config();
    char error[256];
    if (args->config_path &&
        !load_config_file(config, args->config_path, NULL, error, sizeof(error))) {
        log_error("config: %s", error);
        return false;
    }
    if (!create_ros_bridge(argc, argv)) {
        return false;
    }
    apply_ros_parameters(config);
    if (args->urdf_path &&
        !set_config_text(config, "robot.urdf", args->urdf_path, error, sizeof(error))) {
        log_error("--urdf: %s", error);
        return false;
    }
    if (!validate_config(config, error, sizeof(error))) {
        log_error("config: %s", error);
        return false;
    }
    return true;
}

// A URDF file named in the settings. Relative mesh paths resolve against its directory.
static bool load_robot_file(World *world, const char *path)
{
    u64 size = 0;
    char *xml = read_file(path, &size);
    if (!xml) {
        log_error("robot: cannot read %s", path);
        return false;
    }
    char directory[256];
    get_directory(path, directory, sizeof(directory));
    char error[256];
    bool loaded = load_robot(world, xml, size, directory, error, sizeof(error));
    if (!loaded) {
        log_error("robot: %s: %s", path, error);
    }
    free(xml);
    return loaded;
}

// The URDF from /robot_description. Blocks until robot_state_publisher (or anything else)
// has published it.
static bool load_robot_from_ros(World *world, Shared_Global_State *shared)
{
    log_info("no robot.urdf set: waiting for /robot_description");
    u64 size = 0;
    char *xml = wait_for_robot_description(shared, &size);
    if (!xml) {
        return false;
    }
    log_info("robot description from /robot_description (%llu bytes)", (unsigned long long)size);
    char error[256];
    bool loaded = load_robot(world, xml, size, "", error, sizeof(error));
    if (!loaded) {
        log_error("robot from /robot_description: %s", error);
    }
    free(xml);
    return loaded;
}

// One physics step, with the commands that have arrived applied first. In keyboard mode CAN
// commands are thrown away, and the time is noted so the viewer can say so.
static void run_step(World *world, Shared_Global_State *shared, u64 *last_ignored_ns)
{
    Actuator_Command command;
    while (pop_ring_buffer(&shared->actuator_commands, &command)) {
        if (world->control_mode == CONTROL_CAN) {
            command_actuator(&world->actuators, &command, world->step_count);
        } else {
            *last_ignored_ns = get_time_ns();
        }
    }
    step_world(world);
    // Sensor output goes to the ROS thread. If nothing is reading, IMU samples are dropped.
    if (world->has_imu) {
        Imu_Sample samples[IMU_MAX_PENDING];
        u32 count = take_imu_samples(&world->imu, samples, IMU_MAX_PENDING);
        for (u32 i = 0; i < count; i++) {
            push_ring_buffer(&shared->imu_samples, &samples[i]);
        }
    }
    const Lidar_Frame *frame = world->has_lidar ? take_lidar_frame(&world->lidar) : NULL;
    if (frame) {
        Lidar_Frame_Header *slot =
            (Lidar_Frame_Header *)get_triple_buffer_write_slot(&shared->lidar_frames);
        slot->start_ns = frame->start_ns;
        slot->count = frame->count;
        memcpy(get_lidar_slot_points(slot), frame->points, frame->count * sizeof(Lidar_Point));
        publish_triple_buffer(&shared->lidar_frames);
    }
    set_sim_time(shared, get_sim_time_ns(world));
}

// Four 10 cm cubes (red, white, green and blue) scattered 1.5 to 5 m from the spawn point,
// at least 0.5 m apart. The layout comes from terrain.seed, so it is the same every run and
// changes with the terrain.
static void drop_starting_boxes(World *world)
{
    const u32 colors[4] = {0xd03030, 0xf0f0f0, 0x30b040, 0x3060e0};
    v3 half_extents = {0.05f, 0.05f, 0.05f};
    Random random = create_random(world->config.terrain_seed, 3);
    f32 placed[4][2];
    for (u32 i = 0; i < 4; i++) {
        f32 x = 0.0f, y = 0.0f;
        for (u32 attempt = 0; attempt < 100; attempt++) {
            f32 angle = 2.0f * PI_F32 * get_random_f32(&random);
            f32 distance = 1.5f + 3.5f * get_random_f32(&random);
            x = world->config.robot_spawn.x + distance * cosf(angle);
            y = world->config.robot_spawn.y + distance * sinf(angle);
            bool clear = true;
            for (u32 j = 0; j < i; j++) {
                f32 dx = x - placed[j][0], dy = y - placed[j][1];
                clear = clear && dx * dx + dy * dy >= 0.5f * 0.5f;
            }
            if (clear) {
                break;
            }
        }
        placed[i][0] = x;
        placed[i][1] = y;
        v3 position = {x, y, get_terrain_height(&world->terrain, x, y) + 0.2f};
        add_box(world, position, half_extents, 400.0f, colors[i]);
    }
}

int main(int argc, char **argv)
{
    u64 start_ns = get_time_ns();
    Args args;
    if (!parse_args(argc, argv, &args)) {
        return 2;
    }
    static Shared_Global_State shared;
    install_signal_handlers(&shared);

    Sim_Config config;
    if (!load_settings(&args, argc, argv, &config)) {
        destroy_ros_bridge();
        return 1;
    }
    if (args.dump_config) {
        static char text[16384];
        u32 length = format_config(&config, text, sizeof(text));
        fwrite(text, 1, min(length, (u32)sizeof(text) - 1), stdout);
        destroy_ros_bridge();
        return 0;
    }

    Linear_Allocator allocator;
    static World world;
    if (!create_allocator(&allocator, 512 * MEGABYTE) ||
        !create_shared_state(&shared, &allocator, &config) ||
        !create_world(&world, &allocator, &config)) {
        destroy_ros_bridge();
        return 1;
    }
    log_info("world ready: %ux%u terrain samples, %.0f ms after start", world.terrain.cols,
             world.terrain.rows, (f64)(get_time_ns() - start_ns) * 1e-6);
    drop_starting_boxes(&world);
    bool robot_loaded = config.robot_urdf[0] ? load_robot_file(&world, config.robot_urdf)
                                             : load_robot_from_ros(&world, &shared);
    if (!robot_loaded) {
        destroy_world(&world);
        destroy_ros_bridge();
        return is_quit_requested(&shared) ? 0 : 1;
    }

    Viewer viewer;
    if (!args.headless && !create_viewer(&viewer, &world, &allocator)) {
        destroy_world(&world);
        destroy_ros_bridge();
        return 1;
    }
    // The camera renders with the viewer's GL context, so it needs the window.
    static Sensor_Camera sensor_camera;
    bool camera_on = config.camera.enabled && !args.headless;
    if (config.camera.enabled && args.headless) {
        log_warning("camera: off in --headless (it renders with the window's GL context)");
    }
    if (camera_on) {
        char error[256];
        if (!create_sensor_camera(&sensor_camera, &config.camera, &world, error, sizeof(error))) {
            log_error("%s", error);
            destroy_viewer(&viewer);
            destroy_world(&world);
            destroy_ros_bridge();
            return 1;
        }
        viewer.camera_preview = &sensor_camera.color_target.texture;
        log_info("camera %s on %s_link: %ux%u at %.0f Hz", config.camera.name, config.camera.name,
                 config.camera.resolution[0], config.camera.resolution[1], (f64)config.camera.rate);
    }
    if (!start_can_thread(&shared, &config, &world.robot, &world.actuators) ||
        !start_ros_thread(&shared, &config)) {
        request_quit(&shared);
    }
    // CAN has control when it is there to have it; otherwise the keyboard does.
    set_control_mode(&world, is_can_running() || args.headless ? CONTROL_CAN : CONTROL_KEYBOARD);
    log_info("control: %s%s", world.control_mode == CONTROL_CAN ? "CAN" : "keyboard",
             args.headless ? "" : " (M switches)");

    f64 step_seconds = 1.0 / (f64)config.physics_hz;
    f64 accumulator = 0.0;
    bool paused = false;
    bool first_frame = true;
    f32 teleop_drive = 0.0f; // -1..1, from the viewer's arrow keys
    f32 teleop_turn = 0.0f;
    u64 last_ignored_ns = 0; // when a CAN command last arrived in keyboard mode
    u64 previous_ns = get_time_ns();
    u64 last_log_ns = previous_ns;
    Pace_Stats pace = {.window_start_ns = previous_ns};

    while (!is_quit_requested(&shared)) {
        u64 now_ns = get_time_ns();
        f64 elapsed = (f64)(now_ns - previous_ns) * 1e-9;
        previous_ns = now_ns;

        u32 steps = 0;
        if (!paused && args.realtime_factor <= 0.0) {
            steps = MAX_STEPS_PER_FRAME;
        } else if (!paused) {
            accumulator += min(elapsed, 0.25) * args.realtime_factor;
            steps = (u32)(accumulator / step_seconds);
            accumulator -= (f64)steps * step_seconds;
            if (steps > MAX_STEPS_PER_FRAME) {
                steps = MAX_STEPS_PER_FRAME;
                accumulator = 0.0;
            }
        }

        // Exactly one source drives: in keyboard mode the arrow keys, every frame (no key
        // means stop); in CAN mode the actuators, inside each step. Commands land between
        // steps.
        if (world.has_robot && world.control_mode == CONTROL_KEYBOARD) {
            steer_robot(&world.robot, teleop_drive, teleop_turn, config.teleop_speed,
                        config.teleop_turn_rate);
        }
        const Status_Light *light = (const Status_Light *)read_triple_buffer(&shared.status_light);
        if (light) {
            world.status_light = *light;
        }

        for (u32 i = 0; i < steps; i++) {
            u64 step_start_ns = get_time_ns();
            run_step(&world, &shared, &last_ignored_ns);
            pace.window_step_ns += get_time_ns() - step_start_ns;
            pace.window_steps++;
            if (args.max_steps && world.step_count >= args.max_steps) {
                request_quit(&shared);
                break;
            }
        }
        update_pace_stats(&pace, get_time_ns(), get_sim_time_ns(&world));
        pace.frame.paused = paused;
        pace.frame.can_ignored = get_time_ns() - last_ignored_ns < NS_PER_S && last_ignored_ns != 0;
        pace.frame.can_running = is_can_running();

        if (!args.headless) {
            if (args.screenshot_path && is_quit_requested(&shared)) {
                viewer.screenshot_path = args.screenshot_path;
            }
            if (camera_on && is_sensor_camera_due(&sensor_camera, get_sim_time_ns(&world))) {
                Camera_Frame_Header *slot =
                    (Camera_Frame_Header *)get_triple_buffer_write_slot(&shared.camera_frames);
                render_sensor_camera(&sensor_camera, &viewer, &world, slot);
                publish_triple_buffer(&shared.camera_frames);
            }
            f32 alpha = paused ? 1.0f : (f32)(accumulator / step_seconds);
            Viewer_Actions actions = draw_frame(&viewer, &world, min(alpha, 1.0f), &pace.frame);
            if (first_frame) {
                log_info("first frame %.0f ms after start", (f64)(get_time_ns() - start_ns) * 1e-6);
                first_frame = false;
            }
            if (actions.quit) {
                request_quit(&shared);
            }
            teleop_drive = actions.drive;
            teleop_turn = actions.turn;
            if (actions.drop_box) {
                v3 half_extents = {0.4f, 0.4f, 0.4f};
                add_box(&world, actions.drop_position, half_extents, 400.0f, 0xeb9632);
            }
            if (actions.toggle_control && world.has_robot) {
                Control_Mode mode =
                    world.control_mode == CONTROL_CAN ? CONTROL_KEYBOARD : CONTROL_CAN;
                set_control_mode(&world, mode);
                log_info("control: %s", mode == CONTROL_CAN ? "CAN" : "keyboard");
            }
            if (actions.reset_robot && world.has_robot) {
                reset_robot(&world);
                log_info("robot reset to its spawn pose");
            }
            if (actions.toggle_pause) {
                paused = !paused;
            }
            if (paused && actions.single_step) {
                run_step(&world, &shared, &last_ignored_ns);
            }
        } else {
            if (now_ns - last_log_ns > 5 * NS_PER_S) {
                log_info("sim %.1f s, rtf %.2fx, step %.3f ms", (f64)get_sim_time_ns(&world) * 1e-9,
                         pace.frame.realtime_factor, pace.frame.step_ms);
                last_log_ns = now_ns;
            }
            if (args.realtime_factor > 0.0) {
                f64 wait = (step_seconds - accumulator) / args.realtime_factor;
                sleep_ns((u64)(max(wait, 0.0) * 1e9));
            }
        }
    }

    log_info("stopped at sim %.3f s after %llu steps", (f64)get_sim_time_ns(&world) * 1e-9,
             (unsigned long long)world.step_count);
    if (args.print_hash) {
        printf("%016llx\n", (unsigned long long)hash_world_state(&world));
    }
    stop_can_thread();
    destroy_ros_bridge();
    if (camera_on) {
        destroy_sensor_camera(&sensor_camera);
    }
    if (!args.headless) {
        destroy_viewer(&viewer);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
    return 0;
}
