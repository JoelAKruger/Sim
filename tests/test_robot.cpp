#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "can/blcmd.h"
#include "core/math.h"
#include "core/world.h"
#include "tests/check.h"

static const char *banksia_path;

static Sim_Config make_flat_config(f32 gravity)
{
    Sim_Config config = get_default_config();
    config.terrain_size[0] = config.terrain_size[1] = 40.0f;
    config.terrain_relief = 0.0f;
    config.crater_count = 0;
    config.gravity.z = gravity;
    config.soil_enabled = false; // rigid ground; test_soil_drive has the soil
    return config;
}

static bool load_banksia(World *world)
{
    u64 size = 0;
    char *xml = read_file(banksia_path, &size);
    CHECK(xml != NULL);
    if (!xml) {
        return false;
    }
    char error[256];
    bool loaded = load_robot(world, xml, size, "", error, sizeof(error));
    CHECK(loaded);
    if (!loaded) {
        fprintf(stderr, "  %s\n", error);
    }
    free(xml);
    return loaded;
}

static void run_steps(World *world, f32 seconds)
{
    u32 count = (u32)(seconds * (f32)world->config.physics_hz);
    for (u32 i = 0; i < count; i++) {
        step_world(world);
    }
}

static const Robot_Body *get_body_of_link(const Robot *robot, const char *link)
{
    return &robot->bodies[robot->link_body[find_urdf_link(&robot->model, link)]];
}

static f32 get_uprightness(const Robot *robot)
{
    Quat q = get_body_of_link(robot, "base_link")->current.q;
    return rotate_vector(q, v3{0.0f, 0.0f, 1.0f}).z;
}

static f32 get_max_body_speed(const Robot *robot)
{
    f32 speed = 0.0f;
    for (u32 b = 0; b < robot->body_count; b++) {
        speed = max(speed,
                    get_length(get_body_velocity(robot->physics, robot->bodies[b].physics_body)));
    }
    return speed;
}

// The largest gap any joint has opened, and which joint (a ball is named by its first).
static f32 get_max_joint_separation(const Robot *robot, const char **worst = NULL)
{
    f32 separation = 0.0f;
    const char *name = "";
    for (u32 j = 0; j < robot->joint_count; j++) {
        i32 id = robot->joints[j].ball >= 0 ? (i32)robot->balls[robot->joints[j].ball].physics_joint
                                            : robot->joints[j].physics_joint;
        if (id >= 0 && get_joint_separation(robot->physics, (u32)id) > separation) {
            separation = get_joint_separation(robot->physics, (u32)id);
            name = robot->model.joints[robot->joints[j].urdf_joint].name;
        }
    }
    if (worst) {
        *worst = name;
    }
    return separation;
}

static void test_structure(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        const Robot *robot = &world.robot;
        // chassis (with base_link and gps), 2 legs (each welded to its diff end), 4 ankles,
        // 4 wheels, the diff bar and 2 diff links; the 8 ball dummies are gone.
        CHECK(robot->body_count == 14);
        CHECK(robot->ball_count == 4);
        CHECK(robot->joint_count == 23);
        CHECK(robot->wheel_count == 4);
        CHECK(robot->link_body[find_urdf_link(&robot->model, "tl_ball_link_x")] == -1);
        CHECK(robot->link_body[find_urdf_link(&robot->model, "left_diffend_dummy")] ==
              robot->link_body[find_urdf_link(&robot->model, "left_leg")]);
        f32 mass = 0.0f;
        for (u32 b = 0; b < robot->body_count; b++) {
            mass += robot->bodies[b].mass;
        }
        CHECK_NEAR(mass, 24.42288, 1e-3); // every URDF mass except the 8 x 10 mg dummies
        for (u32 w = 0; w < robot->wheel_count; w++) {
            CHECK_NEAR(robot->wheels[w].radius, 0.157, 1e-6);
        }
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

// Dropped onto flat ground, the rover comes to rest upright with every joint holding.
static void test_settles(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        run_steps(&world, 4.0f);
        const Robot *robot = &world.robot;
        // base_link sits at the ground under the chassis, like a base_footprint.
        f32 base = get_body_of_link(robot, "base_link")->current.p.z;
        f32 chassis = get_link_pose(robot, (u32)find_urdf_link(&robot->model, "chassis"), 1.0f).p.z;
        const char *worst;
        f32 gap = get_max_joint_separation(robot, &worst);
        printf("  settled: chassis %.3f m up (base_link %.3f), upright %.4f, fastest body "
               "%.4f m/s, worst joint gap %.2f mm (%s)\n",
               (f64)chassis, (f64)base, (f64)get_uprightness(robot), (f64)get_max_body_speed(robot),
               (f64)gap * 1000.0, worst);
        CHECK(get_uprightness(robot) > 0.99f);
        CHECK(get_max_body_speed(robot) < 0.01f);
        CHECK(gap < 0.002f);
        CHECK_NEAR(base, 0.0, 0.03);
        CHECK_NEAR(chassis, 0.434, 0.03); // the URDF's ride height
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

// The diff bar ties the legs together: pitch one up and the other pitches down.
static void test_diff_bar_couples(void)
{
    Sim_Config config = make_flat_config(0.0f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        Robot *robot = &world.robot;
        i32 left = find_robot_joint(robot, "chassis_to_left_leg");
        i32 right = find_robot_joint(robot, "chassis_to_right_leg");
        const Robot_Joint *left_joint = &robot->joints[left];
        const Robot_Joint *right_joint = &robot->joints[right];
        u32 chassis = robot->bodies[left_joint->body_a].physics_body;
        u32 left_leg = robot->bodies[left_joint->body_b].physics_body;
        // Twist the left leg against the chassis about the leg's own pivot axis.
        for (u32 i = 0; i < 120; i++) {
            Quat frame =
                multiply_quats(robot->bodies[left_joint->body_a].current.q, left_joint->frame_a.q);
            v3 torque = 2.0f * rotate_vector(frame, v3{0.0f, 0.0f, 1.0f});
            apply_body_torque(world.physics, left_leg, torque);
            apply_body_torque(world.physics, chassis, -torque);
            step_world(&world);
        }
        // Compare pitch about the chassis's lateral axis, whatever each joint's axis convention.
        v3 left_axis = rotate_vector(left_joint->frame_a.q, v3{0.0f, 0.0f, 1.0f});
        v3 right_axis = rotate_vector(right_joint->frame_a.q, v3{0.0f, 0.0f, 1.0f});
        f32 left_pitch = left_joint->position * left_axis.y;
        f32 right_pitch = right_joint->position * right_axis.y;
        printf("  diff bar: left leg pitched %.3f rad, right %.3f rad, worst joint gap %.2f mm\n",
               (f64)left_pitch, (f64)right_pitch, (f64)get_max_joint_separation(robot) * 1000.0);
        CHECK(absolute(left_pitch) > 0.02f);
        CHECK(left_pitch * right_pitch < 0.0f);
        CHECK(absolute(right_pitch / left_pitch) > 0.5f &&
              absolute(right_pitch / left_pitch) < 2.0f);
        CHECK(get_max_joint_separation(robot) < 0.002f);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

static f32 wrap_turn(f32 angle)
{
    return angle - 2.0f * PI_F32 * floorf((angle + PI_F32) / (2.0f * PI_F32));
}

static f32 get_yaw(Quat q)
{
    v3 forward = rotate_vector(q, v3{1.0f, 0.0f, 0.0f});
    return atan2f(forward.y, forward.x);
}

// Keyboard-style skid steering moves it forward, then turns it on the spot.
static void test_drives(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        Robot *robot = &world.robot;
        run_steps(&world, 1.0f);
        v3 start = get_body_of_link(robot, "base_link")->current.p;
        set_control_mode(&world, CONTROL_KEYBOARD);
        drive_robot(robot, 0.4f, 0.0f);
        run_steps(&world, 3.0f);
        v3 moved = get_body_of_link(robot, "base_link")->current.p - start;
        printf("  drive: 3 s at 0.4 m/s moved %.3f m forward, %.3f m sideways\n", (f64)moved.x,
               (f64)moved.y);
        CHECK(moved.x > 0.8f && moved.x < 1.4f);
        CHECK(absolute(moved.y) < 0.2f);

        f32 yaw_start = get_yaw(get_body_of_link(robot, "base_link")->current.q);
        for (u32 i = 0; i < 2 * world.config.physics_hz; i++) {
            drive_robot(robot, 0.0f, 0.5f); // every step, as the keyboard does every frame
            step_world(&world);
        }
        f32 turned =
            wrap_turn(get_yaw(get_body_of_link(robot, "base_link")->current.q) - yaw_start);
        v3 drift = get_body_of_link(robot, "base_link")->current.p - start;
        printf("  turn: 2 s at 0.5 rad/s turned %.3f rad\n", (f64)turned);
        // The pivots swing first, then it spins on the spot.
        CHECK(turned > 0.7f && turned < 1.1f);
        CHECK(get_uprightness(robot) > 0.98f);
        (void)drift;
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

// The three angles of a ball joint come back exactly, including a reversed third axis.
static void test_ball_angles(void)
{
    const char *xml =
        "<robot name='ball'>"
        "  <link name='base'><inertial><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' "
        "ixy='0' ixz='0' iyz='0'/></inertial>"
        "    <collision><geometry><box size='0.2 0.2 0.2'/></geometry></collision></link>"
        "  <link name='d1'><inertial><mass value='0.00001'/><inertia ixx='0.01' iyy='0.01' "
        "izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial></link>"
        "  <link name='d2'/>"
        "  <link name='tip'><inertial><mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' "
        "ixy='0' ixz='0' iyz='0'/></inertial>"
        "    <collision><origin xyz='0 0 0.2'/><geometry><box size='0.1 0.1 "
        "0.1'/></geometry></collision></link>"
        "  <joint name='bx' type='revolute'><parent link='base'/><child link='d1'/>"
        "    <origin xyz='0 0 0.5' rpy='0.3 0.2 0.1'/><axis xyz='1 0 0'/><limit lower='-3' "
        "upper='3' effort='1' velocity='1'/></joint>"
        "  <joint name='by' type='revolute'><parent link='d1'/><child link='d2'/>"
        "    <axis xyz='0 1 0'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
        "  <joint name='bz' type='continuous'><parent link='d2'/><child link='tip'/><axis xyz='0 0 "
        "-1'/></joint>"
        "</robot>";
    Sim_Config config = make_flat_config(0.0f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    char error[256];
    bool loaded = load_robot(&world, xml, strlen(xml), "", error, sizeof(error));
    CHECK(loaded);
    if (loaded) {
        Robot *robot = &world.robot;
        CHECK(robot->body_count == 2 && robot->ball_count == 1);
        f32 a = 0.4f, b = -0.3f, c = 0.7f;
        // Rotate the tip about the pivot: Rx(a), then Ry(b), then c about the -z axis.
        const Robot_Ball *ball = &robot->balls[0];
        Pose base = robot->bodies[ball->body_a].current;
        Pose pivot = multiply_poses(base, ball->frame_a);
        Quat turn =
            multiply_quats(make_quat_from_axis_angle(v3{1.0f, 0.0f, 0.0f}, a),
                           multiply_quats(make_quat_from_axis_angle(v3{0.0f, 1.0f, 0.0f}, b),
                                          make_quat_from_axis_angle(v3{0.0f, 0.0f, -1.0f}, c)));
        Quat tip = multiply_quats(pivot.q, turn);
        set_body_pose(world.physics, robot->bodies[ball->body_b].physics_body, Pose{pivot.p, tip});
        read_robot_state(robot);
        CHECK_NEAR(robot->joints[find_robot_joint(robot, "bx")].position, a, 1e-4);
        CHECK_NEAR(robot->joints[find_robot_joint(robot, "by")].position, b, 1e-4);
        CHECK_NEAR(robot->joints[find_robot_joint(robot, "bz")].position, c, 1e-4);
    } else {
        fprintf(stderr, "  %s\n", error);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

static u64 drive_and_hash(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    u64 hash = 0;
    if (load_banksia(&world)) {
        set_control_mode(&world, CONTROL_KEYBOARD);
        drive_robot(&world.robot, 0.5f, 0.3f);
        run_steps(&world, 2.0f);
        hash = hash_world_state(&world);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
    return hash;
}

// A description that fails to load leaves the running robot alone.
// Bad descriptions are refused with a reason, and a world takes only one robot.
static void test_rejects_bad_descriptions(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    char error[256];

    CHECK(create_world(&world, &allocator, &config));
    const char *broken = "<robot name='x'><link name='a'></robot>";
    CHECK(!load_robot(&world, broken, strlen(broken), "", error, sizeof(error)));
    CHECK(!world.has_robot);
    destroy_world(&world);
    reset_allocator(&allocator);

    CHECK(create_world(&world, &allocator, &config));
    const char *floating =
        "<robot name='x'><link name='a'/><link name='b'/>"
        "<joint name='j' type='floating'><parent link='a'/><child link='b'/></joint></robot>";
    CHECK(!load_robot(&world, floating, strlen(floating), "", error, sizeof(error)));
    CHECK(strstr(error, "floating") != NULL);
    destroy_world(&world);
    reset_allocator(&allocator);

    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        CHECK(!load_robot(&world, broken, strlen(broken), "", error, sizeof(error)));
        CHECK(strstr(error, "already") != NULL);
        CHECK(world.robot.body_count == 14);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

static const Robot_Actuator *find_actuator(const World *world, u32 node_id)
{
    for (u32 a = 0; a < world->actuators.count; a++) {
        if (world->actuators.actuators[a].node_id == node_id) {
            return &world->actuators.actuators[a];
        }
    }
    return NULL;
}

// Every command a driver would send for the four wheels at wheel_radps, forwards.
static void command_wheels(World *world, f32 wheel_radps)
{
    for (u32 w = 0; w < world->robot.wheel_count; w++) {
        const Robot_Wheel *wheel = &world->robot.wheels[w];
        for (u32 a = 0; a < world->actuators.count; a++) {
            if (world->actuators.actuators[a].joint == wheel->joint) {
                Actuator_Command command = {.node_id = world->actuators.actuators[a].node_id,
                                            .mode = ACTUATOR_VELOCITY,
                                            .value = wheel->forward_sign * wheel_radps};
                CHECK(command_actuator(&world->actuators, &command, world->step_count));
            }
        }
    }
}

// Banksia's eight BLCMD controllers, commanded as a driver would over CAN.
static void test_actuators(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    config.command_timeout_ms = 250.0f; // off by default; on here to test the watchdog
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (!load_banksia(&world)) {
        destroy_world(&world);
        destroy_allocator(&allocator);
        return;
    }
    Robot *robot = &world.robot;
    CHECK(world.actuators.count == 8);
    const char *joint_of_node[] = {"", "flw", "blw", "brw", "frw", "flp", "blp", "brp", "frp"};
    for (u32 node = 1; node <= 8; node++) {
        const Robot_Actuator *actuator = find_actuator(&world, node);
        CHECK(actuator != NULL);
        if (actuator) {
            const Urdf_Joint *urdf =
                &robot->model.joints[robot->joints[actuator->joint].urdf_joint];
            CHECK(strcmp(urdf->name, joint_of_node[node]) == 0);
        }
    }
    Actuator_Command stranger = {.node_id = 42, .mode = ACTUATOR_VELOCITY, .value = 1.0f};
    CHECK(!command_actuator(&world.actuators, &stranger, 0));
    run_steps(&world, 1.0f);

    // Velocity: renewed every 100 ms, well inside the watchdog.
    v3 start = get_body_of_link(robot, "base_link")->current.p;
    for (u32 i = 0; i < 20; i++) {
        command_wheels(&world, 2.0f);
        run_steps(&world, 0.1f);
    }
    const Robot_Actuator *front_left = find_actuator(&world, 1);
    f32 wheel_speed = absolute(robot->joints[front_left->joint].velocity);
    CHECK_NEAR(wheel_speed, 2.0, 0.2);
    v3 moved = inverse_rotate_vector(get_body_of_link(robot, "base_link")->current.q,
                                     get_body_of_link(robot, "base_link")->current.p - start);
    CHECK(moved.x > 0.5f); // about 2 rad/s × wheel radius × 2 s, forwards

    // Silence: the watchdog expires the commands, and the wheels hold.
    run_steps(&world, 1.0f);
    Actuator_Telemetry telemetry[8];
    CHECK(get_actuator_telemetry(&world.actuators, robot, telemetry, 8) == 8);
    for (u32 a = 0; a < 8; a++) {
        if (telemetry[a].node_id <= 4) {
            CHECK(telemetry[a].timed_out);
            CHECK(telemetry[a].mode == ACTUATOR_VELOCITY);
            CHECK(absolute(telemetry[a].velocity) < 0.05f);
        } else {
            CHECK(!telemetry[a].timed_out); // the pivots were never commanded
        }
    }

    // Position: steer the front-left pivot, renewing the command as a driver would.
    const Robot_Actuator *pivot = find_actuator(&world, 5);
    for (u32 i = 0; i < 20; i++) {
        Actuator_Command steer = {.node_id = 5, .mode = ACTUATOR_POSITION, .value = 0.3f};
        CHECK(command_actuator(&world.actuators, &steer, world.step_count));
        run_steps(&world, 0.1f);
    }
    CHECK_NEAR(robot->joints[pivot->joint].position, 0.3, 0.02);

    // It swings at robot.servo_speed (2 rad/s), not in one step: 1 rad takes half a second.
    Actuator_Command swing = {.node_id = 5, .mode = ACTUATOR_POSITION, .value = 1.3f};
    CHECK(command_actuator(&world.actuators, &swing, world.step_count));
    run_steps(&world, 0.25f);
    f32 halfway = robot->joints[pivot->joint].position;
    run_steps(&world, 0.5f);
    f32 arrived = robot->joints[pivot->joint].position;
    printf("  servo: pivot at %.3f rad after 0.25 s, %.3f rad after 0.75 s (target 1.3)\n",
           (f64)halfway, (f64)arrived);
    CHECK_NEAR(halfway, 0.8, 0.05);
    CHECK_NEAR(arrived, 1.3, 0.02);
    Actuator_Command back = {.node_id = 5, .mode = ACTUATOR_POSITION, .value = 0.3f};
    CHECK(command_actuator(&world.actuators, &back, world.step_count));
    run_steps(&world, 0.75f);

    // Effort: a small torque, well short of the speed limit, is applied as commanded.
    Actuator_Command push = {.node_id = 1, .mode = ACTUATOR_EFFORT, .value = 0.5f};
    CHECK(command_actuator(&world.actuators, &push, world.step_count));
    run_steps(&world, 0.05f);
    CHECK(get_actuator_telemetry(&world.actuators, robot, telemetry, 8) == 8);
    for (u32 a = 0; a < 8; a++) {
        if (telemetry[a].node_id == 1) {
            CHECK(telemetry[a].mode == ACTUATOR_EFFORT);
            CHECK_NEAR(absolute(telemetry[a].effort), 0.5, 0.01);
        }
    }

    // Keyboard mode takes the wheels back into velocity control and stops them; CAN
    // commands then have no effect until control is handed back.
    set_control_mode(&world, CONTROL_KEYBOARD);
    CHECK(robot->joints[front_left->joint].drive == DRIVE_VELOCITY);
    Actuator_Command ignored = {.node_id = 1, .mode = ACTUATOR_VELOCITY, .value = 5.0f};
    CHECK(command_actuator(&world.actuators, &ignored, world.step_count));
    run_steps(&world, 0.5f);
    CHECK(absolute(robot->joints[front_left->joint].velocity) < 0.05f);
    // Handing back to CAN forgets that command: the wheels hold until a new one.
    set_control_mode(&world, CONTROL_CAN);
    run_steps(&world, 0.5f);
    CHECK(absolute(robot->joints[front_left->joint].velocity) < 0.05f);
    CHECK(command_actuator(&world.actuators, &ignored, world.step_count));
    run_steps(&world, 0.2f); // inside the 250 ms watchdog
    CHECK(absolute(robot->joints[front_left->joint].velocity) > 2.0f); // one wheel pushing 24 kg
    destroy_world(&world);

    // Two controllers can't share a node id.
    u64 size = 0;
    char *xml = read_file(banksia_path, &size);
    if (xml) {
        char *second = strstr(xml, "<param name=\"canid\">2</param>");
        CHECK(second != NULL);
        if (second) {
            second[strlen("<param name=\"canid\">")] = '1';
        }
        reset_allocator(&allocator);
        CHECK(create_world(&world, &allocator, &config));
        char error[256];
        CHECK(!load_robot(&world, xml, size, "", error, sizeof(error)));
        CHECK(strstr(error, "node id 1") != NULL);
        destroy_world(&world);
        free(xml);
    }
    destroy_allocator(&allocator);
}

// What blcmd_hardware2 would send for the same joint speed on every wheel, through the BLCMD
// codec with Banksia's URDF mapping. The left motors are reversed, so their values go out
// negated, and the sim must undo that for the rover to drive straight.
static void test_blcmd_drives_straight(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        Robot *robot = &world.robot;
        run_steps(&world, 1.0f);
        const Robot_Body *base = get_body_of_link(robot, "base_link");
        v3 start = base->current.p;
        Quat start_rotation = base->current.q;
        for (u32 i = 0; i < 20; i++) {
            for (u32 node_id = 1; node_id <= 4; node_id++) {
                const Robot_Actuator *actuator = find_actuator(&world, node_id);
                Blcmd_Node node;
                CHECK(actuator && make_blcmd_node(robot, actuator, &node));
                Can_Frame frame = encode_speed_frame(&node, 3.75f);
                CHECK(frame.data[0] == (node.direction > 0.0f ? 0x10 : 0xf0)); // ±4096
                Blcmd_Message message = decode_blcmd_frame(&frame, &node);
                CHECK(message.kind == BLCMD_MESSAGE_COMMAND);
                CHECK(command_actuator(&world.actuators, &message.command, world.step_count));
            }
            run_steps(&world, 0.1f);
        }
        v3 moved = inverse_rotate_vector(start_rotation, base->current.p - start);
        f32 turned = get_yaw(base->current.q) - get_yaw(start_rotation);
        printf("  blcmd: 3.75 rad/s on every wheel moved %.3f m forward, %.3f m sideways, "
               "turned %.3f rad\n",
               (f64)moved.x, (f64)moved.y, (f64)turned);
        CHECK(moved.x > 0.5f);
        CHECK(absolute(moved.y) < 0.1f);
        CHECK(absolute(turned) < 0.1f);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

// Keyboard steering: the pivots point the same way forwards and backwards, and no wheel
// is driven faster than the speed limit over the ground.
static void test_steering(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        Robot *robot = &world.robot;
        f32 forward_steer[4], reverse_steer[4];
        for (u32 pass = 0; pass < 2; pass++) {
            steer_robot(robot, pass == 0 ? 1.0f : -1.0f, 1.0f, 0.6f, 0.8f);
            f32 fastest = 0.0f;
            for (u32 w = 0; w < robot->wheel_count; w++) {
                const Robot_Wheel *wheel = &robot->wheels[w];
                fastest =
                    max(fastest, absolute(robot->joints[wheel->joint].command) * wheel->radius);
                f32 steer = robot->joints[wheel->steer_joint].command;
                (pass == 0 ? forward_steer : reverse_steer)[w] = steer;
            }
            CHECK(fastest <= 0.6f + 1e-4f);
        }
        for (u32 w = 0; w < robot->wheel_count; w++) {
            printf("  steering: %s pivot %.3f rad forwards, %.3f backwards\n",
                   robot->model.joints[robot->joints[robot->wheels[w].steer_joint].urdf_joint].name,
                   (f64)forward_steer[w], (f64)reverse_steer[w]);
            CHECK_NEAR(forward_steer[w], reverse_steer[w], 1e-5);
        }

        // Left held while reversing swings the nose right, like a car.
        run_steps(&world, 1.0f);
        set_control_mode(&world, CONTROL_KEYBOARD);
        f32 yaw_start = get_yaw(get_body_of_link(robot, "base_link")->current.q);
        for (u32 i = 0; i < 2 * world.config.physics_hz; i++) {
            steer_robot(robot, -1.0f, 1.0f, 0.6f, 0.8f);
            step_world(&world);
        }
        f32 turned =
            wrap_turn(get_yaw(get_body_of_link(robot, "base_link")->current.q) - yaw_start);
        printf("  steering: 2 s reversing with left held turned %.3f rad\n", (f64)turned);
        CHECK(turned < -0.3f);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

// A robot turned upside down comes back upright where it spawned, and holds still there.
static void test_reset(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    if (load_banksia(&world)) {
        Robot *robot = &world.robot;
        run_steps(&world, 1.0f);
        v3 settled = get_body_of_link(robot, "base_link")->current.p;

        // Driving over CAN, then flipped onto its back a metre up.
        command_wheels(&world, 3.0f);
        run_steps(&world, 1.0f);
        Pose flip = {v3{0.0f, 0.0f, 1.0f}, make_quat_from_axis_angle(v3{1.0f, 0.0f, 0.0f}, PI_F32)};
        for (u32 b = 0; b < robot->body_count; b++) {
            u32 body = robot->bodies[b].physics_body;
            set_body_pose(world.physics, body,
                          multiply_poses(flip, get_body_pose(world.physics, body)));
        }
        run_steps(&world, 2.0f);
        printf("  reset: flipped, uprightness %.2f\n", (f64)get_uprightness(robot));
        CHECK(get_uprightness(robot) < -0.5f);

        reset_robot(&world);
        run_steps(&world, 2.0f);
        v3 moved = get_body_of_link(robot, "base_link")->current.p - settled;
        printf("  reset: back %.3f m from where it settled, uprightness %.3f\n",
               (f64)get_length(moved), (f64)get_uprightness(robot));
        CHECK(get_length(moved) < 0.05f);
        CHECK(get_uprightness(robot) > 0.98f);
        CHECK(get_max_body_speed(robot) < 0.01f);
        CHECK(get_max_joint_separation(robot) < 0.005f);
        for (u32 a = 0; a < world.actuators.count; a++) {
            CHECK(!world.actuators.actuators[a].active);
        }
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

static const char *digger_path;

// Banksia on the soil, driven through it: the wheels leave ruts. Then the digger lowers its
// bucket, curls it and drives forward, and cuts a trench the soil pushes back against.
static void test_soil_digging(void)
{
    Sim_Config config = make_flat_config(-9.81f);
    config.soil_enabled = true;
    config.soil_size[0] = config.soil_size[1] = 8.0f;
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    static World world;
    CHECK(create_world(&world, &allocator, &config));
    u64 size = 0;
    char *xml = read_file(digger_path, &size);
    CHECK(xml != NULL);
    char error[256];
    bool loaded = xml && load_robot(&world, xml, size, "", error, sizeof(error));
    CHECK(loaded);
    free(xml);
    if (loaded) {
        Robot *robot = &world.robot;
        CHECK(world.boom_joint >= 0 && world.bucket_joint >= 0);
        run_steps(&world, 1.0f);
        f32 untouched = world.soil.grid.max_height;
        u64 changes = world.soil.changes;
        CHECK(changes > 0); // it has sunk in a little already

        // Bucket down into the soil, curled a little, and forwards at 0.1 m/s.
        set_control_mode(&world, CONTROL_KEYBOARD);
        robot->joints[world.boom_joint].command = 0.6f;
        robot->joints[world.bucket_joint].command = 0.3f;
        run_steps(&world, 1.5f);
        v3 start = get_body_of_link(robot, "base_link")->current.p;
        f32 most_force = 0.0f;
        for (u32 i = 0; i < 3 * world.config.physics_hz; i++) {
            drive_robot(robot, 0.1f, 0.0f);
            step_world(&world);
            most_force = max(most_force, world.bucket_soil_force);
        }
        v3 moved = get_body_of_link(robot, "base_link")->current.p - start;
        // The trench: the lowest soil anywhere is well below where the ground started.
        f32 lowest = INFINITY;
        const Terrain *grid = &world.soil.grid;
        for (u32 i = 0; i < grid->rows * grid->cols; i++) {
            lowest = min(lowest, grid->heights[i]);
        }
        printf("  digging: moved %.2f m, bucket load up to %.0f N, trench %.0f mm deep, "
               "%llu soil updates\n",
               (f64)moved.x, (f64)most_force, (f64)((untouched - lowest) * 1000.0f),
               (unsigned long long)world.soil.changes);
        CHECK(moved.x > 0.05f);
        CHECK(most_force > 20.0f);
        CHECK(untouched - lowest > 0.03f);
        CHECK(get_uprightness(robot) > 0.9f);
    } else {
        fprintf(stderr, "  %s\n", error);
    }
    destroy_world(&world);
    destroy_allocator(&allocator);
}

int main(int argc, char **argv)
{
    test_ball_angles();
    if (argc > 1) {
        banksia_path = argv[1];
        test_structure();
        test_settles();
        test_diff_bar_couples();
        test_drives();
        CHECK(drive_and_hash() == drive_and_hash());
        test_rejects_bad_descriptions();
        test_actuators();
        test_blcmd_drives_straight();
        test_reset();
        test_steering();
    }
    if (argc > 2) {
        digger_path = argv[2];
        test_soil_digging();
    }
    return report_checks("robot");
}
