#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "core/urdf.h"
#include "tests/check.h"

static const Urdf_Joint *get_joint(const Urdf_Model *model, const char *name)
{
    i32 index = find_urdf_joint(model, name);
    CHECK(index >= 0);
    return index >= 0 ? &model->joints[index] : &model->joints[0];
}

static const Urdf_Link *get_link(const Urdf_Model *model, const char *name)
{
    i32 index = find_urdf_link(model, name);
    CHECK(index >= 0);
    return index >= 0 ? &model->links[index] : &model->links[0];
}

// The team's real rover, including its non-standard parts: a closed kinematic chain
// written as Gazebo DetachableJoint plugins, ball joints as revolute chains through
// dummy links, and ros2_control blocks naming the CAN hardware.
static void test_banksia(Linear_Allocator *allocator, const char *path)
{
    u64 size = 0;
    char *xml = read_file(path, &size);
    CHECK(xml != NULL);
    if (!xml) {
        return;
    }
    Urdf_Model model;
    char error[256];
    bool parsed = parse_urdf(&model, allocator, xml, size, error, sizeof(error));
    CHECK(parsed);
    if (!parsed) {
        fprintf(stderr, "  %s\n", error);
        free(xml);
        return;
    }

    CHECK(strcmp(model.name, "Banksia") == 0);
    CHECK(model.link_count == 26); // the commented-out ball joint links are not included
    CHECK(model.joint_count == 25);
    CHECK(strcmp(model.links[model.root].name, "base_link") == 0);
    CHECK(find_urdf_link(&model, "tl_ball") < 0);
    CHECK(!model.self_collide);

    CHECK(model.closure_count == 2);
    CHECK(model.closures[0].parent_link == (u32)find_urdf_link(&model, "left_leg"));
    CHECK(model.closures[0].child_link == (u32)find_urdf_link(&model, "left_diffend_dummy"));
    CHECK(model.closures[1].parent_link == (u32)find_urdf_link(&model, "right_leg"));

    const Urdf_Joint *frw = get_joint(&model, "frw");
    CHECK(frw->type == URDF_JOINT_CONTINUOUS);
    CHECK(frw->parent == (u32)find_urdf_link(&model, "fr_ankle"));
    CHECK(frw->child == (u32)find_urdf_link(&model, "fr_wheel"));
    CHECK_NEAR(frw->axis.z, 1.0, 1e-6);
    CHECK_NEAR(frw->origin.p.z, -0.241, 1e-6);
    CHECK_NEAR(frw->effort, 10.0, 1e-6);
    CHECK(frw->actuated && !frw->position_command);

    const Urdf_Joint *frp = get_joint(&model, "frp");
    CHECK(frp->type == URDF_JOINT_REVOLUTE && frp->has_limits);
    CHECK_NEAR(frp->upper, 3.14159, 1e-5);
    CHECK(frp->actuated && frp->position_command);

    // Hardware parameters survive for the CAN adapter.
    CHECK(strcmp(get_control_param(&model, get_joint(&model, "flw")->control, "canid"), "1") == 0);
    CHECK(strcmp(get_control_param(&model, frp->control, "canid"), "8") == 0);
    CHECK(strcmp(get_control_param(&model, frp->control, "gear_ratio"), "36.0") == 0);
    CHECK(strcmp(model.controls[frp->control].plugin, "blcmd_hardware2/BLCMDHardware") == 0);

    u32 actuated = 0;
    for (u32 i = 0; i < model.joint_count; i++) {
        actuated += model.joints[i].actuated;
    }
    CHECK(actuated == 8);
    CHECK(!get_joint(&model, "chassis_to_left_leg")->actuated);
    CHECK(get_joint(&model, "tl_ball_y")->axis.y == 1.0f);

    const Urdf_Link *chassis = get_link(&model, "chassis");
    CHECK_NEAR(chassis->inertial.mass, 5.45693, 1e-5);
    CHECK_NEAR(chassis->inertial.inertia.cx.x, 0.20399116109, 1e-7);
    CHECK_NEAR(chassis->inertial.inertia.cy.z, -7.4614396404e-03, 1e-9);
    CHECK(chassis->collision_count == 1 && chassis->visual_count == 1);
    const Urdf_Shape *box = &model.collisions[chassis->first_collision];
    CHECK(box->geometry.type == URDF_GEOMETRY_BOX);
    CHECK_NEAR(box->geometry.size.y, 0.545, 1e-6);
    const Urdf_Shape *visual = &model.visuals[chassis->first_visual];
    CHECK(visual->geometry.type == URDF_GEOMETRY_MESH);
    CHECK(strstr(visual->geometry.mesh, "meshes/chassis.stl") != NULL);
    CHECK_NEAR(visual->color[0], 0.882353, 1e-6);

    const Urdf_Link *leg = get_link(&model, "right_leg");
    CHECK(leg->collision_count == 9);
    const Urdf_Shape *cylinder = &model.collisions[leg->first_collision];
    CHECK(cylinder->geometry.type == URDF_GEOMETRY_CYLINDER);
    CHECK_NEAR(cylinder->geometry.radius, 0.043, 1e-6);
    CHECK_NEAR(cylinder->geometry.length, 0.058, 1e-6);
    CHECK(cylinder->link == (u32)find_urdf_link(&model, "right_leg"));

    CHECK_NEAR(get_link(&model, "tl_ball_link_x")->inertial.mass, 0.00001, 1e-9);
    CHECK(!get_link(&model, "base_link")->inertial.present);
    free(xml);
}

static void test_materials(Linear_Allocator *allocator)
{
    const char *xml =
        "<robot name='r'>"
        "  <link name='a'><visual><geometry><sphere radius='1'/></geometry>"
        "    <material name='blue'/></visual></link>"
        "  <link name='b'/>"
        "  <joint name='j1' type='revolute'><parent link='a'/><child link='b'/>"
        "    <axis xyz='0 0 2'/><limit lower='-1' upper='1' effort='5' velocity='1'/></joint>"
        "  <link name='c'/>"
        "  <joint name='j2' type='continuous'><parent link='b'/><child link='c'/></joint>"
        "  <material name='blue'><color rgba='0 0 1 1'/></material>"
        "</robot>";
    Urdf_Model model;
    char error[256];
    CHECK(parse_urdf(&model, allocator, xml, strlen(xml), error, sizeof(error)));
    CHECK(model.visuals[0].color[2] == 1.0f && model.visuals[0].color[0] == 0.0f);
    CHECK(model.joints[0].axis.z == 1.0f); // normalised
    CHECK(model.self_collide == false);
}

static void test_rejects(Linear_Allocator *allocator)
{
    struct Bad_Robot {
        const char *xml;
        const char *expected;
    };
    Bad_Robot bad_robots[] = {
        {"<robot name='r'><link name='a'/><link name='b'/>"
         "<joint name='j' type='hinge'><parent link='a'/><child link='b'/></joint></robot>",
         "unknown type \"hinge\""},
        {"<robot name='r'><link name='a'/>"
         "<joint name='j' type='fixed'><parent link='a'/><child link='z'/></joint></robot>",
         "missing link \"z\""},
        {"<robot name='r'><link name='a'/><link name='b'/><link name='c'/>"
         "<joint name='j1' type='fixed'><parent link='a'/><child link='c'/></joint>"
         "<joint name='j2' type='fixed'><parent link='b'/><child link='c'/></joint></robot>",
         "child of both"},
        {"<robot name='r'><link name='a'/><link name='b'/></robot>", "expected one root"},
        {"<robot name='r'><link name='a'/><link name='a'/></robot>", "two links"},
        {"<robot name='r' xmlns:xacro='x'><xacro:property name='p' value='1'/></robot>",
         "unexpanded xacro"},
        {"<robot name='r'><link name='a'><inertial><mass value='heavy'/></inertial></link></robot>",
         "not a number"},
        {"<robot name='r'><link name='a'>\n</robot>", "line 2"},
    };
    for (u32 i = 0; i < ARRAY_COUNT(bad_robots); i++) {
        Urdf_Model model;
        char error[256] = "";
        bool parsed = parse_urdf(&model, allocator, bad_robots[i].xml, strlen(bad_robots[i].xml),
                                 error, sizeof(error));
        CHECK(!parsed);
        if (!parsed && strstr(error, bad_robots[i].expected) == NULL) {
            fprintf(stderr, "  got \"%s\", expected \"%s\"\n", error, bad_robots[i].expected);
            check_failures++;
        }
    }
}

// roll about x, then pitch about y, then yaw about z, all about fixed axes.
static void test_rpy(void)
{
    Quat yaw = make_quat_from_rpy(0.0, 0.0, 0.5 * M_PI);
    v3 x = rotate_vector(yaw, v3{1.0f, 0.0f, 0.0f});
    CHECK_NEAR(x.y, 1.0, 1e-6);
    Quat combined = make_quat_from_rpy(0.5 * M_PI, 0.0, 0.5 * M_PI);
    v3 y = rotate_vector(combined, v3{0.0f, 1.0f, 0.0f});
    // Roll takes y to z; yaw leaves z alone.
    CHECK_NEAR(y.z, 1.0, 1e-6);
}

int main(int argc, char **argv)
{
    Linear_Allocator allocator;
    CHECK(create_allocator(&allocator, 64 * MEGABYTE));
    if (argc > 1) {
        test_banksia(&allocator, argv[1]);
    }
    test_materials(&allocator);
    test_rejects(&allocator);
    test_rpy();
    destroy_allocator(&allocator);
    return report_checks("urdf");
}
