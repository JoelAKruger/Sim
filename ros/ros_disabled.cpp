#include <string.h>

#include "ros/ros_bridge.h"

// Linked instead of ros_bridge.cpp when built with REGOLITH_WITH_ROS=OFF. The sim runs
// the same; it takes settings from --config only and publishes nothing.

bool create_ros_bridge(i32 argc, char **argv)
{
    for (i32 i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--ros-args") == 0) {
            log_warning("built without ROS 2: ignoring --ros-args; use --config for settings");
            break;
        }
    }
    log_info("built without ROS 2: no topics or parameters");
    return true;
}

void apply_ros_parameters(Sim_Config *config) { (void)config; }

char *wait_for_robot_description(Shared_Global_State *shared, u64 *size)
{
    (void)shared;
    *size = 0;
    log_error("built without ROS 2, so there is no /robot_description: set robot.urdf or "
              "pass --urdf");
    return NULL;
}

bool start_ros_thread(Shared_Global_State *shared, const Sim_Config *config)
{
    (void)shared;
    (void)config;
    return true;
}

void destroy_ros_bridge(void) {}
