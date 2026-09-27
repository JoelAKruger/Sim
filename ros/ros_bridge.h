#pragma once

#include "core/shared_state.h"
#include "core/config.h"

// The ROS 2 adapter: the only part of Regolith that includes rclcpp. It is optional.
// With REGOLITH_WITH_ROS=OFF, ros_disabled.cpp provides these same functions as no-ops,
// so the rest of the program is identical either way. There is one bridge per process.

// Initialises rclcpp (without its signal handlers; the app owns Ctrl-C) and creates
// the node.
bool create_ros_bridge(i32 argc, char **argv);

// Declares every setting in the schema as a ROS parameter, with the current value as
// its default, then applies any overrides from --params-file or -p. `ros2 param dump`
// therefore shows the complete effective settings.
void apply_ros_parameters(Sim_Config *config);

// Blocks until /robot_description arrives (it is latched, so one published before the sim
// started counts) or quit is requested. Returns the URDF from malloc (release with free),
// or NULL. Later descriptions are ignored: the robot is loaded once.
char *wait_for_robot_description(Shared_Global_State *shared, u64 *size);

// Starts the ROS thread, which publishes the enabled sensors (the IMU, the LiDAR and the
// camera, as their real drivers do), and /clock with ros.use_sim_time, from the shared
// state until quit is requested. config must outlive the thread.
bool start_ros_thread(Shared_Global_State *shared, const Sim_Config *config);

// Joins the ROS thread and shuts rclcpp down.
void destroy_ros_bridge(void);
