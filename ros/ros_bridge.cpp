#include "ros/ros_bridge.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "core/sensors/camera_model.h"
#include "core/sensors/lidar.h"

struct Ros_Bridge {
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher;
    char *description; // the first /robot_description, from malloc, until handed over
    u64 description_size;
    Shared_Global_State *shared;
    pthread_t thread;
    bool thread_started;
    u64 last_clock_ns;

    // Sensors: a publisher for each enabled one, and its message, sized once.
    const Sim_Config *config;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_publisher;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_transform_publisher;
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::msg::Image color_image;
    sensor_msgs::msg::Image depth_image;
    sensor_msgs::msg::CameraInfo color_info;
    sensor_msgs::msg::CameraInfo depth_info;
};

static Ros_Bridge ros;

bool create_ros_bridge(i32 argc, char **argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    ros.node = rclcpp::Node::make_shared("regolith");
    // Reliable, so it matches every /clock subscriber whatever QoS it asks for.
    ros.clock_publisher =
        ros.node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
    ros.last_clock_ns = UINT64_MAX;
    return true;
}

static rclcpp::ParameterValue get_field_value(const Config_Field *field, const Sim_Config *config)
{
    if (field->type == CONFIG_STRING) {
        return rclcpp::ParameterValue(std::string(get_config_text(config, field)));
    }
    f64 values[CONFIG_MAX_COUNT];
    u32 count = get_config(config, field, values);
    if (field->type == CONFIG_BOOL) {
        return rclcpp::ParameterValue(values[0] != 0.0);
    }
    if (field->type == CONFIG_U32) {
        std::vector<i64> integers;
        for (u32 i = 0; i < count; i++) {
            integers.push_back((i64)values[i]);
        }
        return count == 1 ? rclcpp::ParameterValue(integers[0]) : rclcpp::ParameterValue(integers);
    }
    if (count == 1) {
        return rclcpp::ParameterValue(values[0]);
    }
    return rclcpp::ParameterValue(std::vector<f64>(values, values + count));
}

// Integers and doubles are both accepted for any numeric setting, so "240" and "240.0"
// mean the same, as they do in the config file.
static bool convert_value_to_numbers(const rclcpp::ParameterValue *value, f64 *numbers, u32 *count)
{
    switch (value->get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
        numbers[0] = value->get<f64>();
        *count = 1;
        return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
        numbers[0] = (f64)value->get<i64>();
        *count = 1;
        return true;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
        std::vector<f64> list = value->get<std::vector<f64>>();
        if (list.size() > CONFIG_MAX_COUNT) {
            return false;
        }
        *count = (u32)list.size();
        for (u32 i = 0; i < *count; i++) {
            numbers[i] = list[i];
        }
        return true;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
        std::vector<i64> list = value->get<std::vector<i64>>();
        if (list.size() > CONFIG_MAX_COUNT) {
            return false;
        }
        *count = (u32)list.size();
        for (u32 i = 0; i < *count; i++) {
            numbers[i] = (f64)list[i];
        }
        return true;
    }
    default:
        return false;
    }
}

void apply_ros_parameters(Sim_Config *config)
{
    for (u32 i = 0; i < config_field_count; i++) {
        const Config_Field *field = &config_fields[i];
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = field->help;
        descriptor.dynamic_typing = true; // accept 240 where 240.0 was declared
        rclcpp::ParameterValue value =
            ros.node->declare_parameter(field->key, get_field_value(field, config), descriptor);

        char error[256];
        bool applied;
        if (field->type == CONFIG_BOOL) {
            applied = value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL;
            if (!applied) {
                snprintf(error, sizeof(error), "%s: expected true or false, got %s", field->key,
                         rclcpp::to_string(value.get_type()).c_str());
            } else {
                f64 flag = value.get<bool>() ? 1.0 : 0.0;
                applied = set_config(config, field->key, &flag, 1, error, sizeof(error));
            }
        } else if (field->type == CONFIG_STRING) {
            applied = value.get_type() == rclcpp::ParameterType::PARAMETER_STRING;
            if (!applied) {
                snprintf(error, sizeof(error), "%s: expected a string, got %s", field->key,
                         rclcpp::to_string(value.get_type()).c_str());
            } else {
                applied = set_config_text(config, field->key, value.get<std::string>().c_str(),
                                          error, sizeof(error));
            }
        } else {
            f64 numbers[CONFIG_MAX_COUNT];
            u32 count = 0;
            applied = convert_value_to_numbers(&value, numbers, &count);
            if (!applied) {
                snprintf(error, sizeof(error), "%s: expected a number or up to %u numbers, got %s",
                         field->key, CONFIG_MAX_COUNT, rclcpp::to_string(value.get_type()).c_str());
            } else {
                applied = set_config(config, field->key, numbers, count, error, sizeof(error));
            }
        }
        if (!applied) {
            // Keep the parameter truthful: it reports the value actually in use.
            log_warning("parameter %s; ignored", error);
            ros.node->set_parameter(rclcpp::Parameter(field->key, get_field_value(field, config)));
        }
    }

    // Before this, a misspelt parameter was silently ignored.
    const std::map<std::string, rclcpp::ParameterValue> &overrides =
        ros.node->get_node_parameters_interface()->get_parameter_overrides();
    for (const std::pair<const std::string, rclcpp::ParameterValue> &entry : overrides) {
        const char *key = entry.first.c_str();
        if (!find_config_field(key) && entry.first != "use_sim_time" &&
            entry.first.rfind("qos_overrides.", 0) != 0) {
            log_warning("parameter %s is not a Regolith setting; ignored", key);
        }
    }
}

static void publish_clock(void)
{
    u64 now = get_sim_time(ros.shared);
    if (now == ros.last_clock_ns) {
        return;
    }
    ros.last_clock_ns = now;
    rosgraph_msgs::msg::Clock message;
    message.clock.sec = (i32)(now / NS_PER_S);
    message.clock.nanosec = (u32)(now % NS_PER_S);
    ros.clock_publisher->publish(message);
}

// robot_state_publisher latches the URDF: reliable, transient local, depth 1. A volatile
// subscription would never see a description published before the sim started.
static void handle_robot_description(const std_msgs::msg::String &message)
{
    if (ros.description) {
        return; // only the first is used
    }
    ros.description = (char *)malloc(message.data.size() + 1);
    if (!ros.description) {
        log_error("ros: out of memory for a %zu-byte robot description", message.data.size());
        return;
    }
    memcpy(ros.description, message.data.c_str(), message.data.size() + 1);
    ros.description_size = message.data.size();
}

char *wait_for_robot_description(Shared_Global_State *shared, u64 *size)
{
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription =
        ros.node->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
            &handle_robot_description);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ros.node);
    u64 last_log_ns = get_time_ns();
    while (!ros.description && !is_quit_requested(shared)) {
        executor.spin_once(std::chrono::milliseconds(100));
        if (!ros.description && get_time_ns() - last_log_ns > 5 * NS_PER_S) {
            log_info("still waiting for /robot_description (is robot_state_publisher running?)");
            last_log_ns = get_time_ns();
        }
    }
    executor.remove_node(ros.node);
    subscription.reset(); // later descriptions are not wanted
    char *description = ros.description;
    *size = ros.description_size;
    ros.description = NULL;
    return description;
}

static builtin_interfaces::msg::Time make_stamp(u64 ns)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = (i32)(ns / NS_PER_S);
    stamp.nanosec = (u32)(ns % NS_PER_S);
    return stamp;
}

// livox_ros_driver2's PointCloud2 layout (xfer_format 0, its LivoxPointXyzrtlt): packed,
// 26 bytes a point.
#define LIVOX_POINT_STEP 26

static void add_point_field(sensor_msgs::msg::PointCloud2 *cloud, const char *name, u32 offset,
                            u8 datatype)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud->fields.push_back(field);
}

static void init_livox_cloud(sensor_msgs::msg::PointCloud2 *cloud, const Lidar_Config *config)
{
    cloud->header.frame_id = config->frame;
    cloud->height = 1;
    add_point_field(cloud, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
    add_point_field(cloud, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
    add_point_field(cloud, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    add_point_field(cloud, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    add_point_field(cloud, "tag", 16, sensor_msgs::msg::PointField::UINT8);
    add_point_field(cloud, "line", 17, sensor_msgs::msg::PointField::UINT8);
    add_point_field(cloud, "timestamp", 18, sensor_msgs::msg::PointField::FLOAT64);
    cloud->point_step = LIVOX_POINT_STEP;
    cloud->is_bigendian = false;
    cloud->is_dense = true;
    cloud->data.reserve((u64)get_lidar_capacity(config) * LIVOX_POINT_STEP);
}

// As the driver fills it: the header stamp is the first point's time, and each point's
// timestamp is its own absolute time in ns, as a double.
static void publish_lidar_frame(Lidar_Frame_Header *frame)
{
    sensor_msgs::msg::PointCloud2 *cloud = &ros.cloud;
    const Lidar_Point *points = get_lidar_slot_points(frame);
    cloud->header.stamp = make_stamp(frame->count ? points[0].time_ns : frame->start_ns);
    cloud->width = frame->count;
    cloud->row_step = frame->count * LIVOX_POINT_STEP;
    cloud->data.resize((u64)frame->count * LIVOX_POINT_STEP);
    u8 *out = cloud->data.data();
    for (u32 i = 0; i < frame->count; i++, out += LIVOX_POINT_STEP) {
        const Lidar_Point *point = &points[i];
        f64 timestamp = (f64)point->time_ns;
        memcpy(out + 0, &point->x, 4);
        memcpy(out + 4, &point->y, 4);
        memcpy(out + 8, &point->z, 4);
        memcpy(out + 12, &point->intensity, 4);
        out[16] = point->tag;
        out[17] = point->line;
        memcpy(out + 18, &timestamp, 8);
    }
    ros.lidar_publisher->publish(*cloud);
}

// Like the Mid-360's: no orientation (the driver leaves it at its default) and acceleration
// in g unless the config says otherwise.
static void publish_imu_sample(const Imu_Sample *sample)
{
    sensor_msgs::msg::Imu message;
    message.header.stamp = make_stamp(sample->stamp_ns);
    message.header.frame_id = ros.config->imu.frame;
    message.angular_velocity.x = sample->angular_velocity.x;
    message.angular_velocity.y = sample->angular_velocity.y;
    message.angular_velocity.z = sample->angular_velocity.z;
    message.linear_acceleration.x = sample->linear_acceleration.x;
    message.linear_acceleration.y = sample->linear_acceleration.y;
    message.linear_acceleration.z = sample->linear_acceleration.z;
    ros.imu_publisher->publish(message);
}

// The CameraInfo topic next to an image topic, as camera drivers name it:
// /camera/camera/color/image_raw -> /camera/camera/color/camera_info.
static std::string get_camera_info_topic(const char *image_topic)
{
    const char *slash = strrchr(image_topic, '/');
    std::string topic(image_topic, slash ? (u64)(slash - image_topic + 1) : 0);
    return topic + "camera_info";
}

static void init_camera_messages(sensor_msgs::msg::Image *image, sensor_msgs::msg::CameraInfo *info,
                                 const Camera_Intrinsics *camera, const char *frame,
                                 const char *encoding, u32 bytes_per_pixel)
{
    image->header.frame_id = frame;
    image->width = camera->width;
    image->height = camera->height;
    image->encoding = encoding;
    image->is_bigendian = false;
    image->step = camera->width * bytes_per_pixel;
    image->data.resize((u64)image->step * camera->height);

    info->header.frame_id = frame;
    info->width = camera->width;
    info->height = camera->height;
    info->distortion_model = "plumb_bob";
    info->d.assign(5, 0.0);
    info->k = {camera->fx, 0.0, camera->cx, 0.0, camera->fy, camera->cy, 0.0, 0.0, 1.0};
    info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info->p = {camera->fx, 0.0, camera->cx, 0.0, 0.0, camera->fy,
               camera->cy, 0.0, 0.0,        0.0, 1.0, 0.0};
}

// One frame of a stream, with its CameraInfo carrying the same stamp.
static void publish_camera_frame(Camera_Frame_Header *frame, sensor_msgs::msg::Image *image,
                                 sensor_msgs::msg::CameraInfo *info,
                                 rclcpp::Publisher<sensor_msgs::msg::Image> *image_publisher,
                                 rclcpp::Publisher<sensor_msgs::msg::CameraInfo> *info_publisher)
{
    image->header.stamp = info->header.stamp = make_stamp(frame->stamp_ns);
    memcpy(image->data.data(), (u8 *)(frame + 1), image->data.size());
    image_publisher->publish(*image);
    info_publisher->publish(*info);
}

static void add_static_transform(tf2_msgs::msg::TFMessage *message, const char *parent,
                                 const char *child, b3Transform transform)
{
    geometry_msgs::msg::TransformStamped stamped;
    stamped.header.frame_id = parent;
    stamped.child_frame_id = child;
    stamped.transform.translation.x = transform.p.x;
    stamped.transform.translation.y = transform.p.y;
    stamped.transform.translation.z = transform.p.z;
    stamped.transform.rotation.x = transform.q.v.x;
    stamped.transform.rotation.y = transform.q.v.y;
    stamped.transform.rotation.z = transform.q.v.z;
    stamped.transform.rotation.w = transform.q.s;
    message->transforms.push_back(stamped);
}

// The camera's own frames on /tf_static, as realsense2_camera publishes them (publish_tf):
// <name>_link to each enabled stream's frame, and that to its optical frame.
// Latched, so late subscribers still get them. Nothing else goes on TF.
static void publish_camera_transforms(const Camera_Config *camera)
{
    char link[CONFIG_STRING_SIZE + 32], depth[CONFIG_STRING_SIZE + 32];
    char depth_optical[CONFIG_STRING_SIZE + 32], color[CONFIG_STRING_SIZE + 32];
    char color_optical[CONFIG_STRING_SIZE + 32];
    make_camera_frame_name(camera->name, "_link", link, sizeof(link));
    make_camera_frame_name(camera->name, "_depth_frame", depth, sizeof(depth));
    make_camera_frame_name(camera->name, "_depth_optical_frame", depth_optical,
                           sizeof(depth_optical));
    make_camera_frame_name(camera->name, "_color_frame", color, sizeof(color));
    make_camera_frame_name(camera->name, "_color_optical_frame", color_optical,
                           sizeof(color_optical));
    const b3Transform identity = {{0.0f, 0.0f, 0.0f}, {{0.0f, 0.0f, 0.0f}, 1.0f}};
    b3Transform color_offset = identity;
    color_offset.p = v3{camera->color_offset[0], camera->color_offset[1], camera->color_offset[2]};
    b3Transform optical = get_optical_transform(v3{0.0f, 0.0f, 0.0f});

    tf2_msgs::msg::TFMessage message;
    if (camera->depth.enabled) {
        add_static_transform(&message, link, depth, identity);
        add_static_transform(&message, depth, depth_optical, optical);
    }
    if (camera->color.enabled) {
        add_static_transform(&message, link, color, color_offset);
        add_static_transform(&message, color, color_optical, optical);
    }
    ros.static_transform_publisher = ros.node->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).reliable().transient_local());
    ros.static_transform_publisher->publish(message);
}

// Publishers only for the sensors the config enables, on their drivers' topics. Reliable,
// so best-effort subscribers (as sensor QoS asks) match too.
static void create_sensor_publishers(const Sim_Config *config)
{
    rclcpp::Node *node = ros.node.get();
    if (config->imu.enabled) {
        ros.imu_publisher =
            node->create_publisher<sensor_msgs::msg::Imu>(config->imu.topic, rclcpp::QoS(100));
    }
    if (config->lidar.enabled) {
        ros.lidar_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            config->lidar.topic, rclcpp::QoS(10));
        init_livox_cloud(&ros.cloud, &config->lidar);
    }
    // The camera renders only with the window; its topics exist whenever it is enabled.
    const Camera_Config *camera = &config->camera;
    if (is_camera_enabled(camera)) {
        publish_camera_transforms(camera);
    }
    if (camera->color.enabled) {
        const Camera_Stream_Config *color = &camera->color;
        Camera_Intrinsics intrinsics = make_camera_intrinsics(
            color->resolution[0], color->resolution[1], color->horizontal_fov);
        char frame[CONFIG_STRING_SIZE + 32];
        make_camera_frame_name(camera->name, "_color_optical_frame", frame, sizeof(frame));
        init_camera_messages(&ros.color_image, &ros.color_info, &intrinsics, frame, "rgb8", 3);
        ros.color_publisher =
            node->create_publisher<sensor_msgs::msg::Image>(color->topic, rclcpp::QoS(2));
        ros.color_info_publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            get_camera_info_topic(color->topic), rclcpp::QoS(2));
    }
    if (camera->depth.enabled) {
        const Camera_Stream_Config *depth = &camera->depth;
        Camera_Intrinsics intrinsics = make_camera_intrinsics(
            depth->resolution[0], depth->resolution[1], depth->horizontal_fov);
        char frame[CONFIG_STRING_SIZE + 32];
        make_camera_frame_name(camera->name, "_depth_optical_frame", frame, sizeof(frame));
        init_camera_messages(&ros.depth_image, &ros.depth_info, &intrinsics, frame, "16UC1", 2);
        ros.depth_publisher =
            node->create_publisher<sensor_msgs::msg::Image>(depth->topic, rclcpp::QoS(2));
        ros.depth_info_publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            get_camera_info_topic(depth->topic), rclcpp::QoS(2));
    }
}

// Everything the sim and renderer have produced since the last pass.
static void publish_sensors(void)
{
    Imu_Sample sample;
    while (pop_ring_buffer(&ros.shared->imu_samples, &sample)) {
        if (ros.imu_publisher) {
            publish_imu_sample(&sample);
        }
    }
    Lidar_Frame_Header *lidar = (Lidar_Frame_Header *)read_triple_buffer(&ros.shared->lidar_frames);
    if (lidar && ros.lidar_publisher) {
        publish_lidar_frame(lidar);
    }
    Camera_Frame_Header *color =
        (Camera_Frame_Header *)read_triple_buffer(&ros.shared->color_frames);
    if (color && ros.color_publisher) {
        publish_camera_frame(color, &ros.color_image, &ros.color_info, ros.color_publisher.get(),
                             ros.color_info_publisher.get());
    }
    Camera_Frame_Header *depth =
        (Camera_Frame_Header *)read_triple_buffer(&ros.shared->depth_frames);
    if (depth && ros.depth_publisher) {
        publish_camera_frame(depth, &ros.depth_image, &ros.depth_info, ros.depth_publisher.get(),
                             ros.depth_info_publisher.get());
    }
}

// Waits up to 1 ms for ROS work (parameter services), then
// publishes whatever the sim has produced since the last pass. Publishing happens here
// rather than on the physics thread because DDS can block.
static void *run_ros_thread(void *)
{
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ros.node);
    while (!is_quit_requested(ros.shared)) {
        executor.spin_once(std::chrono::milliseconds(1));
        publish_clock();
        publish_sensors();
    }
    executor.remove_node(ros.node);
    return NULL;
}

bool start_ros_thread(Shared_Global_State *shared, const Sim_Config *config)
{
    ros.shared = shared;
    ros.config = config;
    create_sensor_publishers(config);
    if (pthread_create(&ros.thread, NULL, run_ros_thread, NULL) != 0) {
        log_error("ros: could not start the ROS thread");
        return false;
    }
    ros.thread_started = true;
    return true;
}

void destroy_ros_bridge(void)
{
    if (ros.thread_started) {
        request_quit(ros.shared);
        pthread_join(ros.thread, NULL);
        ros.thread_started = false;
    }
    ros.clock_publisher.reset();
    ros.imu_publisher.reset();
    ros.lidar_publisher.reset();
    ros.color_publisher.reset();
    ros.depth_publisher.reset();
    ros.color_info_publisher.reset();
    ros.depth_info_publisher.reset();
    ros.static_transform_publisher.reset();
    ros.node.reset();
    rclcpp::shutdown();
}
