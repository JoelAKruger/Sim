#include "can/can_bridge.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "can/blcmd.h"
#include "can/led_strip.h"

#define MAX_BLCMDS 16

struct Can_Bridge {
    i32 socket;
    pthread_t thread;
    bool started;
    Shared_Global_State *shared;
    char interface[IFNAMSIZ];
    Blcmd_Node nodes[MAX_BLCMDS]; // the robot's BLCMDs
    u32 node_count;
    Led_Strip_State led_strip;
    bool warned_node[BLCMD_LAST_NODE + 1]; // commands for a BLCMD the robot lacks, once each
    bool warned_function[BLCMD_FUNCTION_COUNT]; // not simulated, once each
    bool warned_receive;
    u64 dropped_commands; // the sim fell behind and the command ring was full
};

static Can_Bridge can_bridge = {.socket = -1};

static void handle_led_strip_frame(const Can_Frame *frame)
{
    Led_Strip_Command command;
    if (!decode_led_strip_frame(frame, &command)) {
        return;
    }
    apply_led_strip_command(&can_bridge.led_strip, &command);
    Status_Light *slot =
        (Status_Light *)get_triple_buffer_write_slot(&can_bridge.shared->status_light);
    *slot = get_led_strip_light(&can_bridge.led_strip);
    publish_triple_buffer(&can_bridge.shared->status_light);
}

static void handle_blcmd_frame(const Can_Frame *frame)
{
    u32 node_id = get_frame_node(frame);
    const Blcmd_Node *node = find_blcmd_node(can_bridge.nodes, can_bridge.node_count, node_id);
    Blcmd_Message message = decode_blcmd_frame(frame, node);
    if (message.kind == BLCMD_MESSAGE_UNSUPPORTED) {
        if (!can_bridge.warned_function[message.function]) {
            log_warning("can: BLCMD function %u (%s) isn't simulated; ignored", message.function,
                        get_blcmd_function_name(message.function));
            can_bridge.warned_function[message.function] = true;
        }
    } else if (message.kind == BLCMD_MESSAGE_COMMAND && !node) {
        if (!can_bridge.warned_node[message.node_id]) {
            log_warning("can: command for BLCMD %u, which the robot doesn't have; ignored",
                        message.node_id);
            can_bridge.warned_node[message.node_id] = true;
        }
    } else if (message.kind == BLCMD_MESSAGE_COMMAND &&
               !push_ring_buffer(&can_bridge.shared->actuator_commands, &message.command)) {
        can_bridge.dropped_commands++;
    }
}

// Each device on the bus has its own node id and protocol.
static void handle_frame(const Can_Frame *frame)
{
    if (is_blcmd_frame(frame)) {
        handle_blcmd_frame(frame);
    } else if (is_led_strip_frame(frame)) {
        handle_led_strip_frame(frame);
    }
}

static void receive_frames(void)
{
    for (;;) {
        can_frame raw;
        ssize_t size = read(can_bridge.socket, &raw, sizeof(raw));
        if (size < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && !can_bridge.warned_receive) {
                log_warning("can: reading %s: %s", can_bridge.interface, strerror(errno));
                can_bridge.warned_receive = true;
            }
            return;
        }
        if (size != sizeof(raw) || (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))) {
            continue;
        }
        Can_Frame frame = {.id = raw.can_id & CAN_SFF_MASK, .length = (u8)min((u32)raw.len, 8u)};
        memcpy(frame.data, raw.data, frame.length);
        handle_frame(&frame);
    }
}

// Blocks in poll until frames arrive, waking every 10 ms to notice quit. Nothing is sent.
static void *run_can_thread(void *)
{
    while (!is_quit_requested(can_bridge.shared)) {
        pollfd readable = {.fd = can_bridge.socket, .events = POLLIN};
        if (poll(&readable, 1, 10) > 0) {
            receive_frames();
        }
    }
    return NULL;
}

static bool open_socket(const char *interface)
{
    u32 index = if_nametoindex(interface);
    if (index == 0) {
        log_warning("can: no interface %s, so no CAN. For a virtual one: sudo modprobe vcan && "
                    "sudo ip link add dev %s type vcan && sudo ip link set up %s",
                    interface, interface, interface);
        return false;
    }
    can_bridge.socket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (can_bridge.socket < 0) {
        log_warning("can: no SocketCAN socket (%s), so no CAN", strerror(errno));
        return false;
    }
    // Nova device commands are standard-id data frames with bits 8..10 clear; the kernel
    // drops everything else before the thread sees it.
    can_filter filter = {.can_id = 0, .can_mask = 0x700 | CAN_EFF_FLAG | CAN_RTR_FLAG};
    setsockopt(can_bridge.socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
    sockaddr_can address = {};
    address.can_family = AF_CAN;
    address.can_ifindex = (int)index;
    if (bind(can_bridge.socket, (sockaddr *)&address, sizeof(address)) != 0) {
        log_warning("can: binding %s: %s, so no CAN", interface, strerror(errno));
        close(can_bridge.socket);
        can_bridge.socket = -1;
        return false;
    }
    ifreq request = {};
    snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", interface);
    if (ioctl(can_bridge.socket, SIOCGIFFLAGS, &request) == 0 && !(request.ifr_flags & IFF_UP)) {
        log_warning("can: %s is down; bring it up with: sudo ip link set up %s", interface,
                    interface);
    }
    return true;
}

// Each actuator's BLCMD mapping, from its URDF <ros2_control> parameters.
static void build_node_table(const Robot *robot, const Actuator_Set *actuators)
{
    for (u32 a = 0; a < actuators->count; a++) {
        const Robot_Actuator *actuator = &actuators->actuators[a];
        if (can_bridge.node_count == MAX_BLCMDS ||
            !make_blcmd_node(robot, actuator, &can_bridge.nodes[can_bridge.node_count])) {
            const Urdf_Joint *urdf =
                &robot->model.joints[robot->joints[actuator->joint].urdf_joint];
            log_warning("can: %s's node id %u isn't a BLCMD address (%u..%u), so it can't be "
                        "commanded",
                        urdf->name, actuator->node_id, BLCMD_FIRST_NODE, BLCMD_LAST_NODE);
            continue;
        }
        can_bridge.node_count++;
    }
}

bool start_can_thread(Shared_Global_State *shared, const Sim_Config *config, const Robot *robot,
                      const Actuator_Set *actuators)
{
    if (!config->can_interface[0]) {
        log_info("can: can.interface is empty, so no CAN");
        return true;
    }
    can_bridge.shared = shared;
    can_bridge.led_strip = get_default_led_strip_state();
    u64 name_length = strlen(config->can_interface);
    if (name_length >= sizeof(can_bridge.interface)) {
        log_warning("can: \"%s\" is longer than an interface name can be, so no CAN",
                    config->can_interface);
        return true;
    }
    memcpy(can_bridge.interface, config->can_interface, name_length + 1);
    build_node_table(robot, actuators);
    if (!open_socket(can_bridge.interface)) {
        return true;
    }
    if (pthread_create(&can_bridge.thread, NULL, run_can_thread, NULL) != 0) {
        log_error("can: could not start the CAN thread");
        close(can_bridge.socket);
        can_bridge.socket = -1;
        return false;
    }
    can_bridge.started = true;
    log_info("can: %s, %u BLCMDs and the LED strip", can_bridge.interface, can_bridge.node_count);
    return true;
}

void stop_can_thread(void)
{
    if (can_bridge.started) {
        request_quit(can_bridge.shared);
        pthread_join(can_bridge.thread, NULL);
        can_bridge.started = false;
    }
    if (can_bridge.socket >= 0) {
        close(can_bridge.socket);
        can_bridge.socket = -1;
    }
    if (can_bridge.dropped_commands) {
        log_warning("can: dropped %llu commands because the sim fell behind",
                    (unsigned long long)can_bridge.dropped_commands);
    }
}

bool is_can_running(void) { return can_bridge.started; }
