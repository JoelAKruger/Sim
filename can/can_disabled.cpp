#include "can/can_bridge.h"

// Linked instead of can_bridge.cpp when built with REGOLITH_WITH_CAN=OFF. The sim runs the
// same; its actuators just never receive commands, so they hold.

bool start_can_thread(Shared_Global_State *shared, const Sim_Config *config, const Robot *robot,
                      const Actuator_Set *actuators)
{
    (void)shared;
    (void)robot;
    (void)actuators;
    if (config->can_interface[0]) {
        log_info("built without CAN: ignoring can.interface %s", config->can_interface);
    }
    return true;
}

void stop_can_thread(void) {}

bool is_can_running(void) { return false; }
