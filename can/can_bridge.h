#pragma once

#include "core/actuator.h"
#include "core/config.h"
#include "core/shared_state.h"

// The SocketCAN adapter: the sim plays the robot's BLCMD motor controllers and LED strip on
// a CAN interface. It is optional. With REGOLITH_WITH_CAN=OFF, can_disabled.cpp provides
// these same functions as no-ops. There is one bridge per process.
//
// The CAN thread owns the socket. BLCMD frames become Actuator_Commands on
// shared->actuator_commands; LED strip frames update shared->status_light. The wire format
// lives in blcmd_protocol.h.

// Opens can.interface and starts the CAN thread. The robot's actuators, and their URDF
// parameters (reversed, zero_offset), say which BLCMDs exist and how their numbers map to
// joints. A missing interface is a warning, not an error: the sim runs without CAN. False
// only on an unexpected failure.
bool start_can_thread(Shared_Global_State *shared, const Sim_Config *config, const Robot *robot,
                      const Actuator_Set *actuators);

// Joins the thread and closes the socket.
void stop_can_thread(void);

// True once the CAN thread is listening, false with no interface or no CAN support.
bool is_can_running(void);
