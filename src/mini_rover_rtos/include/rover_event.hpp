#ifndef ROVER_EVENT_HPP_INCLUDED
#define ROVER_EVENT_HPP_INCLUDED

#include <tinyfsm.hpp>
#include "multi_motor_controller.hpp"

struct RoverEvent: tinyfsm::Event {};

struct SetRoverSpeed : RoverEvent
{
int32_t goalSpeed;
};

struct RoverInitialize : RoverEvent{
    MultiMotorController *wheel_controller;
    MultiMotorController *steer_controller;
    MultiMotorController *lift_controller;
};

struct RoverBrake : RoverEvent {};
struct RoverTorqueOff : RoverEvent {};
struct ExecExperiment1: RoverEvent {};
struct ExecExperiment2: RoverEvent {};
struct ExecExperiment3: RoverEvent {};
struct BackToCenter: RoverEvent {};

#endif