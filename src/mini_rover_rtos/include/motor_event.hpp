#ifndef MOTOR_EVENT_HPP_INCLUDED
#define MOTOR_EVENT_HPP_INCLUDED

#include <tinyfsm.hpp>
#include "multi_motor_controller.hpp"

struct MotorEvent : tinyfsm::Event
{
};

struct SetWheelSpeed : MotorEvent
{
    int32_t goalSpeed[4];
};

struct SetSteerPosition : MotorEvent
{
    int32_t goalPosition[4];
};

struct SetLiftPosition : MotorEvent
{
    int32_t goalPosition[4];
};

struct WheelInitialize : MotorEvent
{
    MultiMotorController *controller;
};
struct SteerInitialize : MotorEvent
{
    MultiMotorController *controller;
};
struct LiftInitialize : MotorEvent
{
    MultiMotorController *controller;
};

struct Brake : MotorEvent
{
};
struct TorqueOff : MotorEvent
{
};
#endif