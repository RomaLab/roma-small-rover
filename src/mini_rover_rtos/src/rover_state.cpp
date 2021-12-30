
#include <tinyfsm.hpp>
#include <iostream>
#include <ros/ros.h>

#include "rover_fsmlist.hpp"
#include "rover_state.hpp"
#include "motor_event.hpp"
// #include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

// ----------------------------------------------------------------------------
// Motor states
//
class RoverStop;
class Experiment1;
class Experiment2;
class Experiment3;

class RoverIdle
    : public RoverState
{
    void entry() override
    {
        ROS_INFO("Rover State: Idle");
    }
    void react(RoverInitialize const &e) override
    {
        WheelInitialize wheelIni;
        wheelIni.controller = e.wheel_controller;
        SteerInitialize steerIni;
        steerIni.controller = e.steer_controller;
        LiftInitialize liftIni;
        liftIni.controller = e.lift_controller;
        send_event(wheelIni);
        send_event(steerIni);
        send_event(liftIni);

        SetSteerPosition ssp;
        ssp.goalPosition[0]=2048;
        ssp.goalPosition[1]=2048;
        ssp.goalPosition[2]=2048;
        ssp.goalPosition[3]=2048;
        send_event(ssp);

        transit<RoverStop>();
    }
    void react(RoverTorqueOff const &)
    {
        //do nothing
    }
    void react(RoverBrake const &)
    {
        //do nothing
    }
};

class RoverStop
    : public RoverState
{
    void entry() override
    {
        send_event(Brake());
        ROS_INFO("Rover State: Stop");
    }
    void react(ExecExperiment1 const&){
        transit<Experiment1>();
    }
    void react(ExecExperiment2 const&){
        transit<Experiment2>();
    }
    void react(ExecExperiment3 const&){
        transit<Experiment3>();
    }
};

class Experiment1 // 直线前进
    : public RoverState
{
    void entry() override
    {
        ROS_INFO("Rover State: Experiment1");
        // 设置方向
        SetSteerPosition ssp;
        ssp.goalPosition[0]=2048;
        ssp.goalPosition[1]=2048;
        ssp.goalPosition[2]=2048;
        ssp.goalPosition[3]=2048;
        send_event(ssp);
        // 正向前进
        SetWheelSpeed sws;
        sws.goalSpeed[0]=100;
        sws.goalSpeed[1]=100;
        sws.goalSpeed[2]=100;
        sws.goalSpeed[3]=100;
        send_event(sws);
        sleep(3);
        send_event(Brake());
        ROS_INFO("Rover Stop Move");
    };
    void react(BackToCenter const&){
        SetWheelSpeed sws;
        sws.goalSpeed[0]=-100;
        sws.goalSpeed[1]=-100;
        sws.goalSpeed[2]=-100;
        sws.goalSpeed[3]=-100;
        send_event(sws);
        sleep(3);
        send_event(Brake());
        transit<RoverStop>();
    }
};

class Experiment2 // 车轮转向
    : public RoverState
{
    void entry() override
    {
        ROS_INFO("Rover State: Experiment2");
        // 设置方向
        SetSteerPosition ssp;
        ssp.goalPosition[0]=1024;
        ssp.goalPosition[1]=1024;
        ssp.goalPosition[2]=1024;
        ssp.goalPosition[3]=1024;
        send_event(ssp);
    };
    void react(BackToCenter const&){
        SetSteerPosition ssp;
        ssp.goalPosition[0]=2048;
        ssp.goalPosition[1]=2048;
        ssp.goalPosition[2]=2048;
        ssp.goalPosition[3]=2048;
        send_event(ssp);
        transit<RoverStop>();
    }
};

class Experiment3 // 车轮转向
    : public RoverState
{
    void entry() override
    {
        ROS_INFO("Rover State: Experiment3");
        // 设置方向
        SetSteerPosition ssp;
        ssp.goalPosition[0]=1024;
        ssp.goalPosition[1]=1024;
        ssp.goalPosition[2]=1024;
        ssp.goalPosition[3]=1024;
        send_event(ssp);
        // 设置速度
        SetWheelSpeed sws;
        sws.goalSpeed[0]=100;
        sws.goalSpeed[1]=100;
        sws.goalSpeed[2]=100;
        sws.goalSpeed[3]=100;
        send_event(sws);
        sleep(3);
        send_event(Brake());
    };
    void react(BackToCenter const&){
        SetWheelSpeed sws;
        sws.goalSpeed[0]=-100;
        sws.goalSpeed[1]=-100;
        sws.goalSpeed[2]=-100;
        sws.goalSpeed[3]=-100;
        send_event(sws);
        sleep(3);
        send_event(Brake());
        transit<RoverStop>();
    }
};

// ----------------------------------------------------------------------------
// Base State: default implementations

// void RoverState::react(SetRoverSpeed   const &) {
//     ROS_INFO("set speed failed");
// }

void RoverState::react(RoverInitialize const &)
{
    ROS_INFO("initialze failed");
}

void RoverState::react(RoverTorqueOff const &)
{
    send_event(TorqueOff());
    ROS_INFO("Rover Torque Off");
    transit<RoverIdle>();
}

void RoverState::react(ExecExperiment1 const &)
{
}

void RoverState::react(ExecExperiment2 const &)
{
}

void RoverState::react(ExecExperiment3 const &)
{
}

void RoverState::react(BackToCenter const &)
{
}

void RoverState::react(RoverBrake const &)
{
    transit<RoverStop>();
}
// ----------------------------------------------------------------------------
// Initial state definition
//

FSM_INITIAL_STATE(RoverState, RoverIdle)
