#include <tinyfsm.hpp>
#include "wheel_motor.hpp"
#include <iostream>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

// Wheel Motor Constructor

// WheelMotor::WheelMotor(){
// }

// ----------------------------------------------------------------------------
// Motor states
//
class WheelRunning;

class WheelStop
: public WheelMotor
{
  void entry() override {
    ROS_INFO("Wheel Motor State: Stop");
  }
  void react(SetWheelSpeed const &e) override{
    // Set goal velocity
    controller->setGoalSpeed(e.goalSpeed);
    // transit to running
    transit<WheelRunning>();
  }
  void react(Brake const &) override{
    // Do nothing
  }
};

class WheelRunning
: public WheelMotor
{
  void entry() override {
    ROS_INFO("Wheel Motor State: Running");
  }
  void react(SetWheelSpeed const &e) override{
    // Set goal velocity
    controller->setGoalSpeed(e.goalSpeed);
  }
  void react(Brake const &e) override{
    // Set velocity to zero
    controller->setSpeedZero();
    transit<WheelStop>();
  };
};


class WheelIdle
: public WheelMotor
{
  void entry() override {
    ROS_INFO("Wheel Motor State: WheelIdle");
  };
  void react(WheelInitialize const & e) override{
    WheelMotor::controller = e.controller;
    bool result = WheelMotor::controller->initialize();
    if (result == true)
    {
      transit<WheelStop>();
    }else
    {
      ROS_ERROR("Failed to initialize wheel motors.");
    }
  }
  void react(TorqueOff const &){
    //do nothing
  }
};



// ----------------------------------------------------------------------------
// Base State: default implementations
//

void WheelMotor::react(Brake   const &) {
  cout << "Brake Failed" << endl;
}

void WheelMotor::react(SetWheelSpeed const &) {
  cout << "Set Speed Failed" << endl;
}

void WheelMotor::react(WheelInitialize const &) {
  cout << "Wheel Initialization Failed" << endl;
}

void WheelMotor::react(TorqueOff const &){
  ROS_INFO("Wheel Event torque off takes effect" );
  WheelMotor::controller->torqueOff();
  transit<WheelIdle>();
}

MultiMotorController *WheelMotor::controller = nullptr;

// ----------------------------------------------------------------------------
// Initial state definition
//

FSM_INITIAL_STATE(WheelMotor,WheelIdle)
