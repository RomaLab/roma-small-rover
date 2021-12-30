#include <tinyfsm.hpp>
#include "steer_motor.hpp"
#include <iostream>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

// ----------------------------------------------------------------------------
// Motor states
//
class SteerRunning;

class SteerStop
: public SteerMotor
{
  void entry() override {
    ROS_INFO("Steer Motor State: Stop");
  }
  void react(SetSteerPosition const &e) override{
    // Set goal position
    controller->setGoalPosition(e.goalPosition);
    // transit to running
    transit<SteerRunning>();
  }
  void react(Brake const &) override{
    // Do nothing
  }
};

class SteerRunning
: public SteerMotor
{
  void entry() override {
    ROS_INFO("Steer Motor State: Running");
  }
  void react(SetSteerPosition const &e) override{
    // Set goal position
    controller->setGoalPosition(e.goalPosition);
  }
  void react(Brake const &e) override{
    // TODO: position to current position
    transit<SteerStop>();
  };
};


class SteerIdle
: public SteerMotor
{
  void entry() override {
    ROS_INFO("Steer Motor State: Idle");
  };
  void react(SteerInitialize const & e) override{
    SteerMotor::controller = e.controller;
    bool result = SteerMotor::controller->initialize();
    if (result == true)
    {
      transit<SteerStop>();
    }else
    {
      ROS_ERROR("Failed to initialize steer motors.");
    }
  }
  void react(TorqueOff const &){
    //do nothing
  }
};



// ----------------------------------------------------------------------------
// Base State: default implementations
//

void SteerMotor::react(Brake   const &) {
  cout << "Brake Failed" << endl;
}

void SteerMotor::react(SetSteerPosition const &) {
  cout << "Set Position Failed" << endl;
}

void SteerMotor::react(SteerInitialize const &) {
  cout << "Steer Initialization Failed" << endl;
}

void SteerMotor::react(TorqueOff const &){
  ROS_INFO("Steer Event torque off takes effect" );
  SteerMotor::controller->torqueOff();
  transit<SteerIdle>();
}

MultiMotorController *SteerMotor::controller = nullptr;

// ----------------------------------------------------------------------------
// Initial state definition
//

FSM_INITIAL_STATE(SteerMotor,SteerIdle)
