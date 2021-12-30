#include <tinyfsm.hpp>
#include "lift_motor.hpp"
#include <iostream>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std;

// ----------------------------------------------------------------------------
// Motor states
//
class LiftRunning;

class LiftStop
: public LiftMotor
{
  void entry() override {
    ROS_INFO("LIFT Motor State: Stop");
  }
  void react(Brake const &) override{
    // Do nothing
  }
};


class LiftIdle
: public LiftMotor
{
  void entry() override {
    ROS_INFO("LIFT Motor State: Idle");
  };
  void react(LiftInitialize const & e) override{
    LiftMotor::controller = e.controller;
    bool result = LiftMotor::controller->liftMotorInitialize();
    if (result == true)
    {
      transit<LiftStop>();
    }else
    {
      ROS_ERROR("Failed to initialize LIFT motors.");
    }
  }
  void react(TorqueOff const &){
    //do nothing
  }
};



// ----------------------------------------------------------------------------
// Base State: default implementations


void LiftMotor::react(Brake   const &) {
  cout << "Brake Failed" << endl;
}

void LiftMotor::react(SetLiftPosition const &) {
  cout << "Set Position Failed" << endl;
}

void LiftMotor::react(LiftInitialize const &) {
  cout << "LIFT Initialization Failed" << endl;
}

void LiftMotor::react(TorqueOff const &){
  ROS_INFO("Lift Event torque off takes effect" );
  LiftMotor::controller->torqueOff();
  transit<LiftIdle>();
}

MultiMotorController *LiftMotor::controller = nullptr;

// ----------------------------------------------------------------------------
// Initial state definition
//

FSM_INITIAL_STATE(LiftMotor,LiftIdle)
