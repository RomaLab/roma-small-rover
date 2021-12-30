#ifndef STEER_MOTOR_HPP_INCLUDED
#define STEER_MOTOR_HPP_INCLUDED

#include <tinyfsm.hpp>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "multi_motor_controller.hpp"
#include "motor_event.hpp"
// ----------------------------------------------------------------------------
// Event declarations
//

// ----------------------------------------------------------------------------
// Motor (FSM base class) declaration

class SteerMotor
: public tinyfsm::Fsm<SteerMotor>
{
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
public:
  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &) { };

  /* non-virtual declaration: reactions are the same for all states */
  virtual void react(Brake   const &);
  virtual void react(SetSteerPosition const &);
  virtual void react(SteerInitialize const &);
  virtual void react(TorqueOff const &);
  virtual void entry(void) { };  /* pure virtual: enforce implementation in all states */
  void exit(void)  { };  /* no exit actions at all */
  static MultiMotorController *controller;

};



#endif
