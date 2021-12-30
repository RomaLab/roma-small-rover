#ifndef ROVER_STATE_HPP_INCLUDED
#define ROVER_STATE_HPP_INCLUDED

#include <tinyfsm.hpp>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "multi_motor_controller.hpp"
#include "rover_event.hpp"
// ----------------------------------------------------------------------------
// Event declarations
//


// ----------------------------------------------------------------------------
// Motor (FSM base class) declaration

class RoverState
: public tinyfsm::Fsm<RoverState>
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
  // virtual void react(SetRoverSpeed const &);
  virtual void react(RoverInitialize const &);
  virtual void react(RoverTorqueOff const &);
  virtual void react(ExecExperiment1 const &);
  virtual void react(ExecExperiment2 const &);
  virtual void react(ExecExperiment3 const &);
  virtual void react(BackToCenter const &);
  virtual void react(RoverBrake const &);
  virtual void entry(void) { };  /* pure virtual: enforce implementation in all states */
  void exit(void)  { };  /* no exit actions at all */

};



#endif
