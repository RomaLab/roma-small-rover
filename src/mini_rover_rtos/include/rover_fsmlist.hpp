#ifndef ROVER_FSMLIST_HPP_INCLUDED
#define ROVER_FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>

#include "wheel_motor.hpp"
#include "steer_motor.hpp"
#include "lift_motor.hpp"
#include "rover_state.hpp"

using fsm_list = tinyfsm::FsmList<WheelMotor,SteerMotor,LiftMotor,RoverState>;
// using fsm_list = tinyfsm::FsmList<RoverState>;

/** dispatch event to both "Motor" and "Elevator" */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}


#endif
