#ifndef LIFT_FSMLIST_HPP_INCLUDED
#define LIFT_FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>

#include "lift_motor.hpp"

using fsm_list = tinyfsm::FsmList<LiftMotor>;

/** dispatch event to both "Motor" and "Elevator" */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}


#endif
