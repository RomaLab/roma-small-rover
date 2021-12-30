#ifndef WHEEL_FSMLIST_HPP_INCLUDED
#define WHEEL_FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>

#include "wheel_motor.hpp"

using fsm_list = tinyfsm::FsmList<WheelMotor>;

/** dispatch event to both "Motor" and "Elevator" */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}


#endif
