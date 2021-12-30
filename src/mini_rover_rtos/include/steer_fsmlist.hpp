#ifndef STEER_FSMLIST_HPP_INCLUDED
#define STEER_FSMLIST_HPP_INCLUDED

#include <tinyfsm.hpp>

#include "steer_motor.hpp"

using fsm_list = tinyfsm::FsmList<SteerMotor>;

/** dispatch event to both "Motor" and "Elevator" */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}


#endif
