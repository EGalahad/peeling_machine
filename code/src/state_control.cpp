#include <Arduino.h>

#include "motor_utils.hpp"
#include "state_control.hpp"

void StateMachine::start_stop() {
    switch (state) {
    case State::STOPPED:
        state = State::CLOSE;
        break;
        case State::CLOSE:
        state = State::OPEN;
        break;
    case State::UP:
        state = State::DOWN;
        break;
    case State::DOWN:
        state = State::UP;
        break;
    case State::OPEN:
        state = State::CLOSE;
        break;
    case State::CLEANING:
        state = State::STOPPED;
        break;

    default:
        break;
    }
}
