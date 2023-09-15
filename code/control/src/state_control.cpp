#include <Arduino.h>

#include "motor_utils.hpp"
#include "state_control.hpp"

#include "constants.hpp"
#include "globals.hpp"

void StateMachine::trigger_signal(Signal signal) {
    switch (signal) {
    case Signal::BUTTON_START_STOP:
        start_stop();
        break;

    case Signal::BUTTON_SELF_CLEAN:
        self_clean();
        break;

    case Signal::PRESSURE_LOWER_THRESHOLD_EXCEEDED:
        pressure_err();
        break;

    default:
        break;
    }
}

void close_init_state();
void up_init_state();
void peel_init_state();
void down_init_state();
void open_init_state();

State StateMachine::next() {
    switch (state) {
    case State::CLOSE:
        up_init_state();
        state = State::UP;
        break;

    case State::UP:
        peel_init_state();
        state = State::PEEL;
        break;

    case State::PEEL:
        down_init_state();
        state = State::DOWN;
        break;

    case State::DOWN:
        open_init_state();
        state = State::OPEN;
        break;

    case State::OPEN:
        state = State::STOPPED;
        break;

    default:
        break;
    }
    return state;
}

void StateMachine::start_stop() {
    switch (state) {
    // from STOPPED to RUNNING
    case State::STOPPED:
        close_init_state();
        state = State::CLOSE;
        break;

    // from RUNNING to EXITING
    case State::CLOSE:
        open_init_state();
        state = State::OPEN;
        break;

    case State::UP:
    case State::PEEL:
        down_init_state();
        state = State::DOWN;
        break;

    default:
        break;
    }
}

void StateMachine::self_clean() {
    switch (state) {
    case State::STOPPED:
        state = State::CLEANING;
        break;

    default:
        break;
    }
}

void StateMachine::pressure_err() {
    motor_upper.set_direction(Direction::STOP);
    motor_lower.set_direction(Direction::STOP);
    motor_peel.set_direction(Direction::STOP);
    motor_rotate_lower.set_direction(Direction::STOP);
    state = State::ERROR;
}

void close_init_state() {
    // init state variable
    close_upper = true;

    // init motor state
    motor_upper.set_direction(Direction::DOWN);
    motor_peel.set_direction(Direction::DOWN);
    motor_lower.set_direction(Direction::STOP);
}

void up_init_state() {
    // init state variable
    height_remaining_upper = up_height;
    height_remaining_lower = up_height;
    height_remaining_peel = up_height;

    // init motor state
    motor_upper.set_direction(Direction::UP);
    motor_peel.set_direction(Direction::UP);
    motor_lower.set_direction(Direction::UP);
}

void peel_init_state() {
    // init state variable
    height_remaining_peel = peel_height;

    // init motor state
    motor_peel.set_direction(Direction::DOWN);
    motor_rotate_lower.set_direction(Direction::UP);
}

void down_init_state() {
    // init motor state
    motor_upper.set_direction(Direction::DOWN);
    motor_lower.set_direction(Direction::DOWN);
}

void open_init_state() {
    // // init state variable
    // open_upper = true;
    // open_lower = true;
    // open_peel = true;

    // init motor state
    motor_upper.set_direction(Direction::UP);
    motor_peel.set_direction(Direction::UP);
    motor_lower.set_direction(Direction::DOWN);
}


