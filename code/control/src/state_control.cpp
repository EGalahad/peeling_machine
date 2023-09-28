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
        Serial.println("Entering UP state.");
        break;

    case State::UP:
        peel_init_state();
        state = State::PEEL;
        Serial.println("Entering PEEL state.");
        break;

    case State::PEEL:
        down_init_state();
        state = State::DOWN;
        Serial.println("Entering DOWN state.");
        break;

    case State::DOWN:
        open_init_state();
        state = State::OPEN;
        Serial.println("Entering OPEN state.");
        break;

    case State::OPEN:
        state = State::STOPPED;
        Serial.println("Entering STOPPED state.");
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
        Serial.println("Entering CLOSE state.");
        break;

    // from RUNNING to EXITING
    case State::CLOSE:
        open_init_state();
        state = State::OPEN;
        Serial.println("Entering OPEN state.");
        break;

    case State::UP:
    case State::PEEL:
        down_init_state();
        state = State::DOWN;
        Serial.println("Entering DOWN state.");
        break;

    default:
        break;
    }
}

void StateMachine::self_clean() {
    switch (state) {
    case State::STOPPED:
        state = State::CLEANING;
        Serial.println("Entering CLEANING state.");
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
    height_to_peel = height_total;

    // init motor state
    motor_upper.set_direction(Direction::DOWN);
    motor_peel.set_direction(Direction::DOWN);
    motor_lower.set_direction(Direction::STOP);
}

void up_init_state() {
    // init state variable
    height_upper = 0;
    height_lower = 0;
    height_peel = 0;

    // init motor state
    motor_upper.set_direction(Direction::UP);
    motor_peel.set_direction(Direction::UP);
    motor_lower.set_direction(Direction::UP);
}

void peel_init_state() {
    Serial.println();
    Serial.println("peel_init_state");

    // init motor state
    motor_peel.set_direction(Direction::DOWN);
    motor_rotate_lower.set_direction(Direction::UP);
}

void down_init_state() {
    // init state variable
    peel_down_downwards = height_peel > 0;
    motor_peel.set_direction(peel_down_downwards ? Direction::DOWN : Direction::UP);

    // init motor state
    motor_upper.set_direction(Direction::DOWN);
    motor_lower.set_direction(Direction::DOWN);
}

void open_init_state() {
    Serial.println();
    Serial.println("open_init_state");

    // init motor state
    motor_upper.set_direction(Direction::UP);
    motor_peel.set_direction(Direction::UP);
    motor_lower.set_direction(Direction::DOWN);
}
