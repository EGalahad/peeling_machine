#pragma once

/**
 * @brief State control utils.
 * @details
 * The external and internal signals control the state of the machine,
 * and the states should sufficient for the motor control loop to determine
 * their move.
 *
 */

enum class State {
    STOPPED,
    // RUNNING,
    CLOSE,
    UP,
    PEEL,
    // EXITING,
    DOWN,
    OPEN,
    
    CLEANING,

    ERROR
};

enum class Signal {
    // when the start-stop button is pressed
    BUTTON_START_STOP,
    // when the self cleaning button is pressed
    BUTTON_SELF_CLEAN,

    // when the pressure reaches the upper threshold
    // PRESSURE_TOUCH_FRUIT,
    // when the pressure reaches the target
    // PRESSURE_TARGET,
    // when the pressure exceeds the lower threshold, for emergency stop
    PRESSURE_LOWER_THRESHOLD_EXCEEDED,

    // when the up height is reached
    // PEEL_START,
    // when the peeling process finishes
    // PEEL_FINISH,
};

/***
 * The state machine should be expose a method to handle signals and does the
 * state transition under the hood. When transition to a new state, the state
 * machine is also responsible for initializing required state variables and
 * initial motor states.
 *
 * This impose the requirement of sharing MotorControl as global variable,
 * because both the main.cpp and state_machine.cpp will need to take control
 * over the motor.
 */
class StateMachine {
  public:
    StateMachine(State initialState) : state(initialState) {}
    void trigger_signal(Signal signal);
    State next();
    const State get_state() const { return state; }

  private:
    void start_stop();
    void self_clean();
    void pressure_err();
    State state;
};
