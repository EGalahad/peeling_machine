#pragma once

/**
 * @brief State control utils.
 * @details
 * The external and internal signals control the state of the machine,
 * and the states should sufficient for the motor control loop to determine
 * their move.
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

    // when the pressure exceeds the lower threshold, for emergency stop
    PRESSURE_LOWER_THRESHOLD_EXCEEDED,
};

/**
 * The state machine does all the state transition under the hood.
 * 
 * The state transition can be invoked by directly invocation of the `next()`
 * method or sending signals through the `trigger_signal()` method.
 *
 * When transition to a new state, the state machine is also responsible for
 * initializing required state variables and initial motor states.
 *
 * This impose the requirement of sharing MotorControl as global variable,
 * because both the main.cpp and state_machine.cpp will need to take control
 * over the motor.
 */
class StateMachine {
  public:
    StateMachine(State initialState) : state(initialState) {}
    /**
     * @brief Trigger some signal from outside (user/emergency) events
     *
     * @param signal (Signal): The signal emitted from some event that happened
     */
    void trigger_signal(Signal signal);
    /**
     * @brief Go to next state by the normal stages pipeline.
     *
     * @return state (State): the new state the machine is in.
     */
    State next();
    const State get_state() const { return state; }

  private:
    void start_stop();
    void self_clean();
    void pressure_err();
    State state;
};
