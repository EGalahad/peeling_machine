#pragma once

/**
 * @brief State control utils.
 * @details
 * The external and internal signals control the state of the machine,
 * and the states should sufficient for the motors to determine their move.
 * 
 * Instead of some controling details:
 * 1. pressure control in UP and DOWN
 */

enum class State {
    STOPPED,
    // RUNNING,
    CLOSE,
    UP,
    // EXITING,
    DOWN,
    OPEN,
    // CLEANING,
    CLEANING
};

enum class Signal {
    // when the start-stop button is pressed
    START_STOP,
    // when the self cleaning button is pressed
    SELF_CLEANING,

    // when the pressure sensor exceeds the upper threshold
    PRESSURE_UPPER_THRESHOLD_EXCEEDED,
    // when the pressure sensor exceeds the lower threshold
    PRESSURE_LOWER_THRESHOLD_EXCEEDED,

    // when the lower clamping blade reached the height of the peeling blade
    UP_LIMIT_EXCEEDED,
    // when both the upper and lower screw rods reached the end
    DOWN_LIMIT_EXCEEDED,
};

class StateMachine {
  public:
    StateMachine(State initialState) : state(initialState) {}
    void signal(Signal signal);

  private:
    void start_stop();
    State state;
};
