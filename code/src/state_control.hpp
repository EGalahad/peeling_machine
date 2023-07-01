#pragma once

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
    START_STOP,
    SELF_CLEANING
};

class StateMachine {
  public:
    StateMachine(State initialState) : state(initialState) {}
    void start_stop();
    void setState(State newState);
    State state;
};
