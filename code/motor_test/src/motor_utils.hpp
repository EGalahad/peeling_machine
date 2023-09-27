#pragma once

enum Direction { UP, DOWN, STOP };

class MotorControl {
  public:
    MotorControl(const int pin_in1, const int pin_in2, const int pin_enable)
        : pin_in1(pin_in1), pin_in2(pin_in2), pin_enable(pin_enable) {}
    void set_direction(Direction dir);
    int set_rpm(int rpm_cur, int rpm_target);

    void reset_pid_states();

  private:
    const int pin_in1;
    const int pin_in2;
    const int pin_enable;

    int last_error;
    int integral;
};
