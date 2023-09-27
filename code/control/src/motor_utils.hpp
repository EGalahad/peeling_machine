#pragma once
#include "constants.hpp"

enum Direction { UP, DOWN, STOP };

class MotorControl {
  public:
    MotorControl(const int pin_in1, const int pin_in2, const int pin_enable,
                 const float kp = kp_default, const float ki = ki_default,
                 const float kd = kd_default)
        : pin_in1(pin_in1), pin_in2(pin_in2), pin_enable(pin_enable), kp(kp),
          ki(ki), kd(kd), last_error(0), integral(0) {}
    void set_direction(Direction dir);
    void set_rpm(int rpm_cur, int rpm_target);

    void reset_pid_states();

  private:
    const int pin_in1;
    const int pin_in2;
    const int pin_enable;
    const float kp;
    const float ki;
    const float kd;

    int last_error;
    int integral;
};
