#include <Arduino.h>

#include "motor_utils.hpp"

#include "constants.hpp"
#include "globals.hpp"

void MotorControl::set_direction(Direction dir) {
    switch (dir) {
    case Direction::UP:
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
        break;

    case Direction::DOWN:
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
        break;
    case Direction::STOP:
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
        break;
    default:
        break;
    }
}

void MotorControl::set_rpm(int rpm_cur, int rpm_target) {
    // sets pwm to the motor using pid algorithm
    int error = rpm_target - rpm_cur;
    integral += error;
    int derivative = error - last_error;
    int pwm = kp * error + ki * integral + kd * derivative;
    last_error = error;
    pwm = constrain(pwm, 0, pwm_write_max);
    analogWrite(pin_enable, pwm);
}

void MotorControl::reset_pid_states() {
    last_error = 0;
    integral = 0;
}
