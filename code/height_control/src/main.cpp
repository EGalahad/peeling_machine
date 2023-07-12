/**
 * @file main.cpp
 * @author elijah
 * @brief This is the code script for testing controlling the motor to move the
 * block on screw rod to move some distances
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <Arduino.h>
#include <FlexiTimer2.h>

#include "../../control/src/constants.hpp"
#include "../../control/src/motor_utils.hpp"
#include "../../control/src/pins.hpp"

// desired height to move, in cm
const int height = 10;
const int rpm_target = 30;

float height_remaining = height;

auto motor_upper = MotorControl(pin_ena_upper, pin_in1_upper, pin_in2_upper);

volatile int encoder_upper_count = 0;
void encoder_upper_count_increment() {
    encoder_upper_count++;
}

void init_pins();
void motor_control_loop();
// void (*f)();

const int motor_control_interval = 50;

void setup() {
    Serial.begin(115200);
    init_pins();

    attachInterrupt(digitalPinToInterrupt(pin_ena_upper), encoder_upper_count_increment, RISING);

    FlexiTimer2::set(motor_control_interval, motor_control_loop);
    FlexiTimer2::start();

    height_remaining = height;
}

void loop() {
}

void init_pins() {
    pinMode(pin_ena_upper, OUTPUT);
    pinMode(pin_in1_upper, OUTPUT);
    pinMode(pin_in2_upper, OUTPUT);

    pinMode(pin_encoder_upper, INPUT);
}

void motor_control_loop() {
    if (height_remaining <= 0) {
        motor_upper.set_direction(Direction::STOP);
        return;
    }

    float rpm_cur_upper = (float)encoder_upper_count / cpr / grr / (motor_control_interval / 1000.0) * 60.0;
    
    Serial.print("rpm_cur_upper: ");
    Serial.print(rpm_cur_upper);

    motor_upper.set_direction(Direction::UP);
    motor_upper.set_rpm(rpm_cur_upper, rpm_target);

    height_remaining -= (float)encoder_upper_count / cpr / grr * screw_lead;

    encoder_upper_count = 0;
}


