#include "FlexiTimer2.h"
#include <Arduino.h>

#include "constants.hpp"
#include "motor_utils.hpp"
#include "pins.hpp"

MotorControl motor_upper(pin_in1_upper, pin_in2_upper, pin_ena_upper);
MotorControl motor_lower(pin_in3_lower, pin_in4_lower, pin_enb_lower);
MotorControl motor_peel(pin_in1_peel, pin_in2_peel, pin_ena_peel);

volatile int encoder_upper_count = 0;
volatile int encoder_lower_count = 0;
volatile int encoder_peel_count = 0;

void encoder_upper_isr() { encoder_upper_count++; }
void encoder_lower_isr() { encoder_lower_count++; }
void encoder_peel_isr() { encoder_peel_count++; }

const unsigned long control_loop_period = 50;
void motor_control_loop();

void setup() {
    Serial.begin(115200);
    pinMode(pin_ena_upper, OUTPUT);
    pinMode(pin_in1_upper, OUTPUT);
    pinMode(pin_in2_upper, OUTPUT);
    pinMode(pin_in3_lower, OUTPUT);
    pinMode(pin_in4_lower, OUTPUT);
    pinMode(pin_enb_lower, OUTPUT);

    pinMode(pin_ena_peel, OUTPUT);
    pinMode(pin_in1_peel, OUTPUT);
    pinMode(pin_in2_peel, OUTPUT);

    pinMode(pin_encoder_upper, INPUT_PULLUP);
    pinMode(pin_encoder_lower, INPUT_PULLUP);
    pinMode(pin_encoder_peel, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(pin_encoder_upper), encoder_upper_isr,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_lower), encoder_lower_isr,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_peel), encoder_peel_isr,
                    RISING);

    FlexiTimer2::set(control_loop_period, motor_control_loop);
    FlexiTimer2::start();
}

void loop() {}

void motor_control_loop() {
    motor_upper.set_direction(Direction::DOWN);
    motor_lower.set_direction(Direction::DOWN);
    motor_peel.set_direction(Direction::DOWN);
    
    float rpm_upper =
        encoder_upper_count / ((float)control_loop_period) * 1000 * 60 / cpr / grr;
    float rpm_lower =
        encoder_lower_count / ((float)control_loop_period) * 1000 * 60 / cpr / grr;
    float rpm_peel =
        encoder_peel_count / ((float)control_loop_period) * 1000 * 60 / cpr / grr;

    int pwm_upper = motor_upper.set_rpm(rpm_upper, rpm_target_up_down);
    int pwm_lower = motor_lower.set_rpm(rpm_lower, rpm_target_up_down);
    int pwm_peel = motor_peel.set_rpm(rpm_peel, rpm_target_up_down);

    encoder_upper_count = 0;
    encoder_lower_count = 0;
    encoder_peel_count = 0;

    static unsigned long last_print = 0;
    if (millis() - last_print > 1000) {
        last_print = millis();
        Serial.print("rpm_upper: ");
        Serial.print(rpm_upper);
        Serial.print(" rpm_lower: ");
        Serial.print(rpm_lower);
        Serial.print(" rpm_peel: ");
        Serial.println(rpm_peel);

        Serial.print("pwm_upper: ");
        Serial.print(pwm_upper);
        Serial.print(" pwm_lower: ");
        Serial.print(pwm_lower);
        Serial.print(" pwm_peel: ");
        Serial.println(pwm_peel);

        Serial.println();
    }
}
