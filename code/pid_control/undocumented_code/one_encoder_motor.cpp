/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief
 * This is the code script for controlling the raw motor (without encoder)
 * coarsely and adjust the encoded motor to match the speed The raw motor is
 * controlled by a variable resistor, and the encoded motor is controlled by a
 * PID controller using pressure as controlling signal
 * @version 0.1
 * @date 2023-06-30
 *
 *
 */
#include <Arduino.h>
#include <FlexiTimer2.h>

// we assume the ena output drives the raw motor, and the enb output drives the
// encoded motor

/**************** motor specific parameters *******************
 **************************************************************/
// the click per rev for the encoder
// const int cpr1 = 360;
// const int cpr2 = 360;
// the gear reduction ratio of the motor
// const int grr1 = 34;
// const int grr2 = 34;
// the standard rpm of the motor. can tune this parameter to change the max rpm,
// effectively
const int rpm_max1 = 35;
const int rpm_max2 = 100;

// the max pwm input to the ena, enb pins
// this depends on the ratio of
// motor standard input with the power input to the l298n board
// V_motor_i / V_l298n = pwm_write_maxi / 255
const int pwm_write_max1 = 255;
const int pwm_write_max2 = 255;

/************* PID control tunable parameters ****************
 ************************************************************/
const float kp = 1, ki = 0.1, kd = 0;
const float gamma = 0.999;

// pin number for motor control, connected to the l298n board
// because two motors accept different voltage input,
// we have to input 12V to the l298n board and limit the pwm to the 6V motor by
// half
const int pin_ena = 5;
const int pin_in1 = 4;
const int pin_in2 = 7;
const int pin_in3 = 12;
const int pin_in4 = 13;
const int pin_enb = 11;

// pin number for variable resistor to control the target rpm
// rpm_target = rpm_max * (analogRead(pin_rpm) / 1024)
// rpm_target = rpm_target * 2 - rpm_max
const int pin_rpm = A5;

const int rpm_max = min(rpm_max1, rpm_max2);
int rpm_target = min(rpm_max1, rpm_max2) * 0.8;

// // interrupts pin for encoder
// const int pin_A2 = 2;

// // pulse counter for the encoder
// volatile unsigned int counter2 = 0;

// pressure sensor: control signal for the encoded motor
const int pin_pressure = A0;
int pressure = 0;
const int pressure_target = 800;

const int pwm_read_max = 1023;

int pwm_cur1 = 0;
int pwm_cur2 = 0;

// control interval
const unsigned long control_interval_millis = 50;
void control();

enum Direction { Forward, Reverse };
void set_direction(Direction dir);

void setup() {
    Serial.begin(115200);
    pinMode(pin_ena, OUTPUT);
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    pinMode(pin_in3, OUTPUT);
    pinMode(pin_in4, OUTPUT);
    pinMode(pin_enb, OUTPUT);

    // set initial direction
    set_direction(Direction::Forward);

    analogWrite(pin_ena, pwm_cur1);
    analogWrite(pin_enb, pwm_cur2);

    FlexiTimer2::set(control_interval_millis, control);
    FlexiTimer2::start();
}

void loop() {}

int pid(int pressure, int pressure_target);

void control() {
    rpm_target = analogRead(pin_rpm) * 1.0 / pwm_read_max * rpm_max;
    rpm_target = rpm_target * 2 - rpm_max;

    Serial.print("rpm_target: ");
    Serial.print(rpm_target);

    set_direction((rpm_target > 0) ? Direction::Forward : Direction::Reverse);

    // control the pwm of the raw motor to match the target rpm
    pwm_cur1 = abs(rpm_target) / (float)rpm_max1 * pwm_write_max1;
    pwm_cur2 = abs(rpm_target) / (float)rpm_max2 * pwm_write_max2;

    // control the pwm of the encoded motor to match the target pressure
    // when forward, if pressure is low should decrease speed
    // when reverse, if pressure is low should increase speed
    pressure = analogRead(pin_pressure);
    Serial.print(" pressure: ");
    Serial.print(pressure);

    // int pwm2_correction = pid(pressure, pressure_target);
    int pwm2_correction = (pressure_target - pressure) * 1.0 / pwm_read_max *
                          pwm_write_max2 * 0.2;

    if (rpm_target > 0)
        pwm2_correction = -pwm2_correction;

    pwm_cur1 = constrain(pwm_cur1, 0, pwm_write_max1);
    pwm_cur2 = constrain(pwm_cur2 + pwm2_correction, 0, pwm_write_max2);

    Serial.print(" pwm_cur1: ");
    Serial.print(pwm_cur1);
    Serial.print(" pwm_cur2: ");
    Serial.println(pwm2_correction);

    analogWrite(pin_ena, pwm_cur1);
    analogWrite(pin_enb, pwm_cur2);
}

int pid(int pressure, int pressure_target) {
    static float integral = 0;
    static float last_error = 0;

    float error = pressure - pressure_target;
    float derivative = error - last_error;

    integral = gamma * integral + error;
    last_error = error;

    float pwm_correction = kp * error + ki * integral + kd * derivative;
    return pwm_correction;
}

void set_direction(Direction dir) {
    if (dir == Direction::Forward) {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
        digitalWrite(pin_in3, LOW);
        digitalWrite(pin_in4, HIGH);
    } else {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
        digitalWrite(pin_in3, HIGH);
        digitalWrite(pin_in4, LOW);
    }
}
