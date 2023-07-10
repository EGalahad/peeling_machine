/**
 * @file main.cpp
 * @author elijah
 * @brief
 * This is the code script to control two identical motors (without encoder)
 * to move in the same direction and speed.
 * The target speed is set by a variable resistor, and the fine speed is
 * controlled by a PID controller using pressure as controlling signal
 * @version 0.1
 * @date 2023-06-30
 *
 */
#include <Arduino.h>
#include <FlexiTimer2.h>

/********************* PIN DEFINITION ***********************
************************************************************/
// pin number for motor control, connected to the l298n board
/**
 * we assume the ena output drives the lower motor, which is coarsely controlled
 * by the target_rpm, and the enb output drives the upper motor, the speed of
 * the upper motor is controlled using PID
 */
const int pin_ena = 5;
const int pin_in1 = 6;
const int pin_in2 = 7;
const int pin_in3 = 8;
const int pin_in4 = 9;
const int pin_enb = 10;

// variable resistor to control the target rpm
const int pin_rpm_target = A5;

// pressure resistor sensor
const int pin_pressure = A0;

/********************* PARAMETERS *********************
 ***********************************************************/
// motor specifics
// the standard rpm of the motor. can tune this parameter to change the max rpm,
// effectively
const int rpm_max = 150;

// PID control parameters
const float kp = 1.5, ki = 0, kd = 0;
const float gamma = 1;
const float ratio = 3;

/********************* GLOBAL VARIABLES *********************
 ***********************************************************/
const int pwm_read_max = 1023;

// the max pwm input to the ena, enb pins
// this depends on the ratio of
// motor standard input with the power input to the l298n board
// V_motor_i / V_l298n = pwm_write_maxi / 255
const int pwm_write_max = 255;

// rpm
int rpm_target = rpm_max * 0.5;

int pwm_cur1 = 0;
int pwm_cur2 = 0;


// pressure
int pressure = 0;
const int pressure_target = 900;
// the threshold for pressure to consider the two motors are in contact
const int pressure_threshold = 980;
// the threshold for pressure for emergency stop
const int pressure_threshold_lower = 850;

// time interval in s to reset the pid integral
const int integral_reset_interval = 10;

// control interval in milli seconds
const unsigned long control_interval_millis = 50;
void control();

enum Direction { UP, DOWN, STOP };
void set_direction(int pin_in1, int pin_in2, Direction dir);

void setup() {
    Serial.begin(115200);
    pinMode(pin_ena, OUTPUT);
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    pinMode(pin_in3, OUTPUT);
    pinMode(pin_in4, OUTPUT);
    pinMode(pin_enb, OUTPUT);

    pinMode(pin_rpm_target, INPUT);
    pinMode(pin_pressure, INPUT);

    // set initial direction
    set_direction(pin_in1, pin_in2, Direction::STOP);
    set_direction(pin_in3, pin_in4, Direction::STOP);

    analogWrite(pin_ena, pwm_cur1);
    analogWrite(pin_enb, pwm_cur2);

    FlexiTimer2::set(control_interval_millis, control);
    FlexiTimer2::start();
}

void loop() {}

int pid(int pressure, int pressure_target);

void control() {
    pressure = analogRead(pin_pressure);

    if (pressure < pressure_threshold_lower) {
        set_direction(pin_in1, pin_in2, Direction::STOP);
        analogWrite(pin_ena, 0);

        set_direction(pin_in3, pin_in4, Direction::STOP);
        analogWrite(pin_enb, 0);

        Serial.println("pressure is lower than threshold, stopping");
        return;
    }

    if (pressure > pressure_threshold) {
        // if the two are not touching, move motor 2 downwards
        set_direction(pin_in1, pin_in2, Direction::UP);
        analogWrite(pin_ena, pwm_write_max / 2);

        set_direction(pin_in3, pin_in4, Direction::DOWN);
        analogWrite(pin_enb, pwm_write_max / 3);

        Serial.print("pressure ");
        Serial.print(pressure);
        Serial.println(" is larger than threshold, moving motor 2 downwards");
        return;
    }

    rpm_target = analogRead(pin_rpm_target) * 1.0 / pwm_read_max * rpm_max;
    rpm_target = rpm_target * 2 - rpm_max;
    Serial.print("rpm_target: ");
    Serial.print(rpm_target);

    set_direction(pin_in1, pin_in2,
                  (rpm_target > 0) ? Direction::UP : Direction::DOWN);
    set_direction(pin_in3, pin_in4,
                  (rpm_target > 0) ? Direction::UP : Direction::DOWN);

    // control the pwm of the raw motor to match the target rpm
    pwm_cur1 = abs(rpm_target) / (float)rpm_max * pwm_write_max;
    pwm_cur2 = abs(rpm_target) / (float)rpm_max * pwm_write_max;
    // pwm_cur2 = abs(pid(pressure, pressure_target));

    // control the pwm of the encoded motor to match the target pressure
    // when forward, if pressure is low (value is larger) should decrease speed
    // when reverse, if pressure is low (value is larger) should increase speed
    Serial.print(", pressure: ");
    Serial.print(pressure);

    int pwm2_correction = (pressure_target - pressure) * 1.0 / pwm_read_max *
                          pwm_write_max * ratio;

    pwm_cur2 = pwm_cur2 + pwm2_correction;

    // pwm_cur2 = 100;
    Serial.print(" pwm_cur1: ");
    Serial.print(pwm_cur1);
    Serial.print(" pwm_cur2: ");
    Serial.println(pwm_cur2);

    pwm_cur1 = constrain(pwm_cur1, 0, pwm_write_max);
    pwm_cur2 = constrain(pwm_cur2, 0, pwm_write_max);
    // pwm_cur2 = constrain(pwm_cur2, 0, pwm_write_max2);

    analogWrite(pin_ena, pwm_cur1);
    analogWrite(pin_enb, pwm_cur2);
}

int pid(int pressure, int pressure_target) {
    static float integral = 0;
    static float last_error = 0;
    static float reset_time = 0;
    // static unsigned long counter = 0;

    // counter++;

    float error = pressure_target - pressure;
    float derivative = error - last_error;

    integral = gamma * integral + error;
    last_error = error;

    float pwm_correction = kp * error + ki * integral + kd * derivative;
    // float pwm_correction = kp * error + ki * (integral / counter) + kd *
    // derivative; float pwm_correction = kp * error
    //                     + ki * (integral / (counter * control_interval_millis
    //                     / 1000.0))
    //                     + kd * derivative * (control_interval_millis /
    //                     1000.0);

    // if (counter == (unsigned long)(-1)) {
    //     counter = 0;
    //     integral = 0;
    // } else {
    //     counter++;
    // }

    // reset the integral term after some interval
    if (millis() - reset_time > 1000 * integral_reset_interval) {
        integral = 0;
        reset_time = millis();
    }

    return pwm_correction;
}

void set_direction(int pin_in1, int pin_in2, Direction dir) {
    if (dir == Direction::UP) {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
    } else if (dir == Direction::DOWN) {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
    } else if (dir == Direction::STOP) {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
    }
}
