/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * This is the code script for controlling two GMR sensor equipped motors with pid closed loop.
 * The target rpm can be controlled by a variable resistor.
 * @version 0.1
 * @date 2023-06-30
 * 
 * motor specific parameters are:
 * @param cpr the click per rev for the encoder
 * @param grr the gear reduction ratio of the motor
 * @param rpm_max the standard rpm of the motor. can tune this parameter to change the max rpm, effectively
 * 
 * tunable paramters are:
 * @param ki the integral gain
 * @param kp the proportional gain
 * @param gamma the discount factor for the integral term
 * 
 * pin parameters are:
 * @param pin_ena, pin_in1, pin_in2, pin_in3, pin_in4, pin_enb the pin number for motor control, connected to the l298n board
 * @param pin_rpm_target the variable resistor pin to control the target rpm
 * @param pin_A1, pin_A2 the interrupts pin for encoder of the motor
 */
#include <Arduino.h>
#include <FlexiTimer2.h>

/********************* PIN DEFINITION ***********************
************************************************************/
// pin number for motor control, connected to the l298n board
const int pin_ena = 9; // needs to support pwm
const int pin_in1 = 8;
const int pin_in2 = 7;
const int pin_in3 = 6;
const int pin_in4 = 5;
const int pin_enb = 4; // needs to support pwm

// interrupts pin for motor encoders
const int pin_A1 = 2;
const int pin_A2 = 3;

// variable resistor pin to control the target rpm
const int pin_rpm_target = A5;


/********************* PARAMETERS *********************
 ***********************************************************/
// motor specifics
// the click per rev for the encoder
const int cpr = 360; 
// the gear reduction ratio of the motor
const int grr = 34;
// the standard rpm of the motor. can tune this parameter to change the max rpm, effectively
const int rpm_max = 60;

// pwm for each motor - using Arduino PWM, so 0-255
const int pwm_write_max = 255;
const int pwm_read_max = 1023;

// PID control parameters
const float ki = 0.1, kp = 1;
const float gamma = 1;

/********************* GLOBAL VARIABLES *********************
 ***********************************************************/
// encoder pulse counters
volatile unsigned int counter1 = 0;
volatile unsigned int counter2 = 0;

// rpm
int rpm_target = rpm_max / 2;
int rpm_cur1 = 0;
int rpm_cur2 = 0;

// bool running = false;

int pwm_cur1 =
    pwm_write_max / 2; // the speed we will run at - half speed (pwm is 0-255)
int pwm_cur2 =
    pwm_write_max / 2; // the speed we will run at - half speed (pwm is 0-255)

// control interval
const unsigned long control_interval_millis = 5;

void ISR_count1() { counter1++; }

void ISR_count2() { counter2++; }

void control();

void setup() {
    Serial.begin(115200);
    pinMode(pin_ena, OUTPUT);
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    pinMode(pin_in3, OUTPUT);
    pinMode(pin_in4, OUTPUT);
    pinMode(pin_enb, OUTPUT);

    pinMode(pin_A1, INPUT);
    pinMode(pin_A2, INPUT);

    // set initial direction
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
    digitalWrite(pin_in3, LOW);
    digitalWrite(pin_in4, HIGH);

    // interrupts to count encoder pulses
    attachInterrupt(digitalPinToInterrupt(pin_A1), ISR_count1, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_A2), ISR_count2, RISING);

    analogWrite(pin_ena, pwm_cur1);
    analogWrite(pin_enb, pwm_cur2);

    FlexiTimer2::set(control_interval_millis, control);
    FlexiTimer2::start();
}

void loop() {}

int pid_A(int velocity, int target);
int pid_B(int velocity, int target);
void set_pwm(int pin, int pwm, int in1, int in2);

void control() {
    rpm_target = analogRead(pin_rpm_target) / (float)pwm_read_max * rpm_max;
    rpm_target = rpm_target * 2 - rpm_max;

    Serial.print("rpm_target: ");
    Serial.print(rpm_target);

    if (rpm_target < 0) {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
        digitalWrite(pin_in3, HIGH);
        digitalWrite(pin_in4, LOW);
        rpm_target = -rpm_target;
    } else {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
        digitalWrite(pin_in3, LOW);
        digitalWrite(pin_in4, HIGH);
    }

    // calculate the rpm of both motors and ajust them to target
    rpm_cur1 =
        counter1 / ((float)cpr * control_interval_millis / 1000.0) * 60 / grr;
    rpm_cur2 =
        counter2 / ((float)cpr * control_interval_millis / 1000.0) * 60 / grr;

    pwm_cur1 = pid_A(rpm_cur1, rpm_target);
    pwm_cur2 = pid_B(rpm_cur2, rpm_target);

    Serial.print(", rpm_cur1: ");
    Serial.print(rpm_cur1);
    Serial.print(", rpm_cur2: ");
    Serial.print(rpm_cur2);
    Serial.print(", pwm_cur1: ");
    Serial.print(pwm_cur1);
    Serial.print(", pwm_cur2: ");
    Serial.println(pwm_cur2);

    set_pwm(pin_ena, pwm_cur1, pin_in1, pin_in2);
    set_pwm(pin_enb, pwm_cur2, pin_in3, pin_in4);

    counter1 = 0;
    counter2 = 0;
}

int pid_A(int velocity, int target) {
    static float bias, pwm = 0, last_bias = 0;
    bias = target - velocity;
    pwm = gamma * pwm + ki * bias + kp * (bias - last_bias);
    pwm = max(min(pwm, pwm_write_max), -pwm);
    last_bias = bias;
    return pwm;
}

int pid_B(int velocity, int target) {
    static float bias, pwm = 0, last_bias = 0;
    bias = target - velocity;
    pwm += ki * bias + kp * (bias - last_bias);
    pwm = max(min(pwm, pwm_write_max), -pwm);
    last_bias = bias;
    return pwm;
}

void set_pwm(int pin, int pwm, int in1, int in2) {
    analogWrite(pin, pwm);
}
