/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief This is the overall control code for the peeling machine, using Arduino.
 * 
 * @details
 * The machine exposes two buttons, start-stop button and a self cleaning button.
 * The start stop button does the main job, while self cleaning has very simple logic
 *
 * The machine has several states: RUNNING, EXITING, STOPPED, CLEANING
 * 
 * When the machine is connected, it first set itself in STOPPED state.
 * When the user puts the fruit in the holding bowl to peel and presses the start button, the machine goes into RUNNING state.
 * 
 * =============================   RUNNING   ===============================
 * Then the machine will go over several stages:
 * 1. (CLOSE) The upper and lower clamping blades (name needs to be changed) will close to hold the fruit in place,
 *  when the pressure sensor detects the fruit, the machine will go to next stage.
 * 2. (UP) The upper and lower clamping blades will work in coordination to lift up the fruit and rotate it to peel the fruit.
 *  The peeling blade is placed on the inner surface of the machine, and the fruit is rotated against the blade to peel.
 *  When the lower clamp blade reaches the height of the peeling blade, the machine will go to next stage.
 * TODO: We have not determined how to put the peeling blade at the same height as the top of the fruit in the beginning of this stage.
 * TODO: We have not determined how to detect when the lower clamp blade reaches the height of the peeling blade in the end.
 * 
 * =============================  EXITING  ==================================
 * If the above stage finishes, the machine will go to the EXITING stage
 * 3. (DOWN) The upper and lower clamping blades will work in coordination to lower the fruit, 
 *  and the fruit will be placed back to the holding bowl.
 *  When the pressure sensor on the upper clamp blade detects a surge in pressure, it is when the fruit reached the holding bowl.
 * TODO: We have not determined how to avoid the peeling blade when lowering the fruit.
 * 4. (OPEN) The upper and lower clamping blades will open to release the fruit.
 *  We place two sensors on the end of the upper and lower clamping blades to detect when they should stop openning.
 *  When the sensors are triggered, the machine will go to next stage.
 * 
 * NOTE: If the start-stop button is pressed during stage 1, the machine will jump to stage 4.
 * NOTE: If the start-stop button is pressed during stage 2, the machine will jump to stage 3.
 * 
 * =============================  STOPPED  ==================================
 * If the above stage finishes, the machine will go to the STOPPED stage
 * 
 * =============================  CLEANING  ==================================
 * If the self cleaning button is pressed, when the machine is in STOPPED state, the machine will go to the CLEANING stage.
 * If the machine is not in STOPPED state, the machine will ignore the self cleaning button.
 * 
 * In the cleaning state, the machine will go over several stages:
 * 1. (CLOSE) The upper and lower clamping blades will close to cover the side breach of the peeling blade.
 * TODO: We have not determined how to determine when to stop closing the clamping blades.
 * 2. (CLEANING) The small water pumps attached onto the inner surface of the machine will spray water to clean the peeling blade.
 * 3. (OPEN) The upper and lower clamping blades will open to release the fruit.
 * 
 * @version 0.1
 * @date 2023-07-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <Arduino.h>
#include "motor_utils.hpp"
#include "state_control.hpp"

auto state_machine = StateMachine(State::STOPPED);

/********************* PIN DEFINITION ***********************
************************************************************/
// button pins
const int pin_start_stop_button = 2;
const int pin_self_cleaning_button = 3;

// clamping blades motor pins
const int pin_ena = 5;
const int pin_in1 = 6;
const int pin_in2 = 7;
const int pin_in3 = 8;
const int pin_in4 = 9;
const int pin_enb = 10;

// // rotating motor pins
// const int pin_ena_rotating = 11;
// const int pin_in1_rotating = 12;
// const int pin_in2_rotating = 13;


// screw rod trigger pins
const int pin_upper_screw_rod_trigger = A0;
const int pin_lower_screw_rod_trigger = A1;

// pressure sensor pins
const int pin_upper_pressure_sensor = A2;

// // water pump pins
// const int pin_water_pump_1 = 13;


/********************* PARAMETERS *********************
 ***********************************************************/

// Clamping motors specifics
// assume the motor will run at max rpm when pwm to enable pin is pwm_max
const int rpm_max = 30; 
const int pwm_max = 255;

const int pwm_read_max = 1023;

// Pressure and speed control

// this is the desired rpm in the UP and DOWN stage, directly controlling the speed of the lower motor coarsely
const int rpm_target = 20;

// pressure reading is lower when the pressure on sensor is higher

// this is the desired pressure in the UP and DOWN stage, the upper motor will adjust its speed to reach this pressure, 
// by finely controlling the speed of the upper motor using pid
const int pressure_target = 900;

// this is the pressure when we assume the fruit is grasped in the clamping blades in CLOSE stage
const int pressure_threshold = 980;

// this is the pressure when we assume the fruit touches the holding bowl in the clamping blades in DOWN stage
const int pressure_threshold = 840;


// PID control parameters
const float kp = 3, ki = 0.1, kd = 0;
const float gamma = 1;
const float ratio = 0.2;


/********************* GLOBAL VARIABLES *********************
 ***********************************************************/

int pwm_upper = 0;
int pwm_lower = 0;

int pressure = 0;


/********************* FUNCTIONS *********************
 ***********************************************************/

void init_pins();
void init_state();

void setup() {
    Serial.begin(115200);
    init_pins();
    init_state();
}


void init_pins() {
    pinMode(pin_start_stop_button, INPUT);
    pinMode(pin_self_cleaning_button, INPUT);

    pinMode(pin_ena, OUTPUT);
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    pinMode(pin_in3, OUTPUT);
    pinMode(pin_in4, OUTPUT);
    pinMode(pin_enb, OUTPUT);

    // pinMode(pin_ena_rotating, OUTPUT);
    // pinMode(pin_in1_rotating, OUTPUT);
    // pinMode(pin_in2_rotating, OUTPUT);

    pinMode(pin_upper_screw_rod_trigger, INPUT);
    pinMode(pin_lower_screw_rod_trigger, INPUT);

    pinMode(pin_upper_pressure_sensor, INPUT);

    // pinMode(pin_water_pump_1, OUTPUT);
}

void init_state() {
    // set initial output values
    set_direction(pin_in1, pin_in2, Direction::STOP);
    analogWrite(pin_ena, 0);
    set_direction(pin_in3, pin_in4, Direction::STOP);
    analogWrite(pin_enb, 0);

    // set_direction(pin_in1_rotating, pin_in2_rotating, Direction::STOP);
    // analogWrite(pin_ena_rotating, 0);
}

