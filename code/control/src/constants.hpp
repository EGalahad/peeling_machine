#pragma once

/********************* PARAMETERS *********************
 ***********************************************************/

// motors specifics
// assume the motor will run at max rpm when pwm to enable pin is pwm_max
const int rpm_max = 30;
const int pwm_max = 255;

const int pwm_write_max = 255;
const int pwm_read_max = 1023;

// encoder specifics
// click per revolution
const int cpr = 360;
// gear reuction rate
const int grr = 34;

// the lead of the screw, in cm
const float screw_lead = 0.5;

// Pressure and speed control

// this is the desired rpm in the UP and DOWN stage, directly controlling the
// speed of the lower motor coarsely
const int rpm_target_close = 20;
const int rpm_target_up_down = 20;
const int rpm_target_peel = 20;
const int rpm_target_open = 30;

// pressure reading is lower when the pressure on sensor is higher

// used in CLOSE to determine if the upper motor should stop
const int pressure_touch = 980;

// used in CLOSE to determine if the lower motor should stop
const int pressure_grasp = 900;

// used in DOWN to determine if the fruit touches the holding bowl
const int pressure_touch_bowl = 880;

// pressure too high, warning!
const int pressure_threshold_lower = 840;

// PID control parameters
const float kp = 3, ki = 0.1, kd = 0;
const float gamma = 1;
const float ratio = 0.2;



// height to lift after the fruit is grasped by the clamping blades, in cm
const int up_height = 10;