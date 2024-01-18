#pragma once

/********************* PARAMETERS *********************
 ***********************************************************/

// Motors Specifics
// assume the motor will run at max rpm when pwm to enable pin is pwm_max
const int rpm_max = 30;
const int pwm_max = 255;

const int pwm_write_max = 255;
const int pwm_read_max = 1023;

// Encoder Specifics
// click per revolution
const int cpr = 360;
const int cpr_rotate = 6;
// gear reduction rate
const int grr = 34;
const int grr_rotate = 90;

// screw lead, in cm
const float screw_lead = 0.5;

// Pressure and Speed Control

// this is the desired rpm in various stages
const int rpm_target_close = 60;
const int rpm_target_up_down = 60;
const int rpm_target_peel = 60;
const int rpm_target_rotate = 20;
const int rpm_target_open = 90;

// @note: pressure reading is lower when the pressure on sensor is higher

// used in CLOSE to determine if the upper motor should stop
const int pressure_touch = 970;

// used in CLOSE to determine if the lower motor should stop
const int pressure_grasp = 950;

// used in DOWN to determine if the fruit touches the holding bowl
const int pressure_touch_bowl = 880;

// used in OPEN to determine if the upper/lower/peel screw rod motor should stop
const int pressure_screw_rod_end_of_range = 880;

// pressure too high, warning!
const int pressure_threshold_lower = 600;

// PID control parameters
const float kp_default = 3, ki_default = 0.1, kd_default = 0;
const float gamma = 1;
const float ratio = 0.2;

// height to lift after the fruit is grasped by the clamping blades, in cm
const float height_up = 10;
// the total distance between the two clamping blades, when separated, in cm
const float height_total = 20;

// height to descend the upper clamping blade before charging the pump
const float height_to_descend = 5;
// height to ascend the upper clamping blade after charging the pump
const float height_to_ascend = 5;

// time to spray water, in seconds
const float spray_time_default = 5;