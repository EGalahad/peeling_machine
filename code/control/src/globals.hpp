#pragma once

class MotorControl;

/***
 * states variables, initialized in state machine during state transitions
 * later controlled by motor control loop and used by motor control.
 */
extern bool close_upper;

extern float height_remaining_upper;
extern float height_remaining_lower;
extern float height_remaining_peel;

extern float peel_height;

// extern bool open_upper;
// extern bool open_lower;
// extern bool open_peel;


/***
 * control handlers, for both main.cpp and state_machine.cpp 
 * to control or initialize the motors, etc.
*/
extern MotorControl motor_upper;
extern MotorControl motor_lower;
extern MotorControl motor_peel;
