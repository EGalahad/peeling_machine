/**
 * @file globals.hpp
 * @author elijah
 * @brief Global variables shared between main.cpp and state_control.cpp
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once


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
 * motor control handlers, for main.cpp and state_machine.cpp
 * to control or initialize the motors, respectively.
 */
class MotorControl;
extern MotorControl motor_upper;
extern MotorControl motor_lower;
extern MotorControl motor_peel;
extern MotorControl motor_rotate_lower;
