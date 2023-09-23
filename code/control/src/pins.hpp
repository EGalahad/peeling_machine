#pragma once
#include <Arduino.h>

/********************* PIN DEFINITION ***********************
************************************************************/
// button pins
const int pin_start_stop_button = 22;
const int pin_self_cleaning_button = 23;

// clamping blades motor pins
const int pin_ena_upper = 13;
const int pin_in1_upper = 12;
const int pin_in2_upper = 11;
const int pin_in3_lower = 10;
const int pin_in4_lower = 9;
const int pin_enb_lower = 8;

// peeling blade motor pins
const int pin_ena_peel = 2;
const int pin_in1_peel = 3;
const int pin_in2_peel = 4;
// rotating motor pins
const int pin_in3_rotate_lower = 5;
const int pin_in4_rotate_lower = 6;
const int pin_enb_rotate_lower = 7;

// encoder signal pins
const int pin_encoder_upper = 18;
const int pin_encoder_lower = 19;
const int pin_encoder_peel = 20;
const int pin_encoder_rotate = 21;

// pressure sensors attached to the end at the range of motion of screw rods
const int pin_pressure_screw_rod_upper = A0;
const int pin_pressure_screw_rod_lower = A1;
const int pin_pressure_screw_rod_peel = A2;

// pressure sensor attached to upper clamping blade
const int pin_pressure_clamp_upper = A3;

// // water pump pins
// const int pin_water_pump_1 = 13;
