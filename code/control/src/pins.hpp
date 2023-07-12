#pragma once
#include <Arduino.h>

/********************* PIN DEFINITION ***********************
************************************************************/
// button pins
const int pin_start_stop_button = 2;
const int pin_self_cleaning_button = 3;

// clamping blades motor pins
const int pin_ena_upper = 5;
const int pin_in1_upper = 6;
const int pin_in2_upper = 7;
const int pin_in3_lower = 8;
const int pin_in4_lower = 9;
const int pin_enb_lower = 10;

// rotating motor pins
const int pin_ena_rotate = 11;
const int pin_in1_rotate = 12;
const int pin_in2_rotate = 13;
// peeling blade motor pins
const int pin_in3_peel = 4;
const int pin_in4_peel = 5;
const int pin_enb_peel = 6;

// encoder signal pins
const int pin_encoder_upper = 2;
const int pin_encoder_lower = 2;
const int pin_encoder_peel = 2;

// pressure sensors attached to the end at the range of motion of screw rods
const int pin_screw_rod_upper = A0;
const int pin_screw_rod_lower = A1;
const int pin_screw_rod_peel = A2;

// pressure sensor attached to upper clamping blade
const int pin_upper_pressure_sensor = A2;

// // water pump pins
// const int pin_water_pump_1 = 13;


