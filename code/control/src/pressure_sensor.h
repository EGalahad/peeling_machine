/*
 * @file: pressure_sensor.h
 * @brief: This is the header file for the pressure sensor. There are
 *        two functions: one is to initialize the pressure sensor, and the
 *        other is to get the pressure value.
 * @author: Gu
 * @date: 10/23/2023
 */
#include <HX711_ADC.h>
#include <Wire.h>

HX711_ADC LoadCell(A0, A1); // Change pin numbers to match the setup

void InitPressureSensor() { // Initialize the pressure sensor; to be placed
                            // in setup()
  LoadCell.begin();
  LoadCell.start(2000);
  LoadCell.setCalFactor(1000.0); // Calibration factor
}

float GetPressure() { // Get the pressure value
  LoadCell.update();
  float pressure = LoadCell.getData();
  return pressure;
}
