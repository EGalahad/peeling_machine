#pragma once
#include "constants.hpp"

class HX711_ADC;

class PressureSensor {
  public:
    PressureSensor(const int pin_pressure_sensor_sck,
                   const int pin_pressure_sensor_dout);
    int get_pressure();

  private:
    const int pin_pressure_sensor_sck;
    const int pin_pressure_sensor_dout;
    HX711_ADC *pressure_sensor;
};

class TouchSensor {
  public:
    TouchSensor(const int pin_touch_sensor);
    bool is_touched();

  private:
    const int pin_touch_sensor;
};
