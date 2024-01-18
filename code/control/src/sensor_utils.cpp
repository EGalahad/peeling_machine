#include <Arduino.h>

#include "sensor_utils.hpp"

#include <HX711.h>

PressureSensor::PressureSensor(const int pin_pressure_sensor_sck,
                               const int pin_pressure_sensor_dout)
    : pin_pressure_sensor_sck(pin_pressure_sensor_sck),
      pin_pressure_sensor_dout(pin_pressure_sensor_dout) {
    pressure_sensor = new HX711();
    pressure_sensor->begin(pin_pressure_sensor_dout, pin_pressure_sensor_sck);
    pressure_sensor->set_scale();
}
int PressureSensor::get_pressure() { return pressure_sensor->get_units(1); }

TouchSensor::TouchSensor(const int pin_touch_sensor)
    : pin_touch_sensor(pin_touch_sensor) {}
bool TouchSensor::is_touched() {
    // returns true if touched
    int touch = digitalRead(pin_touch_sensor);
    return touch == HIGH;
}
