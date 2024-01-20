#include <Arduino.h>

#include "sensor_utils.hpp"

#include <HX711.h>

PressureSensor::PressureSensor(const int pin_pressure_sensor_sck,
                               const int pin_pressure_sensor_dout)
    : pin_pressure_sensor_sck(pin_pressure_sensor_sck),
      pin_pressure_sensor_dout(pin_pressure_sensor_dout) {
    pressure_sensor = new HX711();
    pressure_sensor->begin(pin_pressure_sensor_dout, pin_pressure_sensor_sck);
    // set calibration factor
    const float calibration_factor = 10.;
    pressure_sensor->set_scale(calibration_factor);
    // print sensor calibration factor
    Serial.print("initialized pressure sensor with scale: ");
    Serial.println(pressure_sensor->get_scale());
    Serial.println(pressure_sensor->get_units(2));
    pressure_sensor->tare(10);
    Serial.println(pressure_sensor->get_units(1));


}
int PressureSensor::get_pressure() { return pressure_sensor->get_units(2); }

TouchSensor::TouchSensor(const int pin_touch_sensor)
    : pin_touch_sensor(pin_touch_sensor) {}
bool TouchSensor::is_touched() {
    // returns true if touched
    int touch = digitalRead(pin_touch_sensor);
    return touch == HIGH;
}
