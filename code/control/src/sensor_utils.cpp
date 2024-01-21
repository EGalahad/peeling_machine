#include <Arduino.h>

#include "sensor_utils.hpp"

#include <HX711_ADC.h>

PressureSensor::PressureSensor(const int pin_pressure_sensor_sck,
                               const int pin_pressure_sensor_dout)
    : pin_pressure_sensor_sck(pin_pressure_sensor_sck),
      pin_pressure_sensor_dout(pin_pressure_sensor_dout) {
    pressure_sensor =
        new HX711_ADC(pin_pressure_sensor_dout, pin_pressure_sensor_sck);
    pressure_sensor->begin();
    float calibrationValue;   // calibration value (see example file
                              // "Calibration.ino")
    calibrationValue = 696.0; // uncomment this if you want to set the
                              // calibration value in the sketch
    unsigned long stabilizingtime =
        2000; // preciscion right after power-up can be improved by adding a few
              // seconds of stabilizing time
    boolean _tare = true; // set this to false if you don't want tare to be
                          // performed in the next step
    pressure_sensor->start(stabilizingtime, _tare);
    if (pressure_sensor->getTareTimeoutFlag()) {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        while (1)
            ;
    } else {
        pressure_sensor->setCalFactor(
            calibrationValue); // set calibration value (float)
        Serial.println("Startup is complete");
    }
}
int PressureSensor::get_pressure() {
    // returns pressure in grams
    while (not pressure_sensor->update())
        ;
    return pressure_sensor->getData();
}

TouchSensor::TouchSensor(const int pin_touch_sensor)
    : pin_touch_sensor(pin_touch_sensor) {}
bool TouchSensor::is_touched() {
    // returns true if touched
    int touch = digitalRead(pin_touch_sensor);
    return touch == HIGH;
}
