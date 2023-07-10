#include <Arduino.h>
#include "motor_utils.hpp"

void set_direction(int pin_in1, int pin_in2, Direction dir) {
    switch (dir) {
        case UP:
            digitalWrite(pin_in1, LOW);
            digitalWrite(pin_in2, HIGH);
            break;
        case DOWN:
            digitalWrite(pin_in1, HIGH);
            digitalWrite(pin_in2, LOW);
            break;
        case STOP:
            digitalWrite(pin_in1, LOW);
            digitalWrite(pin_in2, LOW);
            break;
    }
}