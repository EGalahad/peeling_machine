#include <Arduino.h>

#include "../../control/src/pins.hpp"

void setup() {
    Serial.begin(115200);
    pinMode(pin_start_stop_button, INPUT);
    pinMode(pin_self_cleaning_button, INPUT);
}

void loop() {
    // constantly check external signals
    static bool last_start_stop_button_state = LOW;
    static bool last_self_clean_button_state = LOW;

    bool start_stop_button_state = digitalRead(pin_start_stop_button);
    bool self_clean_button_state = digitalRead(pin_self_cleaning_button);

    if (start_stop_button_state != last_start_stop_button_state) {
        if (start_stop_button_state == HIGH) {
            Serial.println("start_stop button pressed!");
        }
    }

    if (self_clean_button_state != last_self_clean_button_state) {
        if (self_clean_button_state == HIGH) {
            Serial.println("self_clean button pressed!");
        }
    }
    Serial.print("start_stop_button_state: ");
    Serial.println(start_stop_button_state);

    last_start_stop_button_state = start_stop_button_state;
    last_self_clean_button_state = self_clean_button_state;
    delay(50);
    return;
}