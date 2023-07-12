/**
 * @file main.cpp
 * @author elijah
 * @brief This is the overall control code for the peeling machine, using
 * Arduino.
 *
 * @details
 * The machine exposes two buttons, a start-stop button and a self cleaning
 * button. The start-stop button does the main job, while self cleaning has very
 * simple logic.
 *
 * The machine has several states: STOPPED, RUNNING, EXITING, CLEANING.
 *
 * When the machine is connected to power, it first set itself in STOPPED state.
 * The user should put the fruit in the holding bowl to peel and press the
 * start button, then the machine goes into RUNNING state.
 *
 * =============================  STATE: RUNNING ===============================
 * Then the machine will go over several stages, transition to the next state by
 * calling StateMachine::next():
 * 1. (STATE: CLOSE) The upper and lower clamping blades (TODO: name needs to be
 * changed) will close to hold the fruit in place. The peeling blades will also
 * be moved down with the upper clamping blade.
 * The upper clamp blade with a pressure sensor is first moved downwards, and
 * when the pressure sensor detects a surge in pressure (pressure <
 * pressure_touch) the upper clamping blade stops and the lower clamping blade
 * starts to move upwards. When the pressure sensor on the upper clamping blade
 * detects another surge in pressure (pressure < pressure_grasp), the machine
 * will go to next stage.
 * 2. (STATE: UP) The upper and lower clamping blades will work in
 * coordination to lift up the fruit, for some fixed distance (This should be
 * implemented by recording the vertical distance moved in the motor control
 * loop)
 * 3. (STATE: PEEL)
 * (At this time the lower clamping blades or the bottom of the fruit is at a
 * fixed pre-determined height, which will be the height at which the peeling
 * blades should descend to when finishing the peeling process). The peeling
 * starts and the clamping blades will rotate the fruit while the peeling blades
 * move downwards. The peeling blade is placed on the inner surface of the
 * machine, and the fruit is rotated against the blade to peel. When the peeling
 * blade reaches the height of the lower clamping blade, then the machine will
 * go to next stage.
 *
 * ==============================  STATE: EXITING ==============================
 * If the above stage finishes, the machine will go to the EXITING stage.
 * 4. (STATE: DOWN) The upper and lower clamping blades will work in
 * coordination to lower the fruit, and the fruit will be placed back to the
 * holding bowl. When the pressure sensor on the upper clamp blade detects a
 * surge in pressure, it is when the
 * fruit reached the holding bowl.
 * TODO: We have not determined how to avoid the peeling blade when lowering the
 * fruit.
 * 5. (STATE: OPEN) The upper and lower clamping blades will open to release the
 * fruit. We place two sensors on the end of the upper and lower clamping blades
 * to detect when they should stop openning. When the sensors are triggered
 * (SIGNAL: DOWN_LIMIT_EXCEEDED), the machine will go to next stage.
 *
 * @note If the start-stop button is pressed during stage 1 (STATE: CLOSE), the
 * machine will jump to stage 5 (STATE: OPEN).
 * @note If the start-stop button is pressed during stage 2 (STATE: UP), the
 * machine will jump to stage 4 (STATE: DOWN).
 * @note If the start-stop button is pressed during stage 3 (STATE: PEEL), the
 * machine will jump to stage 4 (STATE: DOWN).
 *
 * @note By the above jumping logic, the OPEN and DOWN stages should not assume
 * any initial position of the blades and be capable of exiting from RUNNING
 * state from any initial position.
 *
 * =============================  STATE: STOPPED =============================
 * If the above stage finishes, the machine will go to the STATE: STOPPED stage
 *
 * =============================  STATE: CLEANING =============================
 * If the self cleaning button is pressed,
 * when the machine is in STOPPED state, the machine will go to the STATE:
 * CLEANING stage. If the machine is not in STOPPED state, the machine will
 * ignore the self cleaning button.
 *
 * In the cleaning state, the machine will go over several stages:
 * 1. (STATE: CLEANING) The upper and lower clamping blades will close to cover
 * the side breach of the peeling blade.
 * TODO: water proof for the lower clamp blade motor.
 * TODO: We have not determined how to detect when to stop closing the clamping
 * blades.
 * 2. (STATE: CLEANING) The small water pumps attached onto the inner surface of
 * the machine will spray water to clean the peeling blade.
 * 3. (STATE: CLEANING) The upper and lower clamping blades will open to their
 * rested postition.
 *
 * @details System design considerations:
 * The inter-state transition is controlled either by external button signals or
 * internal sensor signals. The external button signals are read in the loop
 * function, while the internal sensor signals are read in the motor control
 * loop.
 *
 * The state are designed so that external button signals can determine the
 * state to transition to. For example, when the start-stop button is pressed in
 * CLOSE state (STATE: CLOSE), the machine will jump to stage 5 (STATE: OPEN).
 * But any finer details of which sub state it is in (e.g. UP_BLADE_DOWN or
 * LOWER_BLADE_UP) is not considered.
 *
 * The signals are categorized into main signals and sub signals, in terms of
 * functionality. The main signals are the signals that are in the loop function
 * used to control the state machine. The sub signals are the signals that are
 * used inside of the motor control loop. The external button signals are all
 * main signals, but the internal sensor signals can either be main signals or
 * sub signals.
 *
 * Some variables are needed to be initialized when entering a state, to
 * determine the sub stage or sub actions in the state. These variables are
 * called state variables. The state variables are initialized in the loop
 * function, and are used in the motor control loop.
 *
 * The state variables are:
 * 1. STATE: CLOSE.
 * @param close_upper: initialized to true. when the upper clamping blade
 * touches the fruit, it is set to false.
 * 2. STATE: UP.
 * @param height_remaining_upper: initialized to the height that the upper
 * clamping blade should move up.
 * @param height_remaining_lower: initialized to the height that the lower
 * clamping blade should move up.
 * @param height_remaining_peel: initialized to the height that the peeling
 * blade should move up.
 *
 * 3. STATE: PEEL.
 * @param height_remaining_peel: initialized to the height that the peeling
 * blade should move down.
 *
 * 4. STATE: DOWN.
 * @param height_remaining_upper: initialized to the height that the upper
 * clamping blade should move down.
 * @param height_remaining_lower: initialized to the height that the lower
 * clamping blade should move down.
 * @param height_remaining_peel: initialized to the height that the peeling
 * blade should move down.
 *
 *
 * @version 0.1
 * @date 2023-07-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <Arduino.h>
#include <FlexiTimer2.h>

#include "motor_utils.hpp"
#include "state_control.hpp"

#include "constants.hpp"
#include "globals.hpp"

auto state_machine = StateMachine(State::STOPPED);

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

MotorControl motor_upper{pin_in2_upper, pin_ena_upper, pin_in1_upper};
MotorControl motor_lower{pin_in3_lower, pin_enb_lower, pin_in4_lower};
MotorControl motor_peel{pin_in3_peel, pin_enb_peel, pin_in4_peel};

/********************* GLOBAL VARIABLES *********************
 ***********************************************************/
volatile unsigned int counter_upper = 0;
volatile unsigned int counter_lower = 0;
volatile unsigned int counter_peel = 0;

int pwm_upper = 0;
int pwm_lower = 0;
int pwm_peel = 0;
int pwm_rotate = 0;

int pressure = 0;

// control interval
const unsigned long control_interval_motors = 50;

/************************** STATE VARIABLES ****************/
bool close_upper = true;

float height_remaining_upper = 0;
float height_remaining_lower = 0;
float height_remaining_peel = 0;

/********************* FUNCTIONS *********************
 ***********************************************************/

void init_pins();
void reset_motors();

void motor_control_loop();

void counter_upper_increment() { counter_upper++; }
void counter_lower_increment() { counter_lower++; }
void counter_peel_increment() { counter_peel++; }

void setup() {
    Serial.begin(115200);
    init_pins();
    reset_motors();

    // run motor controlling loop using FlexiTimer
    FlexiTimer2::set(control_interval_motors, motor_control_loop);
    FlexiTimer2::start();

    // attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(pin_encoder_upper),
                    counter_upper_increment, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_lower),
                    counter_lower_increment, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_peel),
                    counter_peel_increment, RISING);
}

void loop() {
    // check external signals
    static bool last_start_stop_button_state = LOW;
    static bool last_self_clean_button_state = LOW;

    bool start_stop_button_state = digitalRead(pin_start_stop_button);
    bool self_clean_button_state = digitalRead(pin_self_cleaning_button);

    if (start_stop_button_state != last_start_stop_button_state) {
        if (start_stop_button_state == HIGH) {
            reset_motors();
            state_machine.trigger_signal(Signal::BUTTON_START_STOP);
        }
        last_start_stop_button_state = start_stop_button_state;
        last_self_clean_button_state = self_clean_button_state;
        delay(50);
        return;
    }

    if (self_clean_button_state != last_self_clean_button_state) {
        if (self_clean_button_state == HIGH) {
            reset_motors();
            state_machine.trigger_signal(Signal::BUTTON_SELF_CLEAN);
        }
        last_start_stop_button_state = start_stop_button_state;
        last_self_clean_button_state = self_clean_button_state;
        delay(50);
        return;
    }

    // read pressure
    pressure = analogRead(pin_upper_pressure_sensor);
    if (pressure < pressure_threshold_lower) {
        state_machine.trigger_signal(Signal::PRESSURE_LOWER_THRESHOLD_EXCEEDED);
    }
}

void init_pins() {
    pinMode(pin_start_stop_button, INPUT);
    pinMode(pin_self_cleaning_button, INPUT);

    pinMode(pin_ena_upper, OUTPUT);
    pinMode(pin_in1_upper, OUTPUT);
    pinMode(pin_in2_upper, OUTPUT);
    pinMode(pin_in3_lower, OUTPUT);
    pinMode(pin_in4_lower, OUTPUT);
    pinMode(pin_enb_lower, OUTPUT);

    pinMode(pin_ena_rotate, OUTPUT);
    pinMode(pin_in1_rotate, OUTPUT);
    pinMode(pin_in2_rotate, OUTPUT);
    pinMode(pin_in3_peel, OUTPUT);
    pinMode(pin_in4_peel, OUTPUT);
    pinMode(pin_enb_peel, OUTPUT);

    pinMode(pin_encoder_upper, INPUT);
    pinMode(pin_encoder_lower, INPUT);
    pinMode(pin_encoder_peel, INPUT);

    pinMode(pin_screw_rod_upper, INPUT);
    pinMode(pin_screw_rod_lower, INPUT);
    pinMode(pin_screw_rod_peel, INPUT);

    pinMode(pin_upper_pressure_sensor, INPUT);

    // pinMode(pin_water_pump_1, OUTPUT);
}

void reset_motors() {
    motor_upper.set_direction(Direction::STOP);
    motor_lower.set_direction(Direction::STOP);
    motor_peel.set_direction(Direction::STOP);
    // rotate
}

void close();
void up();
void peel();
void down();
void open();

void motor_control_loop() {
    /// TODO: change the motor control loop to a pointer that points to
    /// close(), up() functions
    switch (state_machine.get_state()) {
    case State::CLOSE:
        close();
        break;

    case State::UP:
        up();
        break;

    case State::PEEL:
        peel();
        break;
    
    case State::DOWN:
        down();
        break;
    
    case State::OPEN:
        open();
        break;
    
    default:
        break;
    }
}

void close() {
    if (close_upper) {
        float rpm_cur_upper = counter_upper /
                              ((float)control_interval_motors / 1000) * 60 /
                              cpr / grr;
        float rpm_cur_peel = counter_peel /
                             ((float)control_interval_motors / 1000) * 60 /
                             cpr / grr;

        counter_upper = 0;
        counter_peel = 0;

        motor_upper.set_rpm(rpm_cur_upper, rpm_target_close);
        motor_peel.set_rpm(rpm_cur_peel, rpm_target_close);

        if (pressure < pressure_touch) {
            // sub state transition
            // init state variables
            close_upper = false;

            // init motor state
            motor_upper.set_direction(Direction::STOP);
            motor_peel.set_direction(Direction::STOP);
            motor_lower.set_direction(Direction::UP);
        }
    } else {
        float rpm_cur_lower = counter_lower /
                              ((float)control_interval_motors / 1000) * 60 /
                              cpr / grr;

        counter_lower = 0;

        motor_lower.set_rpm(rpm_cur_lower, rpm_target_close);

        if (pressure < pressure_grasp) {
            state_machine.next();
        }
    }
}

void up() {
    if (height_remaining_upper <= 0) {
        motor_upper.set_direction(Direction::STOP);
    }
    if (height_remaining_lower <= 0) {
        motor_lower.set_direction(Direction::STOP);
    }
    if (height_remaining_peel <= 0) {
        motor_peel.set_direction(Direction::STOP);
    }

    if (height_remaining_upper < 0 && height_remaining_lower < 0 &&
        height_remaining_peel < 0) {
        state_machine.next();
    }
    float rpm_cur_upper = (float)counter_upper / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_lower = counter_lower / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                         ((float)control_interval_motors / 1000) * 60;

    height_remaining_upper -= (float)counter_upper / cpr / grr * screw_lead;
    height_remaining_lower -= (float)counter_lower / cpr / grr * screw_lead;
    height_remaining_peel -= (float)counter_peel / cpr / grr * screw_lead;

    counter_upper = 0;
    counter_lower = 0;
    counter_peel = 0;

    motor_upper.set_rpm(rpm_cur_upper, rpm_target_up_down);
    motor_lower.set_rpm(rpm_cur_lower, rpm_target_up_down);
    motor_peel.set_rpm(rpm_cur_peel, rpm_target_up_down);
}

void peel() {
    if (height_remaining_peel <= 0) {
        motor_peel.set_direction(Direction::STOP);
        state_machine.next();
    }

    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                         ((float)control_interval_motors / 1000) * 60;

    height_remaining_peel -= (float)counter_peel / cpr / grr * screw_lead;

    counter_peel = 0;

    motor_peel.set_rpm(rpm_cur_peel, rpm_target_peel);
}

void down() {
    if (pressure < pressure_touch_bowl) {
        motor_upper.set_direction(Direction::STOP);
        motor_lower.set_direction(Direction::STOP);
        motor_peel.set_direction(Direction::STOP);
        state_machine.next();
    }
    float rpm_cur_upper = (float)counter_upper / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_lower = counter_lower / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    // float rpm_cur_peel = (float)counter_peel / cpr / grr /
    //                      ((float)control_interval_motors / 1000) * 60;

    height_remaining_upper -= (float)counter_upper / cpr / grr * screw_lead;
    height_remaining_lower -= (float)counter_lower / cpr / grr * screw_lead;
    // height_remaining_peel -= (float)counter_peel / cpr / grr * screw_lead;

    counter_upper = 0;
    counter_lower = 0;
    counter_peel = 0;

    motor_upper.set_rpm(rpm_cur_upper, rpm_target_up_down);
    motor_lower.set_rpm(rpm_cur_lower, rpm_target_up_down);
    // motor_peel.set_rpm(rpm_cur_peel, rpm_target_up_down);
}

void open() {
    int pressure_screw_rod_upper = analogRead(pin_screw_rod_upper);
    int pressure_screw_rod_lower = analogRead(pin_screw_rod_lower);
    int pressure_screw_rod_peel = analogRead(pin_screw_rod_peel);

    if (pressure_screw_rod_upper < pressure_threshold_lower) {
        motor_upper.set_direction(Direction::STOP);
    }
    if (pressure_screw_rod_lower < pressure_threshold_lower) {
        motor_lower.set_direction(Direction::STOP);
    }
    if (pressure_screw_rod_peel < pressure_threshold_lower) {
        motor_peel.set_direction(Direction::STOP);
    }
    if (pressure_screw_rod_upper < pressure_threshold_lower &&
        pressure_screw_rod_lower < pressure_threshold_lower &&
        pressure_screw_rod_peel < pressure_threshold_lower) {
        state_machine.next();
    }

    float rpm_cur_upper = (float)counter_upper / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_lower = counter_lower / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                            ((float)control_interval_motors / 1000) * 60;

    counter_upper = 0;
    counter_lower = 0;
    counter_peel = 0;

    motor_upper.set_rpm(rpm_cur_upper, rpm_target_open);
    motor_lower.set_rpm(rpm_cur_lower, rpm_target_open);
    motor_peel.set_rpm(rpm_cur_peel, rpm_target_open);

}