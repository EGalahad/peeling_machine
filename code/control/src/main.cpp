/**
 * @file main.cpp
 * @author elijah
 * @brief This is the overall control code for the peeling machine, based on
 * Arduino.
 *
 * @details
 * The machine exposes two buttons, a start-stop button and a self cleaning
 * button.
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
 * blades should be descended to to finish the peeling process).
 * The peeling starts and the lower clamping blades will rotate the fruit while
 * the peeling blades move downwards. The peeling blade is placed on the inner
 * surface of the machine, and the fruit is rotated against the blade to peel.
 * When the peeling blade reaches the height of the lower clamping blade, the
 * machine will go to next stage.
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
 *
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
 * @copyright Copyright (c) 2023
 *
 */
#include <Arduino.h>
#include <FlexiTimer2.h>

#include "motor_utils.hpp"
#include "sensor_utils.hpp"
#include "state_control.hpp"

#include "constants.hpp"
#include "globals.hpp"
#include "pins.hpp"

StateMachine state_machine{State::STOPPED};

MotorControl motor_upper{pin_in2_upper, pin_in1_upper, pin_ena_upper};
MotorControl motor_lower{pin_in3_lower, pin_in4_lower, pin_enb_lower};
MotorControl motor_peel{pin_in2_peel, pin_in1_peel, pin_ena_peel};
MotorControl motor_rotate_lower{pin_in3_rotate_lower,
                                pin_in4_rotate_lower,
                                pin_enb_rotate_lower,
                                10,
                                0.1,
                                0.1};

PressureSensor pressure_sensor{pin_pressure_clamp_upper_dout, pin_pressure_clamp_upper_sck};
TouchSensor touch_sensor_upper{pin_touch_screw_rod_upper};
TouchSensor touch_sensor_lower{pin_touch_screw_rod_lower};
TouchSensor touch_sensor_peel{pin_touch_screw_rod_peel};

/********************* GLOBAL VARIABLES *********************
 ***********************************************************/
volatile unsigned int counter_upper = 0;
volatile unsigned int counter_lower = 0;
volatile unsigned int counter_peel = 0;
volatile unsigned int counter_rotate = 0;

// // read of the pressure sensor on the upper clamping blade
// int pressure = 0;
// int pressure_screw_rod_upper = 0;
// int pressure_screw_rod_lower = 0;
// int pressure_screw_rod_peel = 0;

// control interval
const unsigned long control_interval_motors = 50;

/************************** STATE VARIABLES ****************/
bool close_upper = true;
bool peel_down_downwards = true;

float height_upper = 0;
float height_lower = 0;
float height_peel = 0;

float height_to_peel = height_total;

float spray_time_start = 0;
bool close_cleaning = false;
bool spray_cleaning = false;
bool open_cleaning = false;

/********************* FUNCTIONS *********************
 ***********************************************************/

void init_pins();
void reset_motors();

void motor_control_loop();

void counter_upper_increment() { counter_upper++; }
void counter_lower_increment() { counter_lower++; }
void counter_peel_increment() { counter_peel++; }
void counter_rotate_increment() { counter_rotate++; }

#define PRESSURE_DEBUG
#ifdef PRESSURE_DEBUG
void setup() {
    Serial.begin(115200);
    init_pins();
    reset_motors();
}

void loop() {
    // output pressure sensor readings
    Serial.print("pressure: ");
    Serial.println(pressure_sensor.get_pressure());
    delay(100);
}
#else
void setup() {
    Serial.begin(115200);
    init_pins();
    reset_motors();

    // run motor controlling loop using FlexiTimer
    FlexiTimer2::set(control_interval_motors, motor_control_loop);

    // attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(pin_encoder_upper),
                    counter_upper_increment, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_lower),
                    counter_lower_increment, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_peel),
                    counter_peel_increment, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_rotate),
                    counter_rotate_increment, RISING);
}

void loop() {
    // constantly check external signals
    static bool last_start_stop_button_state =
        digitalRead(pin_start_stop_button);
    static bool last_self_clean_button_state =
        digitalRead(pin_self_cleaning_button);

    bool start_stop_button_state = digitalRead(pin_start_stop_button);
    bool self_clean_button_state = digitalRead(pin_self_cleaning_button);

    if (start_stop_button_state != last_start_stop_button_state) {
        if (start_stop_button_state == HIGH) {
            reset_motors();
            state_machine.trigger_signal(Signal::BUTTON_START_STOP);
            Serial.println("start_stop button pressed!");
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
            Serial.println("self_clean button pressed!");
        }
        last_start_stop_button_state = start_stop_button_state;
        last_self_clean_button_state = self_clean_button_state;
        delay(50);
        return;
    }

    // read pressure
    int pressure = pressure_sensor.get_pressure();

    if (pressure < pressure_threshold_lower) {
        state_machine.trigger_signal(Signal::PRESSURE_LOWER_THRESHOLD_EXCEEDED);
        Serial.println("Pressure lower threshold exceeded! ");
        Serial.print("pressure=");
        Serial.print(pressure);
        Serial.println();
    }

    static bool motor_control_loop_started = false;
    if (not motor_control_loop_started) {
        state_machine.reset();
        FlexiTimer2::start();
        motor_control_loop_started = true;
    }

    static unsigned long last_log_time = millis();
    if (millis() - last_log_time > 1000) {
        Serial.println();
        Serial.print("state: ");
        switch (state_machine.get_state()) {
        case State::STOPPED:
            Serial.println("STOPPED");
            break;
        case State::CLOSE:
            Serial.println("CLOSE");
            break;
        case State::UP:
            Serial.println("UP");
            break;
        case State::PEEL:
            Serial.println("PEEL");
            break;
        case State::DOWN:
            Serial.println("DOWN");
            break;
        case State::OPEN:
            Serial.println("OPEN");
            break;
        case State::CLEANING:
            Serial.println("CLEANING");
            break;
        case State::ERROR:
            Serial.println("ERROR");
            break;

        default:
            break;
        }
        Serial.print("height_upper: ");
        Serial.print(height_upper);
        Serial.print(", height_lower: ");
        Serial.print(height_lower);
        Serial.print(", height_peel: ");
        Serial.print(height_peel);
        Serial.print(", height_to_peel: ");
        Serial.println(height_to_peel);

        Serial.print("pressure: ");
        Serial.print(pressure);
        Serial.println();

        last_log_time = millis();
    }
}
#endif // PRESSURE_DEBUG

void init_pins() {
    pinMode(pin_start_stop_button, INPUT);
    pinMode(pin_self_cleaning_button, INPUT);

    pinMode(pin_ena_upper, OUTPUT);
    pinMode(pin_in1_upper, OUTPUT);
    pinMode(pin_in2_upper, OUTPUT);
    pinMode(pin_in3_lower, OUTPUT);
    pinMode(pin_in4_lower, OUTPUT);
    pinMode(pin_enb_lower, OUTPUT);

    pinMode(pin_ena_peel, OUTPUT);
    pinMode(pin_in1_peel, OUTPUT);
    pinMode(pin_in2_peel, OUTPUT);
    pinMode(pin_in3_rotate_lower, OUTPUT);
    pinMode(pin_in4_rotate_lower, OUTPUT);
    pinMode(pin_enb_rotate_lower, OUTPUT);

    pinMode(pin_encoder_upper, INPUT);
    pinMode(pin_encoder_lower, INPUT);
    pinMode(pin_encoder_peel, INPUT);
    pinMode(pin_encoder_rotate, INPUT);

    pinMode(pin_touch_screw_rod_upper, INPUT);
    pinMode(pin_touch_screw_rod_lower, INPUT);
    pinMode(pin_touch_screw_rod_peel, INPUT);

    pinMode(pin_pressure_clamp_upper_dout, INPUT);
    pinMode(pin_pressure_clamp_upper_sck, INPUT);

    // pinMode(pin_water_pump_1, OUTPUT);
}

void reset_motors() {
    motor_upper.set_direction(Direction::STOP);
    motor_lower.set_direction(Direction::STOP);
    motor_peel.set_direction(Direction::STOP);
    motor_rotate_lower.set_direction(Direction::STOP);

    motor_upper.reset_pid_states();
    motor_lower.reset_pid_states();
    motor_peel.reset_pid_states();
    motor_rotate_lower.reset_pid_states();
}

void close();
void up();
void peel();
void down();
void open();
void clean();

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

    case State::CLEANING:
        clean();
        break;

    default:
        break;
    }
}

/**
 * @brief The upper and lower clamping blades will close to hold the fruit in
 * place. The peeling blades will also be moved down with the upper clamping
 * blade.
 */
void close() {
    if (close_upper) {
        float rpm_cur_upper = counter_upper /
                              ((float)control_interval_motors / 1000) * 60 /
                              cpr / grr;
        float rpm_cur_peel = counter_peel /
                             ((float)control_interval_motors / 1000) * 60 /
                             cpr / grr;

        height_to_peel -= (float)counter_upper / cpr / grr * screw_lead;

        counter_upper = 0;
        counter_peel = 0;

        motor_upper.set_rpm(rpm_cur_upper, rpm_target_close);
        motor_peel.set_rpm(rpm_cur_peel, rpm_target_close);

        // the upper clamping blade touches the fruit
        if (pressure_sensor.get_pressure() < pressure_touch) {
            // sub state transition
            // init state variables
            close_upper = false;

            // init motor state
            motor_upper.set_direction(Direction::STOP);
            motor_peel.set_direction(Direction::STOP);
            motor_lower.set_direction(Direction::UP);

            Serial.print("pressure=");
            Serial.print(pressure_sensor.get_pressure());
            Serial.print(" < pressure_touch=");
            Serial.print(pressure_touch);

            Serial.println(
                ", Entering sub state transition: close lower clamping blade!");
        }
    } else {
        float rpm_cur_lower = counter_lower /
                              ((float)control_interval_motors / 1000) * 60 /
                              cpr / grr;

        height_to_peel -= (float)counter_lower / cpr / grr * screw_lead;

        counter_lower = 0;

        motor_lower.set_rpm(rpm_cur_lower, rpm_target_close);

        // the lower clamping blade secures the fruit
        if (pressure_sensor.get_pressure() < pressure_grasp)
            reset_motors(), state_machine.next();
    }
}

/**
 * @brief The upper and lower clamping blades will work in coordination to lift
 * up the fruit, for some fixed distance (This should be implemented by
 * recording the vertical distance moved in the prebious stage in the motor
 * control loop)
 */
void up() {
    float rpm_cur_upper = (float)counter_upper / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_lower = (float)counter_lower / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                         ((float)control_interval_motors / 1000) * 60;

    height_upper += (float)counter_upper / cpr / grr * screw_lead;
    height_lower += (float)counter_lower / cpr / grr * screw_lead;
    height_peel += (float)counter_peel / cpr / grr * screw_lead;

    counter_upper = 0;
    counter_lower = 0;
    counter_peel = 0;

    motor_upper.set_rpm(rpm_cur_upper, rpm_target_up_down);
    motor_lower.set_rpm(rpm_cur_lower, rpm_target_up_down);
    motor_peel.set_rpm(rpm_cur_peel, rpm_target_up_down);

    if (height_upper >= height_up)
        motor_upper.set_direction(Direction::STOP);
    if (height_lower >= height_up)
        motor_lower.set_direction(Direction::STOP);
    if (height_peel >= height_up)
        motor_peel.set_direction(Direction::STOP);
    if (height_upper >= height_up and height_lower >= height_up and
        height_peel >= height_up)
        reset_motors(), state_machine.next();
}

/**
 * @brief The clamping blades will rotate the fruit while the peeling blade is
 * moved down.
 */
void peel() {
    motor_peel.set_direction(Direction::DOWN);

    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                         ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_rotate = (float)counter_rotate / cpr_rotate / grr_rotate /
                           ((float)control_interval_motors / 1000) * 60;

    height_peel -= (float)counter_peel / cpr / grr * screw_lead;

    motor_peel.set_rpm(rpm_cur_peel, rpm_target_peel);
    motor_rotate_lower.set_rpm(rpm_cur_rotate, rpm_target_rotate);

    if (height_peel <= height_up - height_to_peel)
        reset_motors(), state_machine.next();

    static unsigned long last_log_time = millis();
    if (millis() - last_log_time > 1000) {
        last_log_time = millis();
        Serial.print("counter_peel: ");
        Serial.print(counter_peel);
        Serial.print(", counter_rotate: ");
        Serial.print(counter_rotate);
        Serial.print(", rpm_cur_peel: ");
        Serial.print(rpm_cur_peel);
        Serial.print(", rpm_cur_rotate: ");
        Serial.println(rpm_cur_rotate);
    }

    counter_peel = 0;
    counter_rotate = 0;
}

/**
 * @brief The upper and lower clamping blades will work in coordination to lower
 * the fruit, and the fruit will be placed back to the holding bowl.
 */
void down() {
    float rpm_cur_upper = (float)counter_upper / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_lower = (float)counter_lower / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                         ((float)control_interval_motors / 1000) * 60;

    height_upper -= (float)counter_upper / cpr / grr * screw_lead;
    height_lower -= (float)counter_lower / cpr / grr * screw_lead;
    height_peel -= (float)counter_peel / cpr / grr * screw_lead *
                   (peel_down_downwards ? 1 : -1);

    motor_upper.set_rpm(rpm_cur_upper, rpm_target_up_down);
    motor_lower.set_rpm(rpm_cur_lower, rpm_target_up_down);
    motor_peel.set_rpm(rpm_cur_peel, rpm_target_up_down);

    bool peel_zero = (peel_down_downwards ? 1 : -1) * height_peel <= 0;
    bool touch_fruit = pressure_sensor.get_pressure() < pressure_touch_bowl;
    if (peel_zero and touch_fruit)
        reset_motors(), state_machine.next();
    if (peel_zero)
        motor_peel.set_direction(Direction::STOP);
    if (touch_fruit)
        motor_upper.set_direction(Direction::STOP),
            motor_lower.set_direction(Direction::STOP);

    static unsigned long last_log_time = millis();
    if (millis() - last_log_time > 1000) {
        last_log_time = millis();
        Serial.print("counter_upper: ");
        Serial.print(counter_upper);
        Serial.print(", counter_lower: ");
        Serial.print(counter_lower);
        Serial.print(", counter_peel: ");
        Serial.print(counter_peel);
        Serial.print(", rpm_cur_upper: ");
        Serial.print(rpm_cur_upper);
        Serial.print(", rpm_cur_lower: ");
        Serial.print(rpm_cur_lower);
        Serial.print(", rpm_cur_peel: ");
        Serial.println(rpm_cur_peel);
    }

    counter_upper = 0;
    counter_lower = 0;
    counter_peel = 0;
}

void open() {
    motor_upper.set_direction(Direction::UP);
    motor_lower.set_direction(Direction::DOWN);
    motor_peel.set_direction(Direction::UP);

    float rpm_cur_upper = (float)counter_upper / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_lower = (float)counter_lower / cpr / grr /
                          ((float)control_interval_motors / 1000) * 60;
    float rpm_cur_peel = (float)counter_peel / cpr / grr /
                         ((float)control_interval_motors / 1000) * 60;

    motor_upper.set_rpm(rpm_cur_upper, rpm_target_open);
    motor_lower.set_rpm(rpm_cur_lower, rpm_target_open);
    motor_peel.set_rpm(rpm_cur_peel, rpm_target_open);

    if (touch_sensor_upper.is_touched() && touch_sensor_lower.is_touched() &&
        touch_sensor_peel.is_touched())
        reset_motors(), state_machine.next();
    if (touch_sensor_upper.is_touched())
        motor_upper.set_direction(Direction::STOP);
    if (touch_sensor_lower.is_touched())
        motor_lower.set_direction(Direction::STOP);
    if (touch_sensor_peel.is_touched())
        motor_peel.set_direction(Direction::STOP);

    static unsigned long last_log_time = millis();
    if (millis() - last_log_time > 1000) {
        last_log_time = millis();
        Serial.print("counter_upper: ");
        Serial.print(counter_upper);
        Serial.print(", counter_lower: ");
        Serial.print(counter_lower);
        Serial.print(", counter_peel: ");
        Serial.print(counter_peel);
        Serial.print(", rpm_cur_upper: ");
        Serial.print(rpm_cur_upper);
        Serial.print(", rpm_cur_lower: ");
        Serial.print(rpm_cur_lower);
        Serial.print(", rpm_cur_peel: ");
        Serial.println(rpm_cur_peel);
    }

    counter_upper = 0;
    counter_lower = 0;
    counter_peel = 0;
}

void clean() {
    if (close_cleaning) {
        motor_upper.set_direction(Direction::DOWN);
        motor_peel.set_direction(Direction::DOWN);
        motor_lower.set_direction(Direction::UP);

        float rpm_cur_upper = (float)counter_upper / cpr / grr /
                              ((float)control_interval_motors / 1000) * 60;
        float rpm_cur_lower = (float)counter_lower / cpr / grr /
                              ((float)control_interval_motors / 1000) * 60;
        float rpm_cur_peel = (float)counter_peel / cpr / grr /
                             ((float)control_interval_motors / 1000) * 60;

        motor_upper.set_rpm(rpm_cur_upper, rpm_target_close);
        motor_peel.set_rpm(rpm_cur_peel, rpm_target_close);
        motor_lower.set_rpm(rpm_cur_lower, rpm_target_close);

        height_upper -= (float)counter_upper / cpr / grr * screw_lead;
        height_peel -= (float)counter_peel / cpr / grr * screw_lead;
        height_lower += (float)counter_lower / cpr / grr * screw_lead;

        if (height_upper < -height_to_descend) {
            motor_upper.set_direction(Direction::STOP);
        }
        if (height_peel < -height_to_descend) {
            motor_peel.set_direction(Direction::STOP);
        }
        if (height_lower > height_to_descend) {
            motor_lower.set_direction(Direction::STOP);
        }
        if (height_upper < -height_to_descend and
            height_peel < -height_to_descend and
            height_lower > height_to_descend) {
            // sub state transition
            // init state variables
            close_cleaning = false;
            spray_cleaning = true;
            spray_time_start = millis();

            // init motor state
            motor_upper.set_direction(Direction::STOP);
            motor_peel.set_direction(Direction::STOP);
            motor_lower.set_direction(Direction::STOP);

            Serial.println("Entering sub state transition: spray water!");
        }
    } else if (spray_cleaning) {
        // spray water for some time
        if (millis() - spray_time_start < spray_time_default * 1000) {
            // spray water
            Serial.println("Spraying water!");
            delay(1000);
        } else {
            // sub state transition
            // init state variables
            spray_cleaning = false;
            open_cleaning = true;

            // init motor state
            motor_upper.set_direction(Direction::UP);
            motor_peel.set_direction(Direction::UP);
            motor_lower.set_direction(Direction::DOWN);

            Serial.println("Entering sub state transition: open!");
        }
    } else if (open_cleaning) {
        motor_upper.set_direction(Direction::UP);
        motor_peel.set_direction(Direction::UP);
        motor_lower.set_direction(Direction::DOWN);

        float rpm_cur_upper = (float)counter_upper / cpr / grr /
                              ((float)control_interval_motors / 1000) * 60;
        float rpm_cur_lower = (float)counter_lower / cpr / grr /
                              ((float)control_interval_motors / 1000) * 60;
        float rpm_cur_peel = (float)counter_peel / cpr / grr /
                             ((float)control_interval_motors / 1000) * 60;

        motor_upper.set_rpm(rpm_cur_upper, rpm_target_open);
        motor_peel.set_rpm(rpm_cur_peel, rpm_target_open);
        motor_lower.set_rpm(rpm_cur_lower, rpm_target_open);

        if (touch_sensor_upper.is_touched() &&
            touch_sensor_lower.is_touched() && touch_sensor_peel.is_touched()) {
            // sub state transition
            // init state variables
            open_cleaning = false;

            // init motor state
            motor_upper.set_direction(Direction::STOP);
            motor_peel.set_direction(Direction::STOP);
            motor_lower.set_direction(Direction::STOP);

            Serial.println("Clean finished!");
        }
        if (touch_sensor_upper.is_touched())
            motor_upper.set_direction(Direction::STOP);
        if (touch_sensor_lower.is_touched())
            motor_lower.set_direction(Direction::STOP);
        if (touch_sensor_peel.is_touched())
            motor_peel.set_direction(Direction::STOP);
    }
}
