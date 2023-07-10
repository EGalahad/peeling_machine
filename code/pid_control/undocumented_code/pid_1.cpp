#include <Arduino.h>

#define debug true // for print statements

int INT_PIN_MOTOR1 = 2; // INT 0 for motor 1 - is on D2
// int INT_PIN_MOTOR2 = 3;      // INT 1 for motor 2 - is on D3
volatile unsigned int counter1 = 0; // pulse count for motor 1

unsigned long ul_PreviousMillis =
    0UL; // using millis rollver code from
         // http://playground.arduino.cc/Code/TimingRollover
unsigned long ul_Interval =
    50UL; // 50ms this is delay between corrections - non blocking to allow
          // interrupts to count encoder pulses

// motor control pins in order of the L298N control boards
int pin_ENA = 9; // PWM pin for motor 1 - must be an Arduino PWM capable pin
int pin_IN1 = 8; // direction for motor 1
int pin_IN2 = 7; // direction for motor 1
int pin_IN3 = 6; // direction for motor 2
int pin_IN4 = 5; // direction for motor 2
int pin_ENB = 4; // PWM pin for motor 2 - must be an Arduino PWM capable pin

int pin_PWM = A5;

// pwm for each motor - using Arduino PWM, so 0-255
int pwm_max = 255; // (pwm is 0-255)
int pwm_cur1 =
    pwm_max / 2; // the speed we will run at - half speed (pwm is 0-255)
int pwm_cur2 =
    pwm_max / 2;   // the speed we will run at - half speed (pwm is 0-255)
int cpr = 240;     // the click per rev for the encoder
int rpm_max = 300; // the 12V open loop RPM
int rpm_target = rpm_max / 1.2;

// motor 1 pulse count interrupt routine
void ISR_count1() {
    counter1++;
    // Serial.println("interrupt rising!");
} // ISR_count1

// setup
void setup() {
    if (debug)
        Serial.begin(115200);

    pinMode(pin_ENA, OUTPUT);
    pinMode(pin_IN1, OUTPUT);
    pinMode(pin_IN2, OUTPUT);
    pinMode(pin_IN3, OUTPUT);
    pinMode(pin_IN4, OUTPUT);
    pinMode(pin_ENB, OUTPUT);

    pinMode(pin_PWM, INPUT);

    // set the H Bridge direction for motor 1
    digitalWrite(pin_IN1, HIGH);
    digitalWrite(pin_IN2, LOW);
    // set the H Bridge direction for motor 2
    digitalWrite(pin_IN3, LOW);
    digitalWrite(pin_IN4, HIGH);

    // interrupt routine to count the pulses on the closed loop motor 1
    attachInterrupt(digitalPinToInterrupt(INT_PIN_MOTOR1), ISR_count1, RISING);

    // run the seond motor open loop
    analogWrite(pin_ENB, pwm_cur2);
} // setup

// loop
void loop() {
    unsigned long ul_CurrentMillis = millis();
    if (ul_CurrentMillis - ul_PreviousMillis > ul_Interval) {
        rpm_target = (float)analogRead(pin_PWM) / (float)1023 * (float)rpm_max;
        rpm_target = rpm_target * 2 - rpm_max;
        Serial.print("rpm_target: ");
        Serial.print(rpm_target);
        Serial.print(", ");

        if (rpm_target < 0) {
            digitalWrite(pin_IN1, LOW);
            digitalWrite(pin_IN2, HIGH);
            rpm_target = -rpm_target;
        } else {
            digitalWrite(pin_IN1, HIGH);
            digitalWrite(pin_IN2, LOW);
        }
        // the count can be used to find the counts per revolution
        if (debug)
            Serial.print("Count: ");
        // Serial.println(counter1);

        // these lines will show rpm, but zero the counter
        // Serial.print((float)ul_Interval/1000.0);
        int rpm =
            ((float)counter1 / ((float)cpr * ((float)ul_Interval / 1000.0))) *
            60.0;
        if (debug)
            Serial.print(counter1);
        if (debug)
            Serial.print(", RPM: ");
        if (debug)
            Serial.print(rpm);
        // kind of a messy P only correction
        // use the error * half the pwn range
        float error = ((float)rpm_target - (float)rpm);
        float correction = ((error / (float)rpm_max) / 2.0) * (float)pwm_max;
        pwm_cur1 = min(255, max(0, pwm_cur1 + correction));
        if (debug)
            Serial.print(", PWM: ");
        if (debug)
            Serial.println(pwm_cur1);
        analogWrite(pin_ENA, pwm_cur1);

        counter1 = 0;                 // reset the counter for the next loop
        ul_PreviousMillis = millis(); // reset the timer for the next correction
    }
} // loop

// void loop() {
//     x_pos = analogRead(x_key); // Reading the horizontal movement value
//     y_pos = analogRead(y_key); // Reading the vertical movement value

//     if (x_pos < 400) { // Rotating the left motor in clockwise direction
//         motor_speed1 = map(x_pos, 400, 0, 0,
//                           255); // Mapping the values to 0-255 to move the
//                           motor
//         digitalWrite(IN1, LOW);
//         digitalWrite(IN2, HIGH);
//         analogWrite(EN_A, motor_speed1);
//     }

//     else if (x_pos > 400 && x_pos < 600) { // Motors will not move when the
//                                            // joystick will be at center
//         digitalWrite(IN1, LOW);
//         digitalWrite(IN2, LOW);
//     }

//     else if (x_pos >
//              600) { // Rotating the left motor in anticlockwise direction
//         motor_speed1 = map(x_pos, 600, 1023, 0, 255);
//         digitalWrite(IN1, HIGH);
//         digitalWrite(IN2, LOW);
//         analogWrite(EN_A, motor_speed1);
//     }

//     if (y_pos < 400) { // Rotating the right motor in clockwise direction
//         motor_speed2 = map(y_pos, 400, 0, 0, 255);
//         digitalWrite(IN3, LOW);
//         digitalWrite(IN4, HIGH);
//         analogWrite(EN_B, motor_speed2);
//     }

//     else if (y_pos > 400 && y_pos < 600) {
//         digitalWrite(IN3, LOW);
//         digitalWrite(IN4, LOW);
//     }

//     else if (y_pos >
//              600) { // Rotating the right motor in anticlockwise direction
//         motor_speed2 = map(y_pos, 600, 1023, 0, 255);
//         digitalWrite(IN3, HIGH);
//         digitalWrite(IN4, LOW);
//         analogWrite(EN_B, motor_speed2);
//     }
// }
