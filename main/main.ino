#include <QTRSensors.h>
#include <Arduino.h>
#include <Arduino.h>
#include <avr/io.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG 1

void calibration_check();
float * pid_calc(uint16_t);
int * IR_sensors_readings();
float distance();
void motor_drive(int, int);
void printlnine();

// QTR
QTRSensors qtr;

const uint8_t SensorCount = 8; //uint8_t stands for "unsigned integer with 8 bits.
uint16_t sensorValues[SensorCount];
const uint8_t pins[] = {3,4,5,6,A2,A3,A4,A5};

//QTR calibration test values
const float maximum_calibration_required = 990;
const float minimum_calibration_required = 10;

//ultrasound
#define trig 12
#define echo 13

//tcrt5000 IR_sensor
#define IR_sensorR A0
#define IR_sensorL A1


// Definitions Arduino pins connected to input H Bridge (motors control)
const uint8_t PWM_right = 9;
const uint8_t PWM_left = 10;
const uint8_t IN1 = 6;  //motor right clockwise ( clockwise = front?)
const uint8_t IN2 = 7;  //motor right counter-clockwise ( clockwise = backwards?)
const uint8_t IN3 = 11; //motor left clockwise
const uint8_t IN4 = 2;  //motor left counter-clockwise

//PID configuration
const float base_speed = 255;
float Rspeed, Lspeed;
int pos;
int button = 3; // to be pressed to find set point

#define trageted_qtr_value 3500
float proportional;
float derivative;
float error, prevError = 0.0, integral = 0.0;
float correction;

float Kp = 1;
float Ki = 0.7;
float Kd = 0.07;

void setup() {
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins(pins, SensorCount);
    // qtr.setEmitterPin(2); //    "IR LEDs" PIN ON THE QRT(the pin between the vcc and pin number 1)!!

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
    // = ~25 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    //configuration the pins of the ultrasound
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    //configuration the pins of the IR sensors
    pinMode(IR_sensorL, INPUT);
    pinMode(IR_sensorL, INPUT);

    // Set the output pins for pont-H
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);


    // print the calibration minimum values measured when emitters were on
    Serial.begin(9600);

    //reading the calibration data
    calibration_check();

    delay(1000);
}

void loop() {
    // read calibrated QTR sensor values and obtain a measure of the line position
    // from 0 to 7000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensorValues);

    if (DEBUG) {
        // print the sensor values as numbers from 0 to 1000, where 0 means maximum
        // reflectance and 1000 means minimum reflectance, followed by the line position
        for (uint8_t i = 0; i < SensorCount; i++) {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
        }
        Serial.println(position);
    }

    //drive
    float * speed_values = pid_calc(position);
    int speedR = (int) speed_values;
    int speedL = (int)(speed_values + 1);
    if (DEBUG) {
        Serial.print("R:");
        Serial.print(speedR);
        Serial.print("  L:");
        Serial.print(speedL);
    }
    printlnine();
    motor_drive(speedR, speedL);

    //distance from the ultrasound to wall using ultrasound
    float distance = distance_of_the_nearset_object_that_the_left();
    if (DEBUG) {
        Serial.print("distance:");
        Serial.print(distance);
    }
    printlnine();

    //detecting the wall using ultrasound
    int * sensors_values = IR_sensors_readings();
    if (DEBUG) {
        Serial.print("R:");
        Serial.print(*sensors_values);
        Serial.print("  L:");
        Serial.print(*(sensors_values + 1));
    }
    printlnine();

    //!!change this later
    delay(1250);
}

void calibration_check() {
    float calibrationOn_minimum;
    float calibrationOn_maximum;
    float calibrationOn_minimum_total = 0;
    float calibrationOn_maximum_total = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
        calibrationOn_minimum = qtr.calibrationOn.minimum[i];
        calibrationOn_minimum_total += calibrationOn_minimum;

        if (DEBUG) {
            Serial.print(calibrationOn_minimum);
            Serial.print(' ');
        }
    }
    printlnine();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++) {
        calibrationOn_maximum = qtr.calibrationOn.maximum[i];
        calibrationOn_maximum_total += calibrationOn_maximum;

        if (DEBUG) {
            Serial.print(calibrationOn_maximum);
            Serial.print(' ');
        }
    }
    printlnine();
    printlnine();

    //checking the calibration
    if (calibrationOn_maximum_total / SensorCount >= maximum_calibration_required && calibrationOn_minimum_total / SensorCount <= minimum_calibration_required) {
        while (true) {

            if (DEBUG) {
                Serial.print("BAD calibration!!");
                Serial.println();
                Serial.print(calibrationOn_maximum_total / SensorCount);
                Serial.print(' ');
                Serial.print(calibrationOn_minimum_total / SensorCount);
                Serial.println();
            }

            delay(500);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

}

float * pid_calc(uint16_t qtr_position) {
    error = trageted_qtr_value - qtr_position;

    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - prevError);
    prevError = error;

    correction = proportional + integral + derivative;

    Rspeed = base_speed + correction;
    Lspeed = base_speed - correction;

    //  motor_drive(Rspeed, Lspeed);  
    float speeds[] = {Rspeed,Lspeed};
    return speeds;
}

void motor_drive(int Rspeed, int Lspeed) {
    //restricting speeds of motors between 255 and -90
    if (Rspeed > base_speed)
        Rspeed = base_speed;
    if (Lspeed > base_speed)
        Lspeed = base_speed;
    if (Rspeed < 0)
        Rspeed = -90.0;
    if (Lspeed < 0)
        Lspeed = -90.0;


    analogWrite(PWM_right, abs(Rspeed));
    if (Rspeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    analogWrite(PWM_left, abs(Lspeed));
    if (Lspeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
}

float distance_of_the_nearset_object_that_the_left() { //using the ultrasound
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long t = pulseIn(echo, HIGH);
    float d = t / (29.1 * 2);
    delay(250);
    return d;
}

int * IR_sensors_readings() {
    int valR = analogRead(IR_sensorR);
    int valL = analogRead(IR_sensorL);
    int values[] = {
        valR,
        valL
    };

    if (DEBUG) {
        Serial.print("R:");
        Serial.print(valR);
        Serial.print(' ');
        Serial.print("L:");
        Serial.print(valL);
        Serial.println();
    }
    return values;
}

void printlnine() {
    if (DEBUG) {
        Serial.println();
    }
}