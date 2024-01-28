#include <QTRSensors.h>
#include <Arduino.h>

// QTR
QTRSensors qtr;
const uint8_t SensorCount = 8; //uint8_t stands for "unsigned integer with 8 bits.
uint16_t sensorValues[SensorCount];
const uint8_t pins[] = {3,4,5,12,7,8,A3,A4};
//QTR calibration test values
const float maximum_calibration_required = 990;
const float minimum_calibration_required = 10;



//ultrasound
#define trig 13
#define echo A2

//tcrt5000 IR_sensor
#define IR_sensorR A0
#define IR_sensorL A1

//motors
#define RMF 6
#define RMB 11
#define LMF 9
#define LMB 10

//speeds
const int base_speed = 255;

int pos;

int button = 3; //to be pressed to find set point

float p;
float i;
float d;
float lp = 0;
float error;
float correction;
float sp;

float Kp = 0.15; // dummy
float Ki = 0; //dummy
float Kd = 1; //(Kp-1)*10

void pid_calc();
void motor_drive(int, int);
int distance();
void calibration();
void calibration_check();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);


    //configure the sensors QTR
    qtr.setTypeRC();
    qtr.setSensorPins(pins, SensorCount);
    calibration();

    // ultrasound
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    //motors
    pinMode(RMF, OUTPUT);
    pinMode(RMB, OUTPUT);
    pinMode(LMF, OUTPUT);
    pinMode(LMB, OUTPUT);


    //button
    pinMode(A5, INPUT_PULLUP);
    while (digitalRead(A5) != 0) {}
}

void loop() {
    pid_calc();

}
void calibration() {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

void pid_calc() {
    int sp = qtr.readLineBlack(sensorValues);

    error = -3500 + sp;

    p = error;
    i += p;
    d = p - lp;
    lp = error;

    correction = int(Kp * p + Ki * i + Kd * d);

    int Rspeed = base_speed - correction;
    int Lspeed = base_speed + correction;

    //restricting speeds of motors between 255 and -255

    if (Rspeed > base_speed)
        Rspeed = base_speed;

    if (Lspeed > base_speed)
        Lspeed = base_speed;

    if (Rspeed < -90)
        Rspeed = -90;

    if (Lspeed < -90)
        Lspeed = -90;

    motor_drive(Rspeed, Lspeed);
}

void motor_drive(int vr, int vl) {

    if (vr > 0) {
        analogWrite(RMF, abs(vr));
        analogWrite(RMB, 0);
    } else {
        analogWrite(RMF, 0);
        analogWrite(RMB, abs(vr));
    }

    if (vl > 0) {
        analogWrite(LMF, abs(vl));
        analogWrite(LMB, 0);
    } else {
        analogWrite(LMF, 0);
        analogWrite(LMB, abs(vl));

    }
}

int distance() {
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned int t = pulseIn(echo, HIGH);
    unsigned int d = t / (29.1 * 2);
    delay(250);
    return d;
}


void calibration_check() {
    float calibrationOn_minimum;
    float calibrationOn_maximum;
    float calibrationOn_minimum_total = 0;
    float calibrationOn_maximum_total = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
        calibrationOn_minimum = qtr.calibrationOn.minimum[i];
        calibrationOn_minimum_total += calibrationOn_minimum;

            Serial.print(calibrationOn_minimum);
            Serial.print(' ');
    }

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < SensorCount; i++) {
        calibrationOn_maximum = qtr.calibrationOn.maximum[i];
        calibrationOn_maximum_total += calibrationOn_maximum;

        
            Serial.print(calibrationOn_maximum);
            Serial.print(' ');
      
    }


    //checking the calibration
    if (calibrationOn_maximum_total / SensorCount >= maximum_calibration_required && calibrationOn_minimum_total / SensorCount <= minimum_calibration_required) {
        while (true) {

          
                Serial.print("BAD calibration!!");
                Serial.println();
                Serial.print(calibrationOn_maximum_total / SensorCount);
                Serial.print(' ');
                Serial.print(calibrationOn_minimum_total / SensorCount);
                Serial.println();

            delay(500);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

}