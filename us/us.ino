#include <QTRSensors.h>
#include <Arduino.h>

//tcrt5000 IR_sensor
#define IR_sensorR A0
#define IR_sensorL A1
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


//motors
#define RMF 6
#define RMB 11
#define LMF 9
#define LMB 10

//speeds
const int base_speed = 255;

int turn_counter = 0;
const uint8_t Truns_R[] = {2,3,5};
const uint8_t Truns_L[] = {1,4};

int button = 3; //to be pressed to find set point

float p;
float i;
float d;
float lp = 0;
float error;
float correction;
float sp;

float Kp = 0.17; 
float Ki = 0; 
float Kd = 1;

int pid_calc();
void motor_drive(int, int);
int distance();
void calibration();
void turnleft();
void turnright();
void motorsforword();
void breakmotors();


unsigned long previousMillis = 0;
const long interval = 1500;
int hasturned = 0;

// Define threshold values for IR sensors
const int IR_THRESHOLD = 500;

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

    //configuration the pins of the IR sensors
    pinMode(IR_sensorL, INPUT);
    pinMode(IR_sensorR, INPUT);

    //button
    pinMode(A5, INPUT_PULLUP);
    while (digitalRead(A5) != 0) {}
}

unsigned long previousMillis = 0;
const long interval = 1500;

void loop() {
  int d = distance();

  if (5<d) {
void turnright();
  } else if (7>d) {
void turnleft();

  } else {
void motorsforword();
  }
}

void calibration() {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    for (uint16_t i = 0; i < 300; i++) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

int pid_calc() {
    int sp = qtr.readLineBlack(sensorValues);

    error = 3500 - sp;

    p = error;
    i += p;
    d = p - lp;
    lp = error;

    correction = int(Kp * p + Ki * i + Kd * d);

    int Rspeed = base_speed + correction;
    int Lspeed = base_speed - correction;

    motor_drive(Rspeed, Lspeed);
    return sp;
}

void motor_drive(int Rspeed, int Lspeed) {
    //restricting speeds of motors between 255 and -255
    if (Rspeed > base_speed)
        Rspeed = base_speed;

    if (Lspeed > base_speed)
        Lspeed = base_speed;

    if (Rspeed < -90)
        Rspeed = -90;

    if (Lspeed < -90)
        Lspeed = -90;

    if (Rspeed > 0) {
        analogWrite(RMF, abs(Rspeed));
        analogWrite(RMB, 0);
    } else {
        analogWrite(RMF, 0);
        analogWrite(RMB, abs(Rspeed));
    }

    if (Lspeed > 0) {
        analogWrite(LMF, abs(Lspeed));
        analogWrite(LMB, 0);
    } else {
        analogWrite(LMF, 0);
        analogWrite(LMB, abs(Lspeed));

    }
}

void turnleft() {
   analogWrite(RMF, base_speed);
   analogWrite(RMB, 0);
   analogWrite(LMF, 0);
   analogWrite(LMB, base_speed);
}

void turnright() {
   analogWrite(RMF, 0);
   analogWrite(RMB, base_speed);
   analogWrite(LMF, base_speed);
   analogWrite(LMB, 0);
}

void breakmotors() {
   analogWrite(RMF, 0);
   analogWrite(RMB, 0);
   analogWrite(LMF, 0);
   analogWrite(LMB, 0);
}
void motorsforword() {
   analogWrite(RMF, base_speed);
   analogWrite(RMB, 0);
   analogWrite(LMF, base_speed);
   analogWrite(LMB, 0);
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