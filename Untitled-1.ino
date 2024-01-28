#include <QTRSensors.h>
#include <Arduino.h>

#define RMF 6
#define RMB 11
#define LMF 9
#define LMB 10
//speeds
const int base_speed = 255;

int P;
int I;
int D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

int button_calibration = A3;
int button_start = 2;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3,4,5,12,7,8,A3,A4}, SensorCount);
  qtr.setEmitterPin(7);


    //motors
    pinMode(RMF, OUTPUT);
    pinMode(RMB, OUTPUT);
    pinMode(LMF, OUTPUT);
    pinMode(LMB, OUTPUT);

  pinMode(button_calibration, INPUT);
  pinMode(button_start, INPUT); 


  while(digitalRead(button_calibration) == LOW) {}
  //10 seconds
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }

  while(digitalRead(button_start) == LOW) {}
}

void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  int motorSpeedA = base_speed + motorSpeedChange;
  int motorSpeedB = base_speed - motorSpeedChange;

  if (motorSpeedA > base_speed) {
    motorSpeedA = base_speed;
  }
  if (motorSpeedB > base_speed) {
    motorSpeedB = base_speed;
  }
  if (motorSpeedA < -90) {
    motorSpeedA = -90;
  }
  if (motorSpeedB < -90) {
    motorSpeedB = -90;
  }
  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
    if (speedA > 0) {
        analogWrite(RMF, abs(speedA));
        analogWrite(RMB, 0);
    } else {
        analogWrite(RMF, 0);
        analogWrite(RMB, abs(speedA));
    }

    if (speedB > 0) {
        analogWrite(LMF, abs(speedB));
        analogWrite(LMB, 0);
    } else {
        analogWrite(LMF, 0);
        analogWrite(LMB, abs(speedB));

    }
}
