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
float pid_calc(uint16_t);
int * IR_sensors_readings();
float distance();
void motor_drive(int, int);
void printline();

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


// Definitions Arduino pins connected to input H Bridge (motors control)
// const uint8_t PWM_right = 9;
// const uint8_t PWM_left = 10;
const uint8_t MLF = 9;  //motor left clockwise (backwards)
const uint8_t MLB = 10;  //motor left counter-clockwise (front)
const uint8_t MRB = 11; //11; //motor right clockwise
const uint8_t MRF = 6; //2;  //motor right counter-clockwise

//PID configuration
const int base_speed = 150;
float Rspeed, Lspeed;
int pos;
int button = 3; // to be pressed to find set point

#define trageted_qtr_value 3500
float proportional;
float derivative;
float error, prevError = 0.0, integral = 0.0;
float correction;

float Kp = 0.6;
float Ki = 0.0;
float Kd = 0.8;

void setup() {
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins(pins, SensorCount);
    qtr.setEmitterPin(2); //    "IR LEDs" PIN ON THE QRT(the pin between the vcc and pin number 1)!!


    
    // print the calibration minimum values measured when emitters were on
    Serial.begin(9600);


    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
    // = ~25 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 150; i++) {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

    // //configuration the pins of the ultrasound
    // pinMode(trig, OUTPUT);
    // pinMode(echo, INPUT);

    // //configuration the pins of the IR sensors
    // pinMode(IR_sensorL, INPUT);
    // pinMode(IR_sensorR, INPUT);

    // Set the output pins for pont-H
    pinMode(MLB, OUTPUT);
    pinMode(MLF, OUTPUT);
    pinMode(MRB, OUTPUT);
    pinMode(MRF, OUTPUT);


    delay(1000);
}

void loop() {
    // read calibrated QTR sensor values and obtain a measure of the line position
    // from 0 to 7000 (for a white line, use readLineWhite() instead)
    int position = qtr.readLine(sensorValues);

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
    float correction = pid_calc(position);
      motor_drive(correction);

    // //distance from the ultrasound to wall using ultrasound
    // float distance = distance_read_by_ultrasound();
    // if (DEBUG) {
    //     Serial.print("distance:");
    //     Serial.print(distance);
    // }
    // printline();

    // //detecting the wall using ultrasound
    // int * sensors_values = IR_sensors_readings();
    // int RI_R = *sensors_values;
    // int RI_L = *(sensors_values + 1);
    // if (DEBUG) {
    //     Serial.print("(IR)R:");
    //     Serial.print(RI_R);
    //     Serial.print("  L:");
    //     Serial.print(RI_L);
    // }
    // printline();

    // // the first obstacle
    // if (RI_L>900 && RI_R>900) {
    //     motor_drive(90, -90);
    // }

    printline();
    printline();
}

float pid_calc(uint16_t qtr_position) {
    error = trageted_qtr_value - (float)qtr_position;

    proportional =  error ;
    integral +=  error;
    derivative = (error - prevError);
    prevError = error;

    correction = Kp * proportional + Ki * integral + Kd * derivative;
delay(20);
    Serial.print("correction: ");
    Serial.print(correction);
    printline();
    return correction;
}

void motor_drive(float correction) {
    
  int Rspeed = base_speed + (int)correction;
  int Lspeed = base_speed - (int)correction;


  if (Rspeed > 255) 
      Rspeed = base_speed;
      
  if (Lspeed > 255) 
      Lspeed = base_speed;
      

  if (Rspeed < 0) 
      Rspeed = 20;
      
  if (Lspeed < 0) 
      Lspeed = 20;

  if (Lspeed < 0) { //right turn
      analogWrite(MLF, 0);
      analogWrite(MLB, abs(Lspeed));
  }
  else{  
    analogWrite(MLF,Lspeed);
    analogWrite(MLB, 0);
  }

  if (Rspeed < 0) { //left
    analogWrite(MRB, abs(Rspeed));
    analogWrite(MRF, 0);
  }
  else {
    analogWrite(MRF, Rspeed);
    analogWrite(MRB, 0);
  }
}

float distance_read_by_ultrasound() { //using the ultrasound
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long t = pulseIn(echo, HIGH);
    float d = t / (29.1 * 2);
    return d;
}

int * IR_sensors_readings() {
    int valR = analogRead(IR_sensorR);
    int valL = analogRead(IR_sensorL);
    int values[] = {valR,valL};

    return values;
}

void printline() {
    if (DEBUG) {
        Serial.println();
    }
}






















unsigned long currentMillis = millis();
if(IR_L>IR_THRESHOLD || IR_R>IR_THRESHOLD) {
int lt = IR_L > IR_THRESHOLD && position > 2650 && position > 2650 && IR_R < IR_THRESHOLD;
int rt = IR_R > IR_THRESHOLD && sensorValues[2] > IR_THRESHOLD && sensorValues[3] > IR_THRESHOLD && sensorValues[4] > IR_THRESHOLD && sensorValues[5] > IR_THRESHOLD && sensorValues[6] > IR_THRESHOLD && sensorValues[7] > IR_THRESHOLD && IR_L < IR_THRESHOLD;
if ((lt || rt) && (currentMillis - previousMillis >= interval)) {
  trun_counter++;
  previousMillis = currentMillis;
  breakmotors();
  delay(2500);

  // Handle left turn
  if (trun_counter == 5 || trun_counter == 6 || trun_counter == 6 || trun_counter == 11 || trun_counter == 13) {
    turnleft();
    while (analogRead(IR_sensorR) < IR_THRESHOLD){}
    breakmotors();
    delay(2500);
  }
  // Handle right turn
  if (trun_counter == 8 || trun_counter == 9 || trun_counter == 12 || trun_counter == 15) {
    turnright();
    while (analogRead(IR_sensorL) < IR_THRESHOLD){}
    breakmotors();
    delay(2500);
  }




#define ENA 10
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define ENB 5


int i ;
int n=5;   //n : nombre des capteur infrarouge utilisé dans le robot

int IR[5]={5,4,2,7,3} ;   // les capteurs infrarouges de gauche jusqu'à droit
//int A_IR[5];                     // les valeurs analogiques des capteurs
int D_IR[5];                     // les valeurs digitales des capteurs aprés le calibrage

// les constantes necessaires pour le controleur pid
int kp=20;   //30
int kd=4;
int ki=0;

int erreur=0;
int last_erreur =0 ;
int sum_erreur=0;

int left_speed,right_speed;
int vitesseMoteur=0;
int base_speed=50;
int maxspeed=245;
int minspeed=0;
//int IR_calibrage_value=500;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  for (i=0;i<n;i++){
    pinMode( IR[i],INPUT);
  }

 
  Serial.begin(9600);

  digitalWrite(ENA,LOW);  //intialiser le robot à ropot
  digitalWrite(ENB,LOW);
 

 
}

void loop() {
  // put your main code here, to run repeatedly: 
  PID();
  

}
void PID(){
 
   for (i=0;i<5;i++){
    D_IR[i]=2*digitalRead(IR[i])-1;
  }
  
   erreur=4*D_IR[0]+2*D_IR[1]-2*D_IR[3]-3*D_IR[4];       //4 2 2 4
   vitesseMoteur= kp*erreur + kd*(erreur-last_erreur) + ki*sum_erreur;

    
   motorDriver(vitesseMoteur);
   
   last_erreur=erreur;
   sum_erreur+=erreur;
   
 
}


void motorDriver(int cor_vitesse){
  left_speed = base_speed - cor_vitesse  ;
  right_speed = base_speed   + cor_vitesse  ;
 
  if (left_speed>maxspeed){
    left_speed=maxspeed;
  }
  if (left_speed<minspeed){
    left_speed=minspeed;
  }
  if (right_speed>maxspeed){
    right_speed=maxspeed;
  }
  if (right_speed<minspeed){
    right_speed=minspeed;
  }

  //left motor 
  if (left_speed>=0) {
    
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,left_speed);
    } 
  if (left_speed<0) { 
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(ENA,left_speed); 
    } 
  

  //right motor 
  if (right_speed>0) {
    
    digitalWrite(IN4,HIGH);
    digitalWrite(IN3,LOW);
    analogWrite(ENB,right_speed);  
    }
 if (right_speed<0){
   
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,HIGH);
  analogWrite(ENB,right_speed);
  }
 

 
}