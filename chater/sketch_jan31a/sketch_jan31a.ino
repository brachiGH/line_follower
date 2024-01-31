#include <QTRSensors.h>
QTRSensors qtr;

#define trig 4
#define echo 3
#define for1 9
#define back1 6
#define for2 10
#define back2 11
#define jnabL 8
#define jnabR 12
# define MaxSpeed 255 
# define minSpeed -255 
# define basespeed 200
# define turningdelay 360

int leftSpeed=0;
int rightSpeed=0;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


const float Kp = 0.1; 
const float Kd = 0.08;  

float previousError = 0;
float integral = 0;

int tright = 0;
int tleft = 0;

void setup(){
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 5, 7}, SensorCount);
  delay(500);
  pinMode(2,INPUT_PULLUP);
  pinMode(for1, OUTPUT);
  pinMode(for2, OUTPUT);
  pinMode(back1, OUTPUT);
  pinMode(back2, OUTPUT);
  pinMode(jnabL, INPUT);
  pinMode(jnabR, INPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  analogWrite(for1, 200);
  analogWrite(back1, 0);
  analogWrite(for2, 0);
  analogWrite(back2, 200);
  for (uint16_t i = 0; i < 300; i++){qtr.calibrate();}
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  analogWrite(for1, 0);
  analogWrite(back1, 0);
  analogWrite(for2, 0);
  analogWrite(back2, 0);

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  delay(300);
  while(digitalRead(2)){};
  while(digitalRead(jnabL)){
    forward();
  }
  delay(100);
}


void loop()
{
  if (tright >= 500 && tleft >= 500){
    
  }

  pid();

  if (tleft > 25){
      float cm3 = sound();
      if (cm3 <=14.0){
      delay(30);
      while(digitalRead(jnabL)==0 || digitalRead(jnabL)==0){
      cm3 = sound();
      soundalgo(cm3);
      }
      tleft = 100;
      tright = 100;}
      if (tleft == 100 && tright == 100){
      while(digitalRead(jnabL)==1 || digitalRead(jnabL)==1){
    pid2();}
    tright = 500;
    tleft = 500;}
  }
  if (tright >= 500 && tleft >= 500){
    forward();
    delay(300);
    /*for (int i = 0 ; i < 190 ; i++){
      pid2();
    }*/
    stop();
    delay(100000);
  }
  if (digitalRead(jnabR)==1){
    right();
    if (tleft != 1){delay(turningdelay / 2);}
    delay(turningdelay / 2);
    while(digitalRead(jnabL)==1){pid();}
    delay(100);
    tleft+=1;
    if (tleft > 5 && tleft < 25){
      right();
      delay(150);
      tleft = 25;
    }
  }
  if (digitalRead(jnabL)==1 && tleft >= 2){

    if(tright == 0  || tright == 1  || tright == 3 || tright == 4){
      left();
      delay(turningdelay);
    }
    if(tright == 3 || tright == 4){
      left();
      delay(turningdelay);
    }
    if(tright == 2){
      forward();
      delay(200);
    }  
    while(digitalRead(jnabL)==1){pid();}
      delay(100);
      tright+=1;
  }

}

void adjust(int left ,int right){

  if (left <=0 ){
      analogWrite(for1, 0);
      analogWrite(back1, abs(left));
    }
  else {
      analogWrite(for1, abs(left));
      analogWrite(back1, 0);
    }
  if (right >= 0){
      analogWrite(for2, right);
      analogWrite(back2, 0);
    }
  else {
      analogWrite(for2, 0);
      analogWrite(back2, -right);
    }
}
void adjust2(int left ,int right){
  if (left <=0 ){
      left += 30;
      analogWrite(for1, 0);
      analogWrite(back1, abs(left));
    }
  else {
      left -= 30;
      analogWrite(for1, abs(left));
      analogWrite(back1, 0);
    }
  if (right >= 0){
      right -= 30;
      analogWrite(for2, right);
      analogWrite(back2, 0);
    }
  else {
      right += 30;
      analogWrite(for2, 0);
      analogWrite(back2, -right);
    }
}

void right(){
  analogWrite(for1, 250);
  analogWrite(back1, 0);
  analogWrite(for2, 0);
  analogWrite(back2, 60);
}

void left(){
  analogWrite(for1, 0);
  analogWrite(back1, 60);
  analogWrite(for2, 250);
  analogWrite(back2, 0);
}

void forward(){
  analogWrite(for1, 180);
  analogWrite(back1, 0);
  analogWrite(for2, 180);
  analogWrite(back2, 0);
}

void stop(){
  analogWrite(for1, 0);
  analogWrite(back1, 0);
  analogWrite(for2, 0);
  analogWrite(back2, 0);
  delay(10000);
}

void pid(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;
  double proportional = Kp * error;
  double derivative = Kd * (error - previousError);

  double output = proportional + derivative;

  int leftSpeed = basespeed + output;
  int rightSpeed = basespeed - output;

  previousError = error;
    if(leftSpeed >MaxSpeed) leftSpeed=MaxSpeed;
    if(rightSpeed >MaxSpeed) rightSpeed=MaxSpeed;
    if(leftSpeed <=minSpeed) leftSpeed=minSpeed;
    if(rightSpeed <=minSpeed) rightSpeed=minSpeed;
  adjust(leftSpeed,rightSpeed);
}

void pid2(){
  uint16_t position = qtr.readLineWhite(sensorValues);
  int error = 3500 - position;
  double proportional = Kp * error;
  double derivative = Kd * (error - previousError);

  double output = proportional + derivative;

  int leftSpeed = basespeed + output;
  int rightSpeed = basespeed - output;

  previousError = error;
    if(leftSpeed >MaxSpeed) leftSpeed=MaxSpeed;
    if(rightSpeed >MaxSpeed) rightSpeed=MaxSpeed;
    if(leftSpeed <=minSpeed) leftSpeed=minSpeed;
    if(rightSpeed <=minSpeed) rightSpeed=minSpeed;
  adjust2(leftSpeed,rightSpeed);
}

float sound() { 
  float duration, cm; 
  digitalWrite(trig, LOW); 
  delayMicroseconds(20); 
  digitalWrite(trig, HIGH); 
  delayMicroseconds(20); 
  digitalWrite(trig, LOW); 
  duration = pulseIn(echo, HIGH); 
  cm = duration / 29 / 2;
  return cm;
}

void soundalgo(float cm3){
    if (cm3 >= 10.0){
      analogWrite(for1, 150);
      analogWrite(back1, 0);
      analogWrite(for2, 0);
      analogWrite(back2, 60);
    }
    else if (cm3 <= 5.8){
      analogWrite(for1, 0);
      analogWrite(back1, 60);
      analogWrite(for2, 150);
      analogWrite(back2, 0);
    }
    else{
      analogWrite(for1, 200);
      analogWrite(back1, 0);
      analogWrite(for2, 200);
      analogWrite(back2, 0);
      delay(50);
    }
}