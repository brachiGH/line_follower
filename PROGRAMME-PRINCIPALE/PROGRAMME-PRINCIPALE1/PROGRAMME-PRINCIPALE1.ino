#include <QTRSensors.h>

QTRSensorsRC qtr((char[]) {3,4,5,12,7,8,A3,A4},8);
unsigned int sensors[8];
//ultrasound
#define trig 10
#define echo 9

//motors
#define rmf 6
#define rmb 11
#define lmf 9 
#define lmb 10

//speeds
const int base_speed = 255;

int pos;

int button = 3; //to be pressed to find set point

float p;
float i;
float d;
float lp=0;
float error;
float correction;
float sp;

float Kp = 0.15; // dummy
float Ki = 0; //dummy
float Kd = 1; //(Kp-1)*10

void pid_calc();
void motor_drive(int , int );
int distance();
void calibration();

void setup()
{ calibration();
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  Serial.begin(9600);
 
  //motors
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);

 //button
    pinMode(A5, INPUT_PULLUP);
    while (digitalRead(A5) != 0) {}
}

void loop(){ 
  pid_calc();
  
}
void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void pid_calc()
{
  int sp = qtr.readLine(sensors);

  error = -3500+sp;
  
 
  p=error;
  i += p;
  d = p - lp;
   lp =error;
  

  correction = int(Kp*p + Ki*i + Kd*d);

  int Rspeed = base_speed - correction;
  int Lspeed = base_speed + correction;

  //restricting speeds of motors between 255 and -255
  
  if (Rspeed > base_speed) 
    Rspeed = base_speed;
    
  if (Lspeed > base_speed) 
    Lspeed = base_speed;
    

  if (Rspeed < 0) 
    Rspeed = -90;
    
  if (Lspeed < 0) 
    Lspeed = -90;
 
 motor_drive( Rspeed, Lspeed);  
} 

void motor_drive(int vr, int vl){
  
  if(vr>0)
  { 
  analogWrite(rmf, abs(vr));   
  analogWrite(rmb,0);
  }
  else 
  { 
  analogWrite(rmf, 0);   
  analogWrite(rmb,abs(vr));
  }
  
 
  if(vl>0)
  { 
  analogWrite(lmf, abs(vl));   
  analogWrite(lmb,0);
  }
  else 
  { 
  analogWrite(lmf, 0);   
  analogWrite(lmb,abs(vl));
   
  }
}

 int distance() {
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    unsigned int t=pulseIn(echo,HIGH);
    unsigned int d=t/(29.1*2);
    delay(250);
    return d;
}
