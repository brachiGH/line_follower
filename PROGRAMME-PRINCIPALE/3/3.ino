#include <QTRSensors.h>
#include <Arduino.h>



//tcrt5000 IR_sensor
#define IR_sensorR A0
#define IR_sensorL A1
// QTR
QTRSensors qtr;
const uint8_t SensorCount = 10; //uint8_t stands for "unsigned integer with 8 bits.
uint16_t sensorValues[SensorCount];
const uint8_t pins[] = {IR_sensorL,3,4,5,12,7,8,A3,A4,IR_sensorR};
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

int pid_calc();
void motor_drive(int, int);
int distance();
void calibration();
int * IR_sensors_readings();
void turnleft();
void turnright();

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
    int error = pid_calc();
    Serial.print("error:");
    Serial.println(error);

    int RI_R = analogRead(IR_sensorR);
    int RI_L = analogRead(IR_sensorL);
    Serial.print("R:");
    Serial.println(RI_R);
    Serial.print("L:");
    Serial.println(RI_L);
    delay(500);







    // if (error<300 && error>-300 && RI_R<250 && RI_L>750) {
    //     turnleft();
    //     while (RI_R<250){}
    // }

    
    // if (error<300 && error>-300 && RI_L<250 && RI_R>750) {
    //     turnright();
    //     while (RI_L<250){}
    // }
}
void calibration() {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    for (uint16_t i = 0; i < 150; i++) {
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
    return error;
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
        analogWrite(RMF, 255);
        analogWrite(RMB, 0);
        analogWrite(LMF, 0);
        analogWrite(LMB, 255);

}

void turnright() {
        analogWrite(RMF, 0);
        analogWrite(RMB, 255);
        analogWrite(LMF, 255);
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
