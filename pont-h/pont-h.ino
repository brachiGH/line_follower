#define RMF 6
#define RMB 11
#define LMF 9
#define LMB 10

#define base_speed 255


void breakmotors(){
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

void setup() {
  // put your setup code here, to run once:
  pinMode(RMF,OUTPUT);
  pinMode(RMB,OUTPUT);
  pinMode(LMF,OUTPUT);
  pinMode(LMB,OUTPUT);

}

void loop() {

        motorsforword();
        delay(1000);
        breakmotors();
        delay(1000);

}
