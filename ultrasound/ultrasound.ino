#define trig 10
#define echo 9
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  unsigned int t=pulseIn(echo,HIGH);
  unsigned int d=t/(29.1*2);
  Serial.println(d);
  delay(250);


}
