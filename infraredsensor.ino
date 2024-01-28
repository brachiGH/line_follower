#define sensor A2
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(sensor,INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  int val=analogRead(sensor);
  Serial.println(val);
  delay(50);

}
