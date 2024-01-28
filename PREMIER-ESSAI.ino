
#include <QTRSensors.h>
QTRSensorsRC qtr((char[]) {A0, A1, A2,A3,A4,A5,3,5},8);
unsigned int sensorValues[8];

void setup() {
  Serial.begin(9600);
  int i;
  for (i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(20);
  
  }
}

void loop() {
  unsigned int sensors[8];
  int position = qtr.readLine(sensors);
  for (unsigned char i = 0; i <8; i++)
 {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(); 
  Serial.println(position); 
  delay(250);


}
