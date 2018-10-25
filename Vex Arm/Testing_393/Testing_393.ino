#include <vexArm.h>
#include <vexMotor.h>

vexArm one;
void setup() {
  Serial.begin(9600);
  one.attachPin(2);
  one.writeMotorPublic(240);
  delay(800);
  one.writeMotorPublic(0);
}

void loop() {
  Serial.println(analogRead(0));  
}
