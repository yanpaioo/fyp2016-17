#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield motor;

void setup()
{
  Serial.begin(9600);
  Serial.println("Dual VNH5019 Motor Shield");
  motor.init();
}

int data;
void loop() 
{
  //motor.setM1Speed(0);
  
  if (Serial.available() > 0)
  {
    data = int(Serial.read()) - 48;
    Serial.println(data);

    if (data == 0)
    {
      Serial.println("Halted");
      motor.setM1Speed(-300);
    }
    
    else if (data == 1)
    {
      Serial.println("Ascending");
      motor.setM1Speed(0);
    }

    else if (data == 2)
    {
      Serial.println("Descending");
      motor.setM1Speed(300);
    }
  }  
  
  delay(200);
  
}
