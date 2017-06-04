const int fore_sw = 7;
const int rear_sw = 8;

void setup() 
{
  pinMode(fore_sw, INPUT);
  pinMode(rear_sw, INPUT);
  Serial.begin(9600);
}

void loop() 
{
  if (digitalRead(fore_sw) == 1)
  {
    Serial.println("fore_sw HIGH");
  }
  else
  {
    Serial.println("fore_sw LOW");
  }

  if (digitalRead(rear_sw) == 1)
  {
    Serial.println("rear_sw HIGH");
  }
  else
  {
    Serial.println("rear_sw LOW");
  }

  delay(500);
}
