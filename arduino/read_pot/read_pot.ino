const int pot = A0; // potentiometer

void setup()
{
  pinMode(pot, INPUT);
  Serial.begin(9600);
}

int pot_val = 0;

void loop() 
{
  pot_val = analogRead(pot);
  Serial.println(pot_val);
  delay(100);
}
