const int DIR = 4;
const int PWM = 5;
const int SLP = 6;
const int POT = A0;

int duty_cycle = 30; //percentage
int period = 1000; //us
int pot_val;
int cmd;


void setup() {
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(POT, INPUT);
  digitalWrite(SLP, HIGH);
  Serial.begin(9600);
  Serial.println("Motor initialized");
}

int pot_read()
{
  pot_val = analogRead(POT);
  Serial.println(pot_val);
  
}

void lift()
{
  Serial.println("Dropping...");
  digitalWrite(DIR, HIGH);
  analogWrite(PWM, (int)(duty_cycle*255/100));
}

void drop()
{

  Serial.println("Lifting...");
  digitalWrite(DIR, LOW);
  analogWrite(PWM, (int)(duty_cycle*255/100));
}

void loop() {
   //pot_read();
   if (Serial.available() > 0) 
   {
    cmd = int(Serial.read()) - 48;
   }

   if (cmd == 1)
   {
    lift();
   }
   else if (cmd == 2)
   {
    drop();
   }
   else
   {
      analogWrite(PWM, 0);
      Serial.println("Stopped...");
   }

}
