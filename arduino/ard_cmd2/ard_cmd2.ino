const int DIR = 2;
const int PWM = 3;
const int SLP = 4;
const int POT = A0;
const int high_lim = 850;
const int low_lim = 550;
int i;

int duty_cycle = 40; //percentage
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
  Serial.println(i);
  
}

void drop()
{
  Serial.println("Dropping...");
  digitalWrite(DIR, HIGH);
  analogWrite(PWM, (int)(duty_cycle*255/100));
}

void lift()
{

  Serial.println("Lifting...");
  digitalWrite(DIR, LOW);
  analogWrite(PWM, (int)(duty_cycle*255/100));
}

void loop() {
   pot_read();
   
   if (Serial.available() > 0) 
   {
    cmd = int(Serial.read()) - 48;
   }

   
   if (pot_val < low_lim and cmd == 1)
   {
    Serial.println("Dropping done!");
    cmd = 0;
   }
   else if (pot_val > high_lim and cmd == 2)
   {
    Serial.println("Lifting done!");
    cmd = 0;
    i++;
   }

   
   if (cmd == 1)
   {
    drop();
   }
   else if (cmd == 2)
   {
    lift();
   }
   else
   {
    analogWrite(PWM, 0);
   }

}
