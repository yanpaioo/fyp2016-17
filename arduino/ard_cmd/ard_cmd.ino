const int DIR = 2;
const int PWM = 3;
const int SLP = 4;
const int POT = A0;

int duty_cycle = 100; //percentage
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
  digitalWrite(PWM, HIGH);
  delayMicroseconds((period*duty_cycle)/100);
  digitalWrite(PWM, LOW);
  delayMicroseconds((period*(100-duty_cycle))/100);
  Serial.println(pot_val);
}

void drop()
{

  Serial.println("Lifting...");
  digitalWrite(DIR, LOW);
  digitalWrite(PWM, HIGH);
  delayMicroseconds((period*duty_cycle)/100);
  digitalWrite(PWM, LOW);
  delayMicroseconds((period*(100-duty_cycle))/100);
  pot_read();
  Serial.println(pot_val);
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

}
