#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String msg;
std_msgs::UInt8 val;
ros::Publisher ard_chatter("ard_chatter", &msg);

const int DIR = 2;
const int PWM = 3;
const int SLP = 4;
const int POT = A0;
const int high_lim = 850;
const int low_lim = 550;

int duty_cycle = 40; //percentage
int period = 1000; //us
int pot_val;
int cmd;

void messageCb(const std_msgs::UInt8& chat)
{
  cmd = chat.data;
}

ros::Subscriber<std_msgs::UInt8> sub("ros_chatter", &messageCb);

void setup()
{
  nh.initNode();
  nh.advertise(ard_chatter);
  nh.subscribe(sub);

  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(POT, INPUT);
  digitalWrite(SLP, HIGH);
}

int pot_read()
{
  pot_val = analogRead(POT);
}

void drop()
{
  digitalWrite(DIR, HIGH);
  analogWrite(PWM, (int)(duty_cycle*255/100));
  msg.data = "Dropping...";
  ard_chatter.publish(&msg);
}

void lift()
{
  digitalWrite(DIR, LOW);
  analogWrite(PWM, (int)(duty_cycle*255/100));
  msg.data = "Lifting...";
  ard_chatter.publish(&msg);
}

void loop()
{
  pot_read();

  if (pot_val < low_lim and cmd == 1)
  {
    msg.data = "Dropping done!";
    ard_chatter.publish(&msg);
    cmd = 0;
  }
  else if (pot_val > high_lim and cmd == 2)
  {
    msg.data = "Lifting done!";
    ard_chatter.publish(&msg);
    cmd = 0;
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
    msg.data = "Stop!";
    ard_chatter.publish(&msg); 
  }
   
  nh.spinOnce();
}
