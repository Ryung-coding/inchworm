//1. arduino upload 
//2. roscore 
//3. rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
//4. rostopic echo /angle_deg

//Encoder pin map
//pin 4  VCC  (line black )-> vcc(5V)
//pin 6  GND  (line white )-> Gnd
//pin 8  A+   (line gray  )-> A_plus pin
//pin 9  B+   (line purple)-> B_plus pin
//pin 10 A-   (line blue  )-> Gnd
//pin 11 B-   (line green )-> Gnd

#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
std_msgs::Float64 angle_deg;
ros::Publisher p("angle_deg", &angle_deg);

#define A_plus 3  
#define B_plus 2 
#define gear_ratio 3.076923077 // 40(big_gear) / 13(small_gear) 
#define counter_to_deg 0.703125 // 360(deg) / 512(PPR)

int counter = 0; int counter_past = 0;        
int state_rotation; int state_rotation_past;  
  
float angle = 0.0;

void Read_encoder()
{
  state_rotation = digitalRead(A_plus);
  if (state_rotation != state_rotation_past && state_rotation == 1) counter = digitalRead(B_plus) != state_rotation ? counter+1 : counter-1;
  state_rotation_past = state_rotation; 
  counter_past = counter;
}


void setup() 
{
  pinMode(A_plus, INPUT_PULLUP);
  pinMode(B_plus, INPUT_PULLUP);
  state_rotation_past = digitalRead(A_plus);

  nh.initNode();
  nh.advertise(p);
}

void loop() 
{
  Read_encoder();
  angle = (counter/gear_ratio)*counter_to_deg;

  angle_deg.data = angle;
  p.publish( &angle_deg );
  nh.spinOnce();
  
}
