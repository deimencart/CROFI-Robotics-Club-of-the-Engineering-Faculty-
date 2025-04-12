//Proggrama para mover cuatro motores con Drivers Roboclaw
//Lee los mesajes tipo twist del Paquete teleop_twist_keyboard de ROS
// Versión de ROS Melodic


#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;

geometry_msgs::Twist msg;
std_msgs::String direction_msg;

float move1;
float move2;

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial_1(10,11);	//Primer RoboClaw con los pines dentro de los parentesis
SoftwareSerial serial_2(7,6);

RoboClaw roboclaw_1(&serial_1,10000);
RoboClaw roboclaw_2(&serial_2,10000);

#define address 0x80

void callback(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.angular.z;
  if (move1 > 0 && move2 == 0)
  {
    direction_msg.data = "FOWARD";
    front();
  }
  else if (move1 > 0 && move2 > 0 )
  {
    direction_msg.data = "LEFT";

    left();
  }
  else if (move1 > 0 && move2 < 0 )
  {
    direction_msg.data = "RIGHT";
    right();
  }
  else if (move1 < 0)
  {
    direction_msg.data = "BACKWARDS";
    back();
  }
  else
  {
    direction_msg.data = "DIE";
    die();
  }
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", callback); //Lineas para hacer un subscriptor y publicador en ROS al mismo tiepmo
ros::Publisher direction_publisher("direction_given", &direction_msg);

void setup() {
  Serial.begin(115200); //Frecuencia de comunicaición de la computadora con la computadora
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  //Open roboclaw serial ports
  roboclaw.begin(38400);

//Esto para mandar el mensaje que se está publicando
nh.initNode();
nh.subscribe(sub);
nh.advertise(direction_publisher);
}

void loop() {
  direction_publisher.publish(&direction_msg);
  nh.spinOnce();
  delay(100);
}
//Velocidad de los roboclaw va de 0-127
void front(int x) {
  roboclaw_1.ForwardM1(address,x);
  roboclaw_1.ForwardM2(address,x);
  roboclaw_2.ForwardM1(address,x);
  roboclaw_2.ForwardM2(address,x);
}

void back(int x) {
  roboclaw_1.BackwardM1(address,x);
  roboclaw_1.BackwardM2(address,x);
  roboclaw_2.BackwardM1(address,x);
  roboclaw_2.BackwardM2(address,x);
}
void left(int x) {
  roboclaw_1.BackwardM1(address,x);
  roboclaw_1.ForwardM2(address,x);

  roboclaw_2.ForwardM1(address,x);
  roboclaw_2.BackwardM2(address,x);
}

void right(int x) {
  roboclaw_1.BackwardM2(address,x);
  roboclaw_1.ForwardM1(address,x);
  roboclaw_2.ForwardM2(address,x);
  roboclaw_2.BackwardM1(address,x);
}

void die() {
  roboclaw_1.ForwardM1(address,0);
  roboclaw_1.ForwardM2(address,0);
  roboclaw_2.ForwardM1(address,0);
  roboclaw_2.ForwardM2(address,0);
}
