//#if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"
//#else
//#include <WProgram.h>
//#endif
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

geometry_msgs::Twist msg;
std_msgs::String direction_msg;

float move1;
float move2;

#define LED_F 13
#define LED_B 12
#define LED_L 11
#define LED_R 10




void callback(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.angular.z;
  if (move1 > 0 && move2 == 0)
  {
    direction_msg.data = "FOWARD";
    digitalWrite(LED_F,HIGH);
    digitalWrite(LED_B,LOW);
    digitalWrite(LED_L,LOW);
    digitalWrite(LED_R,LOW);

    //front();
  }
  else if (move1 > 0 && move2 > 0 )
  {
    direction_msg.data = "LEFT";
    digitalWrite(LED_F,LOW);
    digitalWrite(LED_B,LOW);
    digitalWrite(LED_L,HIGH);
    digitalWrite(LED_R,LOW);

    //left();
  }
  else if (move1 > 0 && move2 < 0 )
  {
    direction_msg.data = "RIGHT";
    digitalWrite(LED_F,LOW);
    digitalWrite(LED_B,LOW);
    digitalWrite(LED_L,LOW);
    digitalWrite(LED_R,HIGH);
    //right();
  }
  else if (move1 < 0)
  {
    direction_msg.data = "BACKWARDS";
    digitalWrite(LED_F,LOW);
    digitalWrite(LED_B,HIGH);
    digitalWrite(LED_L,LOW);
    digitalWrite(LED_R,LOW);
    //back();
  }
  else
  {
    direction_msg.data = "DIE";
    digitalWrite(LED_F,LOW);
    digitalWrite(LED_B,LOW);
    digitalWrite(LED_L,LOW);
    digitalWrite(LED_R,LOW);
    //die();
  }
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", callback);
ros::Publisher direction_publisher("direction_given", &direction_msg);


void setup() {

Serial.begin(115200);
nh.getHardware()->setBaud(115200);
nh.initNode();
pinMode(LED_F, OUTPUT);
pinMode(LED_B, OUTPUT);
pinMode(LED_L, OUTPUT);
pinMode(LED_R, OUTPUT);


nh.initNode();
nh.subscribe(sub);
nh.advertise(direction_publisher);



}

void loop() {
direction_publisher.publish(&direction_msg);  
nh.spinOnce();
delay(100);
}
