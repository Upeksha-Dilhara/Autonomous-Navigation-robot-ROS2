#include <ros.h>
#include <std_msgs/String.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

ros::NodeHandle nh;

int left_speed = 0;
int right_speed = 0;
void velocityCallback(const std_msgs::String& msg){
  char* token = strtok(msg.data, ",");  
  left_speed = atoi(token);  
  token = strtok(NULL, ",");
  right_speed = atoi(token);
  setMotorSpeeds(left_speed, right_speed);
  delay(100);
  setMotorSpeeds(0, 0);
  delay(10);
}
ros::Subscriber <std_msgs::String> velSub("/cmd_vel", velocityCallback); 

SoftwareSerial serial(10,11);	
//RoboClaw Motor Controller
RoboClaw roboclaw(&serial,10000);  
#define address 0x80


void setup(){
  // Serial.begin(57600);
  roboclaw.begin(38400);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(velSub);
}

void loop(){
  nh.spinOnce();
}

void setMotorSpeeds(int left_speed, int right_speed){
  if (left_speed>=0) roboclaw.ForwardM1(address, left_speed);
  else roboclaw.BackwardM1(address, abs(left_speed));

  if (right_speed>=0) roboclaw.ForwardM2(address, right_speed);
  else roboclaw.BackwardM2(address, abs(right_speed));
}