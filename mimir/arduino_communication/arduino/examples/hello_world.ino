//
//
//

#include <Arduino.h>

//ROS Serial Communication Packages
#include <ros.h>
#include <std_msgs/String.h>

//instantiate a node handle
ros::NodeHandle nh;

//Publisher initialization
std:msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msp);

char hello[13] = "hello world!";

void setup() {
  //Initialize the node handle
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  //Publish the message
  str_msg.data = hello;
  chatter.publish(&str_msg);
  //Cause all callbacks to be called
  nh.spinOnce();
  delay(1000);
}