//
//
//

//Arduino Packages
#include <Arduino.h>
//ROS Serial Communication Packages
#include <ros.h>
#include <std_msgs/String.h>

//instantiate a node handle
ros::NodeHandle nh;

//Publisher initialization
std_msgs::String str_msg;
std_msgs::String str_msg;

//Subscriber initialization
void messageCb(const std_msgs::String& msg){
  //Do something with the message
}
  //First argument is the name of the topic, second argument is the callback function
ros::Subscriber<std_msgs::String> sub(/*publisher name*/, &messageCb);

void setup() {
  //Initialize the node handle
    nh.initNode();
  //Initialize the publisher
    nh.advertise(chatter);
  //Initialize the subscriber
    nh.subscribe(sub);
}

void loop() {

    //Publish the message
    //str_msg.data = hello;
    //chatter.pusblish(&str_msg);

    //Cause callbacks to be called
    // nh.spinOnce();
    // delay(1000);
}