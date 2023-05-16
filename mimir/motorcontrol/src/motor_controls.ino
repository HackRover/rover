/*==========================================================================
// Author : Danny Kha @ Hackrover
// Project : BTD7960 Motor Control Board driven by Arduino.
// Description : Speed and direction controlled by a potentiometer attached
// to analog input A0. One side pin of the potentiometer (either one) to
// ground; the other side pin to +5V
// Source-Code : BTS7960.ino
// Program: Control DC motors using BTS7960 H Bridge Driver.
//==========================================================================
// Connection to the BTS7960 board:
// BTS7960 Pin 1 (RPWM) to Arduino pin 5(PWM)
// BTS7960 Pin 2 (LPWM) to Arduino pin 6(PWM)
// BTS7960 Pin 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
// BTS7960 Pin 8 (GND) to Arduino GND
// BTS7960 Pin 5 (R_IS) and 6 (L_IS) not connected
*/

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

//int SENSOR_PIN = 0; // center pin of the potentiometer
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

boolean Direction_left = true;
boolean Direction_right = true;

// Set linear velocity and PWM variable values for each wheel
// double velLeftWheel = 0;
// double velRightWheel = 0;
// double pwmLeftReq = 0;
// double pwmRightReq = 0;
double pwmMotorReq = 0;


// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

// Keep track of the number of wheel ticks
// std_msgs::Int16 right_wheel_tick_count;
// ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
// std_msgs::Int16 left_wheel_tick_count;
// ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::Int16 wheel_tick_count;
ros::Publisher wheelPub("wheel_ticks", &wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

const int K_P = 252;

void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis()/1000);

  pwmMotorReq = K_P * cmdVel.linear.x;
}

void set_pwm_values() {
  int tempMotorPwm = 0;
  if (pwmMotorReq < 0) {
    // reverse
    tempMotorPwm = -1 * (pwmMotorReq);
    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, tempMotorPwm);    
  }
  else {
    // forward
    tempMotorPwm = pwmMotorReq;
    analogWrite(LPWM_Output, tempMotorPwm);
    analogWrite(RPWM_Output, 0);
  }
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );


void setup()
{
  Serial.begin(9600);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(wheelPub);
  nh.subscribe(subCmdVel);
  // nh.advertise(rightPub);
  // nh.advertise(leftPub);
  // nh.subscribe(subCmdVel);
}

void loop()
{
  nh.spinOnce();

  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // publishing tick counts to topics
    // leftPub.publish( &left_wheel_ticket_count);
    // rightPub.publish( &right_wheel_ticket_count);
    wheelPub.publish( &wheel_tick_count );

    // calculating the velocity of the right and left wheels
    // calc_vel_right_wheel();
    // calc_vel_left_wheel();
    //calc_vel_wheel();
  }

  // stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    // pwmLeftReq = 0;
    // pwmRightReq = 0;
    pwmMotorReq = 0;
  }

  set_pwm_values();
}