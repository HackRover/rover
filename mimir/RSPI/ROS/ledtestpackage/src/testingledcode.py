#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy

from std_msgs.msg import String

def hello_callback(data):
    rospy.loginfo("I heard %s", data.data)
    if data.data == "hello":
        pub.publish("hi")

def light_callback(data):
    rospy.loginfo("I heard %s", data.data)
    if data.data == "on":
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(1, GPIO.OUT)
        GPIO.output(1, GPIO.HIGH)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("hello", String, hello_callback)
    rospy.Subscriber("light", String, light_callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('hello', String, queue_size=10)
    listener()
