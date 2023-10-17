#!/usr/bin/env python

# importaing the GPIO as the PBPI for 3B+ and 4 or higher
import RPi.GPIO as GPIO
import rospy

from std_msgs.msg import String, Float64

# Testing funcation
# return 'HI' as the respond for the call
def hello_callback(data):
    if data.data == "hello":
        pub.publish("HI")
# Led Pin Testing funcation
# THis is only workijng if you have connect the Led to the GPIO 1 And connect other end to GND
def light_callback(data):
    if data.data == "on":
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(1, GPIO.OUT)
        GPIO.output(1, GPIO.HIGH)

def speed_callback(data):
    speed = data.data
    rospy.loginfo("Received speed: %s", speed)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("hello", String, hello_callback)
    rospy.Subscriber("light", String, light_callback)
    rospy.Subscriber("speed", Float64, speed_callback)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('hello', String, queue_size=10)
    listener()
