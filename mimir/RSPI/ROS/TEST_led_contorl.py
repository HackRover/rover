# Created By: Houming Ge
# 2023/09/07
# 
# FIXME: Have not yet tested on the RSPI. I need some people who have RSPI that have ROS to run and tell me what error code.
# 
#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Bool

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the LED GPIO pin
LED_PIN = 12

# Initialize the GPIO pin as an output
GPIO.setup(LED_PIN, GPIO.OUT)

def led_control_callback(data):
    # Callback function to control the LED based on incoming messages
    GPIO.output(LED_PIN, data.data)

def led_control_node():
    rospy.init_node('led_control_node', anonymous=True)
    
    # Subscribe to a topic to control the LED
    rospy.Subscriber('led_control', Bool, led_control_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        led_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
