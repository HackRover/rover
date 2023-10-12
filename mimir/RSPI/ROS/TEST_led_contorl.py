# Created By: Houming Ge
# 2023/09/07
# 
# FIXME: Have not yet tested on the RSPI. I need some people who have RSPI that have ROS to run and tell me what error code.
# 
#!/usr/bin/env python

# THis is still in debug. THis file is a testing code when you need to using this code. You need to folllow the README.md file to seeting on the system setting.

# importanting the library
import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Bool

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the LED GPIO pin
# If your RBPI having already using this Pin
# Please change
# Same Mean as GPIO 1
LED_PIN = 12

# Initialize the GPIO pin as an output
GPIO.setup(LED_PIN, GPIO.OUT)

# function that chcking the message that being send to this function
# this function that also setting the led status base on the incoming message
# when the led recived the led message it will also sending the confim informtion back as message type
def led_control_callback(data):
    # Callback function to control the LED based on incoming messages
    new_state = data.data

    # Set the LED state
    GPIO.output(LED_PIN, new_state)

    # Publish a message to confirm the LED state change
    rospy.loginfo("LED state set to: %s", new_state)
    led_state_pub.publish(new_state)

# this function first init a node name led_control_node
# it will start to listern any message that was topic name led_control_node
# When it bing initial, it will send the initial led state
def led_control_node():
    global led_state_pub
    rospy.init_node('led_control_node', anonymous=True)
    
    # Subscribe to a topic to control the LED
    rospy.Subscriber('led_control', Bool, led_control_callback)

    # Publish the initial LED state (assuming it's initially OFF)
    led_state_pub = rospy.Publisher('led_state', Bool, queue_size=10)
    led_state_pub.publish(False)

    rospy.spin()

# main
if __name__ == '__main__':
    try:
        led_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
