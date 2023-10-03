# DEBUG
# Created By: Houming Ge
# 2023/9/9
#
# This is still in progress of testing
# Not the final version
# Verison: 0.2.3

# How to use:
# To control the motors, you can publish messages to the motor_control topic with the std_msgs/Int16 message type.
# Example:
# rostopic pub /motor_control std_msgs/Int16 "data: [50, -75]"
# 50: 50% of the speed in forward
# -75: 75% of the speed in backward

# OUTPUT Example:
# [INFO] [1632772812.123456]: Motor speeds updated - Motor 1: 50%, Motor 2: -75%
#  Type          ID           Text about this       Current speed  Current speed
#                               information          for Motor 1    for Motor 2

#!/usr/bin/env python
# Impotanting the library
import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Int16
from motor_control.msg import MotorSpeed  # Create a custom message type

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the motor control pins
# If your RBPI having Pins that have already using
# Please change them
MOTOR1_PWM_PIN = 18
MOTOR1_DIR_PIN1 = 23
MOTOR1_DIR_PIN2 = 24

MOTOR2_PWM_PIN = 12
MOTOR2_DIR_PIN1 = 20
MOTOR2_DIR_PIN2 = 21

# Initialize the GPIO pins
GPIO.setup(MOTOR1_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR1_DIR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR1_DIR_PIN2, GPIO.OUT)

GPIO.setup(MOTOR2_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR2_DIR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR2_DIR_PIN2, GPIO.OUT)

# Create PWM objects for motors
motor1_pwm = GPIO.PWM(MOTOR1_PWM_PIN, 1000)  # 1000 Hz frequency
motor2_pwm = GPIO.PWM(MOTOR2_PWM_PIN, 1000)  # 1000 Hz frequency

# Initialize the motor speeds
# The motor speeds should be zero at the Initialize
current_motor1_speed = 0
current_motor2_speed = 0

def motor_control_callback(data):
    global current_motor1_speed, current_motor2_speed

    motor1_speed = data.data[0]
    motor2_speed = data.data[1]

    # TODO: need to be making sure this GPIO is output like that if the output is wrobng swide need adding in the desption
    # Set motor direction and speed using PWM duty cycle (0 to 100%)
    # Motor 1 direction
    if motor1_speed >= 0:
        GPIO.output(MOTOR1_DIR_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR1_DIR_PIN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR1_DIR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR1_DIR_PIN2, GPIO.HIGH)
    motor1_pwm.start(abs(motor1_speed))

    # Motor 2 direction
    if motor2_speed >= 0:
        GPIO.output(MOTOR2_DIR_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR2_DIR_PIN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR2_DIR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR2_DIR_PIN2, GPIO.HIGH)
    motor2_pwm.start(abs(motor2_speed))

    # Update current motor speeds
    current_motor1_speed = motor1_speed
    current_motor2_speed = motor2_speed

    # Publish a message with the current motor speeds
    motor_speed_msg = MotorSpeed()
    motor_speed_msg.motor1_speed = current_motor1_speed
    motor_speed_msg.motor2_speed = current_motor2_speed
    motor_speed_pub.publish(motor_speed_msg)

def motor_control_node():
    global motor_speed_pub
    # required to initing a node for the server on the nano to know this is a node.
    rospy.init_node('motor_control_node', anonymous=True)

    # Subscribe to a topic to control the motors
    rospy.Subscriber('motor_control', Int16, motor_control_callback)

    # Publish the initial motor speeds (0%)
    motor_speed_pub = rospy.Publisher('motor_speed', MotorSpeed, queue_size=10)
    initial_motor_speed_msg = MotorSpeed()
    initial_motor_speed_msg.motor1_speed = 0
    initial_motor_speed_msg.motor2_speed = 0
    motor_speed_pub.publish(initial_motor_speed_msg)

    rospy.spin()

# NO CHANGE 
# This is the inition that need to send as this is also like main in C++
if __name__ == '__main__':
    # hecking
    # Debug the node at the main node when it waws in the debug mode.
    # If the node can not taking any message, Checking the connect between Nano and RBPI.
    # If the node is return the error checking the input of the sppe IT was Not 128
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor1_pwm.stop()
        motor2_pwm.stop()
        GPIO.cleanup()

