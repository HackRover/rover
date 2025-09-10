# Created By: Houming Ge
# 2025/9/10
#
# This is still in progress of testing
# Not the final version
# Version: 0.3.1

#!/usr/bin/env python
import rospy
import pygame
import RPi.GPIO as GPIO
from motor_control.msg import MotorSpeed  # your custom message


# GPIO Setup

GPIO.setmode(GPIO.BCM)

MOTOR1_PWM_PIN = 18
MOTOR1_DIR_PIN1 = 23
MOTOR1_DIR_PIN2 = 24

MOTOR2_PWM_PIN = 12
MOTOR2_DIR_PIN1 = 20
MOTOR2_DIR_PIN2 = 21

GPIO.setup(MOTOR1_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR1_DIR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR1_DIR_PIN2, GPIO.OUT)

GPIO.setup(MOTOR2_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR2_DIR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR2_DIR_PIN2, GPIO.OUT)

motor1_pwm = GPIO.PWM(MOTOR1_PWM_PIN, 1000)  # 1000 Hz
motor2_pwm = GPIO.PWM(MOTOR2_PWM_PIN, 1000)

motor1_pwm.start(0)
motor2_pwm.start(0)


# ROS Node + Publisher

def xbox_motor_control_node():
    rospy.init_node("xbox_motor_control_node", anonymous=True)
    motor_speed_pub = rospy.Publisher("motor_speed", MotorSpeed, queue_size=10)

    # Init pygame joystick
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        rospy.logerr("No Xbox controller detected! Make sure it's paired over Bluetooth.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    rospy.loginfo("Xbox Controller connected: %s", joystick.get_name())

    rate = rospy.Rate(20)  # 20 Hz loop

    while not rospy.is_shutdown():
        pygame.event.pump()

        # Left stick Y-axis → Motor 1
        left_stick_y = -joystick.get_axis(1)
        # Right stick Y-axis → Motor 2
        right_stick_y = -joystick.get_axis(3)

        motor1_speed = int(left_stick_y * 100)  # range -100..100
        motor2_speed = int(right_stick_y * 100)


        # Motor Direction + PWM Control
        if motor1_speed >= 0:
            GPIO.output(MOTOR1_DIR_PIN1, GPIO.HIGH)
            GPIO.output(MOTOR1_DIR_PIN2, GPIO.LOW)
        else:
            GPIO.output(MOTOR1_DIR_PIN1, GPIO.LOW)
            GPIO.output(MOTOR1_DIR_PIN2, GPIO.HIGH)
        motor1_pwm.ChangeDutyCycle(abs(motor1_speed))

        if motor2_speed >= 0:
            GPIO.output(MOTOR2_DIR_PIN1, GPIO.HIGH)
            GPIO.output(MOTOR2_DIR_PIN2, GPIO.LOW)
        else:
            GPIO.output(MOTOR2_DIR_PIN1, GPIO.LOW)
            GPIO.output(MOTOR2_DIR_PIN2, GPIO.HIGH)
        motor2_pwm.ChangeDutyCycle(abs(motor2_speed))


        # Publish current motor speeds

        msg = MotorSpeed()
        msg.motor1_speed = motor1_speed
        msg.motor2_speed = motor2_speed
        motor_speed_pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        xbox_motor_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor1_pwm.stop()
        motor2_pwm.stop()
        GPIO.cleanup()
        pygame.quit()
