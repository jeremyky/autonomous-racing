#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

from collections import deque

# PID Control Params
# kp = 0.0 #TODO
# kd = 0.0 #TODO
# ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0

# Queue for integral term
errors = deque()
total_error = 0
INTEGRAL_LENGTH = 10


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
# vel_input = 0.0	#TODO

# Publisher for moving the car.
command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global total_error
	global kp, kd, ki

	print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	E = 50
	error = E * data.pid_error

	# Bookkeeping for integral value
	errors.append(error)
	total_error += error
	if len(errors) > INTEGRAL_LENGTH:
		total_error -= errors.popleft()


	# 2. Apply the PID equation on error to compute steering
	P = error
	I = total_error
	D = prev_error - error

	angle = kp * P + kd * D # + ki * I + kd * D


	vel_error = data.pid_vel
	kp_vel = 12

	speed = kp_vel * vel_error
	speed = max(min(50, speed), 0)


	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	print("Error: " + str(data.pid_error))
	print("Steering Angle: " + str(angle))
	print("Vel Error: " + str(vel_error))
	print("Vel: " + str(speed))
	command.steering_angle = max(min(100, angle), -100)

	# TODO: Make sure the velocity is within bounds [0,100]
	command.speed = speed

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp, kd, ki
	global vel_input
	kp = 6 # input("Enter Kp Value: ")
	kd = 4.5 # input("Enter Kd Value: ")
	ki = 0 # input("Enter Ki Value: ")
	vel_input = 15 # input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
