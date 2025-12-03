#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.8	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 1.3 # distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

prev_readings = {}
cur_vel = 0

# Publisher that will publish on the 'error' topic messages of type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data, angle):
    # data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
     #TODO: implement

    #  if not -30 <= angle <= 210:
    #     raise ValueError "Angle " + str(angle) + " out of bounds"

     # Convert to radians because LaserScan is in radians
    angle_rad = math.radians(angle)

    angle_min = -(data.angle_min % math.pi)
    index = int((angle_rad - angle_min) / data.angle_increment)
    
    range_value = data.ranges[index]

    if np.isnan(range_value):
        range_value = prev_readings.get(angle, 2)
    else:
        prev_readings[angle] = range_value
    
    return range_value


def callback(data):
    """
    Callback function to process LIDAR data and publish errors for PID control.
    """
    global forward_projection

    thetas = [45, 60, 65, 70, 72, 75, 80]# Angles to use for error calculation

    error = 0
    for theta in thetas:
        a = getRange(data,theta) # obtain the ray distance for theta
        b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
        swing = math.radians(theta)

        # ----------------------------------------------
        
        ## Your code goes here to determine the projected error as per the alrorithm
        # Compute Alpha, AB, and CD..and finally the error.
        alpha = math.atan((a * math.cos(swing) - b)/ (a * math.sin(swing)))
        AB = b * math.cos(alpha)

        # dynamic_forward_projection = min(1, cur_vel *  0.05)
        # print("Forward projection: ", dynamic_forward_projection)
        
        CD = AB + (forward_projection) * math.sin(alpha) # forward projection is AC
        # try w/ (forward_projection + car_length) * math.sin(alpha) if bad results
        error += desired_distance - CD
    error /= len(thetas)
    # Get distance straight ahead (90 degrees)
    dist_ahead = getRange(data, 90)
    vel_error = dist_ahead
    print("Distance ahead: " + str(dist_ahead) + " meters")

    # ----------------------------------------------

    # Create and publish the pid_input message
    msg = pid_input()  # An empty msg is created of the type pid_input
    msg.pid_error = error  # Steering error for PID
    msg.pid_vel = vel_error          # Velocity error for PID
    pub.publish(msg)

def vel_callback(data):
    global cur_vel
    cur_vel = data.speed

if __name__ == '__main__':
    print("Hokuyo LIDAR node started")
    rospy.init_node('dist_finder', anonymous=True)
    # Subscribe to the correct scan topic
    rospy.Subscriber('/car_8/offboard/command', AckermannDrive, vel_callback)
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()