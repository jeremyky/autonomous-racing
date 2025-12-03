#!/usr/bin/env python

# Import necessary libraries
import rospy
from ackermann_msgs.msg import AckermannDrive
import tf

from overtaker_config import *
from pp import PurePursuit
from pp_visualizer import PPVisualizer
from pp_config import *

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0


class MultiPPControl:
    def __init__(self):
        self.racelines = RACELINES
        self.current_raceline = None

        self.pp_visualizer = PPVisualizer()

        self.pp = PurePursuit(LOOKAHEAD_DISTANCE, MIN_SPEED, MAX_SPEED)
        self.select_raceline(self.racelines[0])

    def select_raceline(self, raceline):
        if raceline not in self.racelines:
            rospy.logerr("Invalid raceline: {}".format(raceline))
            return
        
        if raceline == self.current_raceline:
            return

        # rospy.loginfo("Switching to raceline: {}".format(raceline))
        self.pp.construct_path(raceline)
        self.current_raceline = raceline
        # rospy.loginfo("Switching to raceline: {} complete".format(raceline))

    def pp_control(self, data):
        # Obtain the current position of the race car from the inferred_pose message
        odom_x = data.pose.position.x
        odom_y = data.pose.position.y

        # Calculate heading angle of the car (in radians)
        heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                            data.pose.orientation.y,
                                                            data.pose.orientation.z,
                                                            data.pose.orientation.w))[2]


        steering_angle = self.pp.pure_pursuit(odom_x, odom_y, heading)

        pose_x, pose_y = self.pp.base_proj
        target_x, target_y = self.pp.target

        command = AckermannDrive()

        # Scale, check within range, and then assign
        steering_angle = max(-STEERING_RANGE, min(STEERING_RANGE, steering_angle))
        command.steering_angle = steering_angle
        command.speed = self.pp.get_dynamic_velo()

        self.pp_visualizer.publish_rviz_markers(odom_x, odom_y, pose_x, pose_y, target_x, target_y)
        return command
