#!/usr/bin/env python

# Import necessary libraries
import rospy
import sys
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
import tf

from pp_config import *
from pure_pursuit import PurePursuit

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0


class PurePursuitControl:
    def __init__(self):
        # Global variables for storing the path, path resolution, frame ID, and car details
        self.frame_id = 'map'
        self.car_name = "car_8"  # str(sys.argv[1])
        self.trajectory_name = str(rospy.get_param("~raceline", "demoline")) # str(sys.argv[2])

        # Publishers for sending driving commands and visualizing the control polygon
        self.command_pub = rospy.Publisher('/{}/offboard/command'.format(self.car_name), AckermannDrive, queue_size=1)
        self.polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(self.car_name), PolygonStamped,
                                           queue_size=1)

        # Global variables for waypoint sequence and current polygon
        self.wp_seq = 0
        self.control_polygon = PolygonStamped()

        self.pp = PurePursuit(LOOKAHEAD_DISTANCE, VELOCITY_LOOKAHEAD_DISTANCE, MIN_SPEED, MAX_SPEED)
        self.setup()

    def setup(self):
        rospy.loginfo("Obtaining trajectory")
        self.pp.construct_path(self.trajectory_name)

        # This node subscribes to the pose estimate provided by the Particle Filter.
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        car_name = "car_8"
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped,
                         self.pure_pursuit_control)
        
        rospy.loginfo("Ready for pure pursuit")

    def pure_pursuit_control(self, data):
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

        # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
        # Your code here
        # check within range and then assign
        steering_angle *= STEERING_MULTIPLIER
        steering_angle = max(-STEERING_RANGE, min(STEERING_RANGE, steering_angle))
        command.steering_angle = steering_angle

        # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
        # basically what we did in ftg algo and also without dealing with numpy
        abs_steering_angle = abs(steering_angle)
        if abs_steering_angle >= 100:
            command.speed = MIN_SPEED
        else:
            command.speed = MAX_SPEED - ((abs_steering_angle / 100.0) * (MAX_SPEED - MIN_SPEED))
        # command.speed = self.pp.get_dynamic_velo()

        self.command_pub.publish(command)
        self.publish_rviz_markers(odom_x, odom_y, pose_x, pose_y, target_x, target_y)

    def publish_rviz_markers(self, odom_x, odom_y, pose_x, pose_y, target_x, target_y):
        # Visualization code
        # Make sure the following variables are properly defined in your TODOs above:
        # - odom_x, odom_y: Current position of the car
        # - pose_x, pose_y: Position of the base projection on the reference path
        # - target_x, target_y: Position of the goal/target point

        base_link = Point32()
        nearest_pose = Point32()
        nearest_goal = Point32()
        base_link.x = odom_x
        base_link.y = odom_y
        nearest_pose.x = pose_x
        nearest_pose.y = pose_y
        nearest_goal.x = target_x
        nearest_goal.y = target_y
        self.control_polygon.header.frame_id = self.frame_id
        self.control_polygon.polygon.points = [nearest_pose, base_link, nearest_goal]
        self.control_polygon.header.seq = self.wp_seq
        self.control_polygon.header.stamp = rospy.Time.now()
        self.wp_seq = self.wp_seq + 1
        self.polygon_pub.publish(self.control_polygon)


if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)
        pp_control = PurePursuitControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("An error occurred: {}", rospy.ROSInterruptException)
        pass
