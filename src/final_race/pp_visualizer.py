import rospy
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

from overtaker_config import *


class PPVisualizer:
    def __init__(self):
        self.frame_id = 'map'
        self.polygon_pub = rospy.Publisher('/{}/purepursuit_control/visualize'.format(CAR_NAME), PolygonStamped,
                                           queue_size=1)

        # Global variables for waypoint sequence and current polygon
        self.wp_seq = 0
        self.control_polygon = PolygonStamped()

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
