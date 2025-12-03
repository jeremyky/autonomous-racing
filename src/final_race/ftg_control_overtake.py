import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from disparity_extension import DisparityExtender
from gap_finder import GapFinder
from ftg_config import *
from ftg_visualizer import FTGVisualizer
class FTGControl:
    def __init__(self):
        self.disparity_extender = DisparityExtender(DISPARITY_DISTANCE, SAFETY_EXTENSION, MAX_LIDAR_DISTANCE)
        self.gap_finder = GapFinder(GAP_SELECTION, POINT_SELECTION, MIN_GAP_SIZE, MIN_GAP_DISTANCE, CORNERING_DISTANCE)
        self.ftg_visualizer = FTGVisualizer()
               
        self.ranges = None
        self.prev_speed = 0
    
    def ftg_control(self, data):
        """
        callback function to process LIDAR data
        - identify disparities and applies safety measures
        - search for the best gap to navigate
        - publish steering and velocity commands based on the identified gap
        """
        # preprocess LIDAR data by converting LIDAR data to numpy array and handle NaNs (replace NaNs w/ max range)
        ranges = np.array(data.ranges)
        ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)
        ranges[ranges < 0.05] = MAX_LIDAR_DISTANCE
        # disparity extension
        ranges = self.disparity_extender.extend_disparities(ranges, data.angle_increment)
        self.ranges = ranges
        # find largest gap and select the best point from that gap
        self.gap_finder.update_data(ranges, data)
        gap = self.gap_finder.get_gap()
        start_i, end_i = gap
        best_point = self.gap_finder.get_point(start_i, end_i)
        # best_point = self.gap_finder.get_point_to_go_to(data) # This does the same as above, but in one line
        # calculate steering angle towards best point
        steering_angle = self.get_steering_angle(best_point, data)
        speed = self.dynamic_velocity(steering_angle)
        # publish
        ret = self.publish_drive(speed, steering_angle)
        self.ftg_visualizer.publish_gap_points(data, start_i, end_i)
        
        self.ftg_visualizer.publish_lidar(data, ranges)
        return ret
    # TODO: This probably needs tuning
    def get_steering_angle(self, best_point, data):
        steering_direction = self.get_angle_of_lidar_idx(best_point, data)
        steering_offset = steering_direction - 90
        steering_angle = steering_offset * STEERING_MULTIPLIER
        return steering_angle
    
    def get_angle_of_lidar_idx(self, idx, data):
        # angle_min = data.angle_min
        angle_min = -(data.angle_min % math.pi)
        angle = idx * data.angle_increment + angle_min
        return math.degrees(angle)
    def publish_drive(self, speed, steering_angle):
        """
        publish to AckermannDrive with our calculated steering angle
        """
        # create a new AckermannDrive message
        command = AckermannDrive()
        # clamp steering angle between [-100, 100] degrees
        command.steering_angle = min(max(steering_angle, -100), 100)
        
        # set speed for no gaps dtected for car to slow down or to defult
        command.speed = min(self.prev_speed + 5, speed)
        self.prev_speed = command.speed
        # publish drive command
        return command
    def dynamic_velocity(self, steering_angle):
        """
        adjust speed based on the distance to the nearest obstacle
        """
        # calculate absolute value of the steering angle to determine severity of the turn
        abs_angle = abs(steering_angle)
        # gradually slow speed as the steering angle increases (sharp turn)
        speed = np.interp(abs_angle, [0, 100], [MAXIMUM_SPEED, MINIMUM_SPEED])
        return speed
    
    def better_dynamic_velocity(self):
        min_index = self.gap_finder.get_index_of(40)
        max_index = self.gap_finder.get_index_of(140)
        new_ranges = self.ranges
        new_ranges[new_ranges < 0.1] = 10
        closest = np.min(new_ranges[min_index : max_index])
        speed = np.interp(closest, [0.25, MAX_LIDAR_DISTANCE], [MINIMUM_SPEED, MAXIMUM_SPEED])
        # print(closest, speed)
        return speed
if __name__ == '__main__':
    """
    init ros node and keep running 
    """
    rospy.init_node('follow_gap', anonymous=True)
    FTGControl()
    rospy.spin()