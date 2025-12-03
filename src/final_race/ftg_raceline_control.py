#!/usr/bin/env python

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

from raceline_merchant import RacelineMerchant
from overtaker_config import *

class FTGRacelineControl:
    def __init__(self):
        self.disparity_extender = DisparityExtender(DISPARITY_DISTANCE, SAFETY_EXTENSION, MAX_LIDAR_DISTANCE)
        self.gap_finder = GapFinder(GAP_SELECTION, POINT_SELECTION, MIN_GAP_SIZE, MIN_GAP_DISTANCE, CORNERING_DISTANCE)

        self.ftg_visualizer = FTGVisualizer()
               
        self.ranges = None
        self.prev_speed = 0

        self.raceline_merchant = RacelineMerchant()
    
    def ftg_control(self, data):
        """
        callback function to process LIDAR data
        - identify disparities and applies safety measures
        - search for the best gap to navigate
        - check if racelines go through gaps
        """
        # preprocess LIDAR data by converting LIDAR data to numpy array and handle NaNs (replace NaNs w/ max range)
        ranges = np.array(data.ranges)
        ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)
        ranges[ranges < 0.05] = MAX_LIDAR_DISTANCE

        # disparity extension
        ranges = self.disparity_extender.extend_disparities(ranges, data.angle_increment)
        self.ranges = ranges

        # find gaps and check racelines
        best_raceline = None
        try:
            self.gap_finder.update_data(ranges, data)
            start_i, end_i = self.gap_finder.get_gap()
            start_angle = self.get_angle_of_lidar_idx(start_i, data)
            end_angle = self.get_angle_of_lidar_idx(end_i, data)
            
            # Get current pose
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            current_yaw = 2 * math.atan2(self.current_pose.orientation.z, 
                                       self.current_pose.orientation.w)
            
            # Check each raceline
            for raceline_name in RACELINES_IN_ORDER:
                self.raceline_merchant.construct_path(raceline_name)
                raceline_points = self.raceline_merchant.plan
                
                # Check points until beyond lidar range
                works = True
                for point in raceline_points:
                    # Convert point to local coordinates
                    dx = point[0] - current_x
                    dy = point[1] - current_y
                    
                    # Rotate to car's frame
                    local_x = dx * math.cos(-current_yaw) - dy * math.sin(-current_yaw)
                    local_y = dx * math.sin(-current_yaw) + dy * math.cos(-current_yaw)
                    
                    # Skip if point is beyond lidar range
                    distance = math.sqrt(local_x**2 + local_y**2)
                    if distance > MAX_LIDAR_DISTANCE:
                        break
                        
                    # Get angle to point
                    point_angle = math.atan2(local_y, local_x)
                    
                    # Check if point is in gap
                    if not (start_angle <= point_angle <= end_angle):
                        works = False
                        break
                
                if works:
                    best_raceline = raceline_name
                    break
        except:
            pass
        
        self.ftg_visualizer.publish_lidar(data, ranges)
        return best_raceline

    def get_angle_of_lidar_idx(self, idx, data):
        # angle_min = data.angle_min
        angle_min = -(data.angle_min % math.pi)
        angle = idx * data.angle_increment + angle_min
        return math.degrees(angle)


