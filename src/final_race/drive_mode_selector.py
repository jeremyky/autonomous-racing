import cv2
import rospy
import numpy as np
import math
from std_msgs.msg import Int32, String, Float32, Header, Int16
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf
from overtaker_config import *

from laser_geometry import LaserProjection
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32
from obstacle_detector import ObstacleDetector
from raceline_merchant import RacelineMerchant

# Map metadata from the YAML file
origin_x, origin_y = -6.977912, -3.423147  # Origin of the map
resolution = 0.025000  # Resolution of the map


class DriveModeSelector:

    # set mode
    def __init__(self):
        self.obstacle_detector = ObstacleDetector()

        self.drive_mode_pub = rospy.Publisher('/{}/drive_mode'.format(CAR_NAME), Int32, queue_size=1)
        self.raceline_pub = rospy.Publisher('/{}/select_raceline'.format(CAR_NAME), String, queue_size=1)
        self.drive_mult_pub = rospy.Publisher('/{}/speed_mult'.format(CAR_NAME), Float32, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/{}/obstacles'.format(CAR_NAME), PointCloud2, queue_size=1)
        
        self.map = cv2.imread("/home/{}/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race/map/base_map.pgm".format(PATH_FOLDER), cv2.IMREAD_GRAYSCALE)
        
        self.current_pose = None
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), 
                        PoseStamped, self.pose_callback)
        
        self.raceline_merchant = RacelineMerchant()
        self.counter = 0
        self.every = 1

        self.origin_x, self.origin_y = -6.977912, -3.423147  # Origin of the map


        self.sector = None
        rospy.Subscriber('/{}/sector'.format(CAR_NAME), Int32, self.sector_callback)


    def sector_callback(self, data):
        self.current_sector = data.data
    
    def pose_callback(self, msg):
        self.current_pose = msg.pose

        obstacles = self.obstacle_detector.get_obstacle_points()

        raceline = None
        for raceline_name in RACELINES_IN_ORDER:
            raceline_points = self.raceline_merchant.construct_path(raceline_name)
            if self.is_path_clear(raceline_points, obstacles):
                raceline = raceline_name
                break

        print("raceline:", raceline)

        if raceline is not None:
            self.set_raceline(raceline)
        else:
            print("setting to ftg")
            self.set_mode_ftg()

    def _get_distance(self, p1, p2):
        x,y = p1.position.x, p1.position.y
        p1 = (x,y)
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def get_closest_idx(self,pose,raceline):
        """
        Find closest point on the reference path to the car's current position
        """
        min_dist = float('inf')
        base_projection_index = 0  # index to store waypoint on the path
        for index, point in enumerate(raceline):
            distance = self._get_distance(pose, point)
            if distance < min_dist:
                min_dist = distance
                base_projection_index = index
        return base_projection_index

    def is_path_clear(self, raceline_points, obstacles):
        # converted_raceline_points = []
        # for i in range(len(raceline_points)):
        #     converted_raceline_points.append((raceline_points[i][0], raceline_points[i][1]))


        total_dist = 0
        idx = self.get_closest_idx(self.current_pose,raceline_points) #starts at closest point
        idx -= 4

        prev_point = raceline_points[idx]

        
        while(1):
            idx = (idx + 1) % len(raceline_points) # increment to loop

            point = raceline_points[idx]
            # Calculate distance along raceline
            total_dist += math.sqrt((point[0] - prev_point[0])**2 + (point[1] - prev_point[1])**2)
            
            # Stop checking if we've looked far enough ahead
            if total_dist > RACELINE_LOOKAHEAD:
                return True
                
            # Check if any obstacles are too close to this point
            for obstacle in obstacles:
                dist = math.sqrt((point[0] - obstacle[0])**2 + (point[1] - obstacle[1])**2)
                # print(obstacle, point)
                if dist < SAFETY_DISTANCE:
                    return False
                    
            prev_point = point
            
        return True

    def set_mode_stop(self):
        self.drive_mode_pub.publish(DriveMode.STOP)

    def set_drive_mult(self, mult):
        self.drive_mult_pub.publish(mult)
        
    def set_mode_cc(self):
        self.raceline_pub.publish(RACELINES_IN_ORDER[0])

        # maintain a distance of 1.5 meters from the closest object ahead.        
        closest_distance = self.get_closest_object() # closest point (in front)

        if closest_distance < CC_DISTANCE: 
            self.set_drive_mult(0.0)
        else:
            self.set_drive_mult(1.0)

    def get_closest_object(self):
        # If no obstacle points, return max distance
        if not self.obstacle_detector.obstacle_points:
            return float('inf')
            
        car_x, car_y = self.current_pose.position.x, self.current_pose.position.y
        min_distance = float('inf')

        for x, y in self.obstacle_detector.obstacle_points:
            distance = math.sqrt((x - car_x)**2 + (y - car_y)**2)
            min_distance = min(min_distance, distance)
            
        return min_distance

    def set_mode_ftg(self):
        self.drive_mode_pub.publish(DriveMode.FTG)
        
    def set_mode_pp(self):
        self.drive_mode_pub.publish(DriveMode.PP)

    def set_raceline(self, raceline):
        if raceline not in RACELINES:
            raise ValueError("Invalid raceline")
        self.set_mode_pp()
        self.raceline_pub.publish(raceline)
