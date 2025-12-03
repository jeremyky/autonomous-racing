import os
import csv
import rospy
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from overtaker_config import *

PATH_FOLDER = '/home/{}/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race/racelines'.format(PATH_FOLDER)

class RacelineMerchant:
    _instance = None
    _cache = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RacelineMerchant, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.initialized = True
            self.plan = []
            self.raceline_pub = rospy.Publisher('/{}/raceline'.format(CAR_NAME), PolygonStamped, queue_size=1)

    def construct_path(self, trajectory_name, publish=False):
        """
        Function to construct the path from a CSV file. Uses caching to avoid re-reading files.
        """
        if trajectory_name in RacelineMerchant._cache:
            self.plan = RacelineMerchant._cache[trajectory_name]
            if publish:
                self.publish_raceline()
            return self.plan

        self.plan = []
        self.speed_plan = []
        
        file_path = os.path.expanduser(
            '{}/{}.csv'.format(PATH_FOLDER, trajectory_name))

        csv_read = []
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                csv_read.append(waypoint)

        # Convert string coordinates to floats
        for index in range(0, len(csv_read)):
            self.plan.append([0, 0])
            self.plan[index][0] = float(csv_read[index][0])
            self.plan[index][1] = float(csv_read[index][1])
            if len(csv_read[index]) > 2:
                self.speed_plan.append(0)
                self.speed_plan[index] = float(csv_read[index][2])
                
        # Cache the processed path
        self._cache[trajectory_name] = self.plan[:]

        # Publish the new raceline
        if publish:
            self.publish_raceline()
        return self.plan

    def publish_raceline(self):
        """
        Publishes the current raceline as a polygon for visualization in RViz
        """
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.Time.now()
        polygon.header.frame_id = "map"

        for point in self.plan:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            polygon.polygon.points.append(p)

        for _ in range(5):
            self.raceline_pub.publish(polygon)
            rospy.sleep(0.1)
