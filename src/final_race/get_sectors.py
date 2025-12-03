import rospy
import os
import csv
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String

from overtaker_config import *
from raceline_merchant import RacelineMerchant

PATH_FOLDER = '/home/{}/depend_ws/src/F1tenth_car_workspace/wallfollow/src/final_race/racelines'.format(PATH_FOLDER)

sec_names = {
    "FREE": Sectors.FREE,
    "MID": Sectors.MID,
    "DANGER": Sectors.DANGER
}

sec_ind_to_name = {
    Sectors.FREE: "FREE",
    Sectors.MID: "MID",
    Sectors.DANGER: "DANGER"
}


class GetSectors():
    def __init__(self):
        self.raceline_merchant = RacelineMerchant()
        self.plan = self.construct_path()

        self.command_pub = rospy.Publisher('/{}/sector'.format(CAR_NAME), Int32, queue_size=1)
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), PoseStamped, self.mpp_control)

    def construct_path(self):
        """
        Function to construct the path from a CSV file
        """
        sector_csv_name = "speed_sectors" if SPEED_MODE == "FAST" else "sectors"

        file_path = os.path.expanduser(
            '{}/{}.csv'.format(PATH_FOLDER, sector_csv_name))

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            self.sec_dict = {}
            self.starts = []
            self.secs = []
            for start, sector in csv_reader:
                self.starts.append(int(start))
                self.secs.append(sector)

        self.raceline_merchant.construct_path("mindist")

        return self.raceline_merchant.plan

    def mpp_control(self, data):
        x, y = data.pose.position.x, data.pose.position.y
        self.pose = (x, y)
        self.get_sector()

    def get_sector(self):
        self.get_closest_idx()
        for i, start in enumerate(self.starts):
            if self.closest_idx >= int(start):
                val = sec_names[self.secs[i]]
                self.sector = val

        self.publish_command()

    def get_closest_idx(self):
        """
        Find closest point on the reference path to the car's current position
        """
        min_dist = float('inf')
        base_projection_index = 0  # index to store waypoint on the path
        for index, point in enumerate(self.plan):
            distance = self._get_distance(self.pose, point)
            if distance < min_dist:
                min_dist = distance
                base_projection_index = index
        self.base_proj = (self.plan[base_projection_index][0], self.plan[base_projection_index][1])
        self.closest_idx = base_projection_index

    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def publish_command(self):
        command = Sectors()
        command = self.sector
        print("sector: ", sec_ind_to_name[self.sector])
        self.command_pub.publish(command)
