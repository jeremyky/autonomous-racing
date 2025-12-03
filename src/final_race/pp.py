#!/usr/bin/env python

import rospy
import math
from pp_config import *

from raceline_merchant import RacelineMerchant
from overtaker_config import *

from std_msgs.msg import Int32

WHEELBASE_LEN = 0.325


class PurePursuit:
    def __init__(self, lookahead_distance, min_speed, max_speed):
        self.lookahead_distance = lookahead_distance
        self.velo_lookahead_distance = 0
        self.sector_velo_mult = 0
        self.steering_angle = STEERING_MULTIPLIER
        self.min_speed = min_speed
        self.max_speed = max_speed

        self.raceline_merchant = RacelineMerchant()
        self.plan = []
        self.speed_plan = []

        self.odom = None
        self.base_proj = None
        self.base_proj_index = None
        self.target = None

        self.heading = None

        self.sector = None
        rospy.Subscriber('/{}/sector'.format(CAR_NAME), Int32, self.set_sector)

        self.current_steering_angle = 0
        self.max_steering_change = MAX_STEERING_CHANGE

    def set_sector(self, data):
        self.sector = data.data

        if self.sector == Sectors.FREE:
            self.lookahead_distance = FREE_LOOKAHEAD
            self.velo_lookahead_distance = 2 * FREE_LOOKAHEAD
            self.sector_velo_mult = FREE_VELO_MULT
            self.steering_angle = STEERING_MULTIPLIER + 0.5
            self.max_steering_change = MAX_STEERING_CHANGE - 4
        elif self.sector == Sectors.MID:
            self.lookahead_distance = MID_LOOKAHEAD
            self.velo_lookahead_distance = 2 * MID_LOOKAHEAD
            self.sector_velo_mult = MID_VELO_MULT
            self.steering_angle = STEERING_MULTIPLIER - 0.5
            self.max_steering_change = MAX_STEERING_CHANGE - 1
        elif self.sector == Sectors.DANGER:
            self.lookahead_distance = DANGER_LOOKAHEAD
            self.velo_lookahead_distance = 2 * DANGER_LOOKAHEAD
            self.sector_velo_mult = DANGER_VELO_MULT
            self.steering_angle = STEERING_MULTIPLIER - 0.5
            self.max_steering_change = 200

    def construct_path(self, trajectory_name):
        """
        Function to construct the path from a CSV file
        """
        self.plan = self.raceline_merchant.construct_path(trajectory_name, publish=True)
        self.speed_plan = self.raceline_merchant.speed_plan

    def pure_pursuit(self, odom_x, odom_y, heading):
        self.odom = (odom_x, odom_y)
        self.heading = heading

        self.get_base_projection()
        self.get_lookahead_point()
        angle = self.get_steering_angle()
        return angle

    def get_base_projection(self):
        """
        Find closest point on the reference path to the car's current position
        """
        min_dist = float('inf')
        base_projection_index = 0  # index to store waypoint on the path
        for index, point in enumerate(self.plan):
            distance = self._get_distance(self.odom, point)
            if distance < min_dist:
                min_dist = distance
                base_projection_index = index
        # get the coordinates for base projection
        # print("IDX", base_projection_index)
        self.base_proj = (self.plan[base_projection_index][0], self.plan[base_projection_index][1])
        self.base_proj_index = base_projection_index

    def get_lookahead_point(self):
        """
        Follow path starting from base_projection and get first point that is lookahead_distance away
        """
        # target_index = (self.base_proj_index + self.lookahead_distance_index) % len(self.plan)
        target_index = self.get_raceline_point_dist_away(self.base_proj_index, self.lookahead_distance)
        self.target = (self.plan[target_index][0], self.plan[target_index][1])
        return 0

    def get_steering_angle(self, target=None):
        """
        Compute the steering angle given the pose of the car, target point, and lookahead distance
        """
        if target is None:
            target = self.target

        alpha = math.atan2(target[1] - self.odom[1], target[0] - self.odom[0]) - self.heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        steering_angle = math.atan(2 * WHEELBASE_LEN * math.sin(alpha)/ 1.2)
        steering_angle = math.degrees(steering_angle)
        # print("Alpha:", alpha, "Steering Angle:", steering_angle)
        target_steering_angle = steering_angle * self.steering_angle
        if target_steering_angle > self.current_steering_angle:
            if target_steering_angle > self.current_steering_angle + MAX_STEERING_CHANGE:
                self.current_steering_angle = self.current_steering_angle + MAX_STEERING_CHANGE
            else:
                self.current_steering_angle = target_steering_angle
        else:
            if target_steering_angle < self.current_steering_angle - MAX_STEERING_CHANGE:
                self.current_steering_angle = self.current_steering_angle - MAX_STEERING_CHANGE
            else:
                self.current_steering_angle = target_steering_angle

        return self.current_steering_angle


    def get_dynamic_velo(self):
        # # Dynamic speed based on further lookahead point
        target_index = self.get_raceline_point_dist_away(self.base_proj_index, self.velo_lookahead_distance)
        vel_target = (self.plan[target_index][0], self.plan[target_index][1])

        # vel_angle = self.get_steering_angle(vel_target)
        # return (120 - vel_angle) * 0.8

        distance = self._get_distance(self.base_proj, vel_target)

        velo = distance * VELO_MULT * self.sector_velo_mult
        velo = max(self.min_speed, min(self.max_speed, velo))
        return velo
    
        target_index = self.get_raceline_point_dist_away(self.base_proj_index, self.velo_lookahead_distance / 4)
        read_speed = self.speed_plan[target_index]
        return read_speed * 10
    
    def get_raceline_point_dist_away(self, point_ind, dist):
        """
        Returns the index of a point that is dist away from point_ind when following the raceline
        """
        current_dist = 0
        current_ind = point_ind
        
        while current_dist < dist and current_ind < len(self.plan) - 1:
            next_ind = (current_ind + 1) % len(self.plan)
            segment_dist = self._get_distance(self.plan[current_ind], self.plan[next_ind])
            current_dist += segment_dist
            current_ind = next_ind
            
            if current_ind == point_ind:  # We've gone full circle
                break
                
        return current_ind

    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
