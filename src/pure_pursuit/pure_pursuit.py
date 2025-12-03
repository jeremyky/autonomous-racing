#!/usr/bin/env python

import os
import csv
import math
from pp_config import *
from overtaker_config import PATH_FOLDER

WHEELBASE_LEN = 0.325


class PurePursuit:
    def __init__(self, lookahead_distance, velo_lookahead_distance, min_speed, max_speed):
        self.lookahead_distance = lookahead_distance
        self.velo_lookahead_distance = velo_lookahead_distance
        self.min_speed = min_speed
        self.max_speed = max_speed

        self.plan = []
        self.path_resolution = []

        self.odom = None
        self.base_proj = None
        self.base_proj_index = None
        self.target = None

        self.heading = None

    def construct_path(self, trajectory_name):
        """
        Function to construct the path from a CSV file
        """

        # TODO: Modify this path to match the folder where the csv file containing the path is located.
        file_path = os.path.expanduser(
            '/home/{}/depend_ws/src/F1tenth_car_workspace/wallfollow/src/pure_pursuit/{}.csv'.format(PATH_FOLDER, trajectory_name))

        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for waypoint in csv_reader:
                self.plan.append(waypoint)

        # Convert string coordinates to floats and calculate path resolution
        for index in range(0, len(self.plan)):
            for point in range(0, len(self.plan[index])):
                self.plan[index][point] = float(self.plan[index][point])

        for index in range(1, len(self.plan)):
            dx = self.plan[index][0] - self.plan[index - 1][0]
            dy = self.plan[index][1] - self.plan[index - 1][1]
            self.path_resolution.append(math.sqrt(dx * dx + dy * dy))

    def pure_pursuit(self, odom_x, odom_y, heading):
        self.odom = (odom_x, odom_y)
        self.heading = heading

        self.get_base_projection()
        self.get_lookahead_point()
        test = self.get_steering_angle()
        angle = abs(test)
        self.lookahead_distance = MIN_LOOK_AHEAD_DISTANCE + (((100-angle) / 100) * (MAX_LOOK_AHEAD_DISTANCE - MIN_LOOK_AHEAD_DISTANCE))
        #if angle >= 90:
            #self.lookahead_distance = MAX_LOOK_AHEAD_DISTANCE
        #else:
            #self.lookahead_distance = MIN_LOOK_AHEAD_DISTANCE

        print("lookahead:")
        print(self.lookahead_distance)
        print("angle: ")
        print(angle)

        angle = self.get_steering_angle()
        return angle

    def get_base_projection(self):
        """
        Find closest point on the reference path to the car's current position
        """
        # TODO 1: The reference path is stored in the 'plan' array.
        # initizalize min_dist to infinity for comparison purposes so that we can always find the smallest dist later
        min_dist = float('inf')
        base_projection_index = 0  # index to store waypoint on the path
        # loop over all the waypoints in the ref path and check which is closest to car
        for index, point in enumerate(self.plan):
            distance = self._get_distance(self.odom, point)
            if distance < min_dist:
                min_dist = distance
                base_projection_index = index
        # get the coordinates for base projection
        self.base_proj = (self.plan[base_projection_index][0], self.plan[base_projection_index][1])
        self.base_proj_index = base_projection_index

    def get_lookahead_point(self):
        """
        Follow path starting from base_projection and get first point that is lookahead_distance away
        """
        # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
        target_index = self.base_proj_index
        # loop through waypoints along our path until target point is at least lookahead_distance away and we havent
        # reached end
        while target_index < len(self.plan) - 1:
            if self._get_distance(self.odom, self.plan[target_index]) >= self.lookahead_distance:
                break
            target_index += 1

        # TODO: Make this interpolate if target point is beyond lookahead distance
        # target coordiantes for where we want the car to go to next bc they're just far enough past lookahead dist
        # without overshooting
        self.target = (self.plan[target_index-1][0], self.plan[target_index-1][1])

    def get_steering_angle(self):
        """
        Compute the steering angle given the pose of the car, target point, and lookahead distance
        """

        # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
        # Your code here

        """
        calc angle btwn car's curr direction and target point
        short info on formula below but its mostly in lec 14 lol
        explanation:
        math.atan2(target_y - odom_y, target_x - odom_x): calc angle of the line from the car's position to the target point relative to the x-axis
        subtracting 'heading' accounts for the car's curr orientation, which gives alpha (the misalignment angle)
        alpha tells us how much the car needs to turn to face the target point (left = pos, right = neg lmao)
        """
        alpha = math.atan2(self.target[1] - self.odom[1] ,  self.target[0] - self.odom[0]) - self.heading

        # alternative equation from slides
        # alpha = math.asin(self.odom[1] - self.target[1] / self.lookahead_distance)

        """
        WHEELBASE_LEN: dist btwn the car's front and rear wheels since it affects how sharply the car can turn
        math.sin(alpha): measure how far off the car is from being aligned with the target point (lateral error)
        lookahead_distance: dist to target point - larger lookahead should have smoother & less reactive paths i think
        basically this: larger alpha -> more misaligned -> larger steering angle -> sharper turn
    
        """
        steering_angle = math.atan2(2 * WHEELBASE_LEN * math.sin(alpha), self.lookahead_distance)
        steering_angle = math.degrees(steering_angle)
        print("Alpha:", alpha, "Steering Angle:", steering_angle)

        return steering_angle
    
    def get_dynamic_velo(self):
        velo_index = self.base_proj_index
        dist = 0
        while dist < self.velo_lookahead_distance:
            velo_index = (velo_index + 1) % len(self.plan)
            dist = self._get_distance(self.plan[velo_index], self.odom)

        target = (self.plan[velo_index][0], self.plan[velo_index][1])
        dist = self._get_distance(self.odom, target)

        alpha = math.atan2(target[1] - self.odom[1], target[0] - self.odom[0]) - self.heading
        steering_angle = math.atan2(2 * WHEELBASE_LEN * math.sin(alpha), dist)
        steering_angle = math.degrees(steering_angle)

        steering_angle = abs(steering_angle)
        print(steering_angle)
        if steering_angle >= 15:
            speed = self.min_speed
        else:
            speed = self.max_speed - ((steering_angle / 15.0) * (self.max_speed - self.min_speed))

        return speed



    def _get_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
