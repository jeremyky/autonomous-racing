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
from get_sectors import GetSectors


class FTGControl:
    def __init__(self):
        self.drive_pub = rospy.Publisher("/car_8/offboard/command", AckermannDrive, queue_size=10)
        self.lidar_pub = rospy.Publisher("/car_8/lidar/processed", PointCloud2, queue_size=2)
        self.marker1_pub = rospy.Publisher('marker1', Marker, queue_size=10)
        self.marker2_pub = rospy.Publisher('marker2', Marker, queue_size=10)
        rospy.Subscriber("/car_8/scan", LaserScan, self.lidar_callback)

        self.disparity_extender = DisparityExtender(DISPARITY_DISTANCE, SAFETY_EXTENSION, MAX_LIDAR_DISTANCE)
        self.gap_finder = GapFinder(GAP_SELECTION, POINT_SELECTION, MIN_GAP_SIZE, MIN_GAP_DISTANCE, CORNERING_DISTANCE)
        
        
        # count for frames w/o gaps
        self.no_gap_detected = 0
    
    
        # count for frames w/o gaps
        self.no_gap_detected = 0
        
        self.ranges = None
        self.prev_speed = 0
    
    def lidar_callback(self, data):
        """
        callback function to process LIDAR data
        - identify disparities and applies safety measures
        - search for the best gap to navigate
        - publish steering and velocity commands based on the identified gap
        """
        # preprocess LIDAR data by converting LIDAR data to numpy array and handle NaNs (replace NaNs w/ max range)
        # data.ranges = [i + 0.25 for i in data.ranges]
        ranges = np.array(data.ranges)
        # TODO: We can also try linearly interpolating NaNs instead of setting to max distance
        ranges = np.where(np.isnan(ranges), MAX_LIDAR_DISTANCE, ranges)
        ranges[ranges < 0.05] = MAX_LIDAR_DISTANCE
        # ranges = np.maximum(ranges+0.5,0)

        # disparity extension
        ranges = self.disparity_extender.extend_disparities(ranges, data.angle_increment)
        self.ranges = ranges

        # find largest gap and select the best point from that gap
        try:
            self.gap_finder.update_data(ranges, data)
            start_i, end_i = self.gap_finder.get_gap()
            best_point = self.gap_finder.get_point(start_i, end_i)
            # best_point = self.gap_finder.get_point_to_go_to(data) # This does the same as above, but in one line

            # calculate steering angle towards best point
            steering_angle = self.get_steering_angle(best_point, data)
            speed = self.dynamic_velocity(steering_angle)

            # publish
            self.publish_drive(speed, steering_angle)
            self.publish_gap_points(data, start_i, end_i)
        except:
            self.publish_drive(0, 0)
        self.publish_lidar(data, ranges)

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
        
        # set speed based on steering angle
        # commenting out for now 
        # #command.speed = self.dynamic_velocity(steering_angle)

        # set speed for no gaps dtected for car to slow down or to defult
        command.speed = min(self.prev_speed + 500, speed)
        self.prev_speed = command.speed

        # publish drive command
        self.drive_pub.publish(command)

    def dynamic_velocity(self, steering_angle):
        """
        adjust speed based on the distance to the nearest obstacle
        """
        # return DEFAULT_SPEED

        # if turn is sharp (> 30 degrees) then slow down, if not keep default speed
        # if abs(steering_angle) > 50:
        #     return 15.0
        # #    return TURN_SPEED  # if the turn are too sharp
        # else:
        #     return MAXIMUM_SPEED

        # uhhh attempt to slow speed graudally but not actually sure if it'l work
        # # calculate absolute value of the steering angle to determine severity of the turn
        abs_angle = abs(steering_angle)

        # # gradually slow speed as the steering angle increases (sharp turn)
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

    def publish_lidar(self, data, ranges):
        """
        publish LIDAR data as PointCloud2, add color to borders of gap
        """
        # Setup header for PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = data.header.frame_id
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        angle = data.angle_min
        points = []
        for r, r_ext in zip(data.ranges, ranges):
            if r == r_ext:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, 0XFFFFFF]  # white
                points.append(point)
            else:  # this point was disparity extended
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0

                point = [x, y, z, 0XFFFFFF]  # white
                points.append(point)

                x_ext = r_ext * math.cos(angle)
                y_ext = r_ext * math.sin(angle)
                z_ext = 0

                point_ext = [x_ext, y_ext, z_ext, 0XFF0000]  # red
                points.append(point_ext)

            angle += data.angle_increment

        # create PointCloud2 message
        pc2_msg = point_cloud2.create_cloud(header, fields, points)

        # resize = 200
        # pc2_msg.height = resize
        # pc2_msg.width = resize
        # pc2_msg.row_step = resize * pc2_msg.point_step

        # new_size = pc2_msg * resize * pc2_msg.point_step
        # pc2_msg.data = bytearray(new_size)
        self.lidar_pub.publish(pc2_msg)

    def publish_gap_points(self, data, start_i, end_i):
        # add all laser scan points, but set start_i and end_i to red (edge of gap)
        r1 = data.ranges[start_i]
        angle = data.angle_min + start_i * data.angle_increment
        x1 = r1 * math.cos(angle)
        y1 = r1 * math.sin(angle)
        z1 = 0
        m1 = self.generate_marker(x1, y1, z1, [0.0, 1.0, 0.0, 1.0])
        self.marker1_pub.publish(m1)

        r2 = data.ranges[end_i]
        angle = data.angle_min + end_i * data.angle_increment
        x2 = r2 * math.cos(angle)
        y2 = r2 * math.sin(angle)
        z2 = 0
        m2 = self.generate_marker(x2, y2, z2, [1.0, 0.0, 0.0, 1.0])
        self.marker2_pub.publish(m2)

    def generate_marker(self, x, y, z, color):
        marker = Marker()
        marker.header.frame_id = "car_8_laser"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker


if __name__ == '__main__':
    """
    init ros node and keep running 
    """
    rospy.init_node('follow_gap', anonymous=True)
    FTGControl()
    GetSectors()
    rospy.spin()
