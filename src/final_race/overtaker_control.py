import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String, Float32

from multi_pp_control import MultiPPControl
from ftg_control_overtake import FTGControl
from overtaker_config import *

class OvertakerControl:
    def __init__(self):
        self.drive_mode = DriveMode.PP
        rospy.Subscriber('/{}/drive_mode'.format(CAR_NAME), Int32, self.set_drive_mode)

        self.racelines = RACELINES
        rospy.Subscriber('/{}/select_raceline'.format(CAR_NAME), String, self.set_raceline)
        
        self.command_pub = rospy.Publisher('/{}/offboard/command'.format(CAR_NAME), AckermannDrive, queue_size=1)

        self.pp_control = MultiPPControl()
        self.init_pp()

        self.ftg_control = FTGControl()
        self.init_ftg()

        self.current_speed_mult = 1.0
        self.target_speed_mult = 1.0

        rospy.Subscriber('/{}/speed_mult'.format(CAR_NAME), Float32, self.speed_mult_callback)

    def set_raceline(self, raceline):
        self.pp_control.select_raceline(raceline.data)

    def set_drive_mode(self, msg):
        self.drive_mode = msg.data

    def speed_mult_callback(self, msg):
        self.target_speed_mult = msg.data

    def init_pp(self):
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(CAR_NAME), PoseStamped,
                         self.mpp_control)

    def mpp_control(self, data):
        if self.drive_mode == DriveMode.PP:
            command = self.pp_control.pp_control(data)
            steering_angle = command.steering_angle
            speed = command.speed

            self.target_speed = speed
            self.publish_command(steering_angle)
            # print("steering:", steering_angle, "speed:", speed)

    def init_ftg(self):
        rospy.Subscriber("/car_8/scan", LaserScan, self.ftg_control_callback)

    def ftg_control_callback(self, data):
        if self.drive_mode == DriveMode.FTG:
            command = self.ftg_control.ftg_control(data)
            steering_angle = command.steering_angle
            speed = command.speed

            self.target_speed = speed
            self.publish_command(steering_angle)


    def get_speed(self):
        if self.target_speed_mult < self.current_speed_mult:
            self.current_speed_mult = max(self.target_speed_mult, self.current_speed_mult - 0.1)
        else:
            self.current_speed_mult = self.target_speed_mult
        
        print("speed_mult:", self.current_speed_mult)
        return self.target_speed * self.current_speed_mult

    def publish_command(self, steering_angle):
        command = AckermannDrive()
        command.steering_angle = steering_angle
        command.speed = self.get_speed()
        self.command_pub.publish(command)
