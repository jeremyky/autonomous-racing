MAX_LIDAR_DISTANCE = 2.5
STEERING_MULTIPLIER = 3.3
MINIMUM_SPEED = 30
MAXIMUM_SPEED = 80

DISPARITY_DISTANCE = 0.3
SAFETY_EXTENSION = 25

GAP_SELECTION = "best" # least_steering, widest, deepest, largest_integral
POINT_SELECTION = "best" # lear_steering, middle, deepest
MIN_GAP_SIZE = 10
MIN_GAP_DISTANCE = 1
CORNERING_DISTANCE = 0.28


import rospy
from std_msgs.msg import Int32

class Sectors:
    FREE = 0
    MID = 1
    DANGER = 2


def sector_callback(msg):
    global MAXIMUM_SPEED
    sector = msg.data
    if sector == Sectors.FREE:
        MAXIMUM_SPEED = 80
    elif sector == Sectors.MID:
        MAXIMUM_SPEED = 70
    elif sector == Sectors.DANGER:
        MAXIMUM_SPEED = 30

rospy.Subscriber('/car_8/sector', Int32, sector_callback)
