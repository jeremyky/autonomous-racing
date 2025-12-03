#!/usr/bin/env python

import rospy
from overtaker_control import OvertakerControl
from drive_mode_selector import DriveModeSelector
from get_sectors import GetSectors
from overtaker_config import *

if __name__ == "__main__":
    try:
        if MODE == MODES[0]:
            rospy.init_node("overtaker", anonymous=True)
            overtaker = OvertakerControl()
            sector = GetSectors()
            rospy.spin()
        elif MODE == MODES[1]:
            rospy.init_node("overtaker", anonymous=True)
            overtaker = OvertakerControl()
            dms = DriveModeSelector()
            sector = GetSectors()
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("An error occurred: {}", rospy.ROSInterruptException)
        pass
