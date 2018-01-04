#!/usr/bin/env python


import rospy
from std_msgs import msg
import time


def talker():
    is_run = msg.Int8(1)
    pub = rospy.Publisher('talker', msg.Int8, queue_size=10)
    rospy.init_node('is_run', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(is_run)
        pub.publish(is_run)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
