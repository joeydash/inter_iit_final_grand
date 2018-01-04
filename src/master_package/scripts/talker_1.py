#!/usr/bin/env python
import rospy
from std_msgs import msg
import time


def listener():
    data = msg.Bool(data=False)
    pub = rospy.Publisher('master/is_run', msg.Bool, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    time.sleep(5)
    pub.publish(data)
    rospy.loginfo("published!")
    exit()
    rate.sleep()


if __name__ == '__main__':
    listener()
