#!/usr/bin/env python


import rospy
from std_msgs import msg


def callback(data):
    # data = msg.Int16MultiArray()
    # data.layout.dim = 2
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)


def listener():
    data = msg.Bool(data=True)
    pub = rospy.Publisher('chatter', msg.Bool, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(data)
        rospy.Subscriber('chat', msg.Bool, callback)
        rate.sleep()


if __name__ == '__main__':
    listener()
