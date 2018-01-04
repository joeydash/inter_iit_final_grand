#!/usr/bin/env python

import cv2, numpy as np
import rospy
from std_msgs.msg import _Int16, _Bool
from rospy.numpy_msg import numpy_msg

# xy_correction = _Int8MultiArray
# xy_correction.layout.dim.size = 2
is_run = True
lower_color = np.array([20, 45, 80])
upper_color = np.array([46, 130, 220])


def change_is_run(msg):
    global is_run
    if not msg:
        is_run = False
    else:
        pass


def get_xy(cap, lower_colour, upper_colour):
    point_to_displace_in_px = np.zeros(2, dtype=np.int8)
    _, frame = cap.read()
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv, (15, 15), 0)
    mask = cv2.inRange(blur, lower_colour, upper_colour)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours):
        filter(lambda a: cv2.contourArea(a) > 500, contours)
        c = sorted(contours, key=cv2.contourArea)[::-1]
        for contour in c[0:1:]:
            x, y, w, h = cv2.boundingRect(contour)
            x_displacement_in_pixels = ((2 * x + w) / 2) - (width / 2)
            y_displacement_in_pixels = (height / 2) - ((2 * y + h) / 2)
            point_to_displace_in_px = [x_displacement_in_pixels, y_displacement_in_pixels]
        return point_to_displace_in_px
    else:
        return point_to_displace_in_px


def send_target_xy(cap, publisher):
    global xy_correction
    if is_run:

        xy_correction = get_xy(cap, lower_color, upper_color)

        publisher.publish(xy_correction)
    else:
        pass


def main():
    cap = cv2.VideoCapture(0)
    print "landing_target node running"
    rospy.init_node('target_xy', anonymous=True)
    rospy.Subscriber('master_node/is_run', _Bool, change_is_run)
    publisher = rospy.Publisher('target_xy/xy_pos', numpy_msg(_Int16), queue_size=25)
    timer = rospy.Timer(rospy.Duration(0.1), send_target_xy(cap, publisher))
    rospy.spin()
    timer.shutdown()
    cap.release()


if __name__ == '__main__':
    main()
