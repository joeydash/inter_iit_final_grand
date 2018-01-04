#!/usr/bin/env python
import cv2
from threading import Thread
import numpy as np
import rospy
from std_msgs import msg

Is_run = True


def callback(msg):
    # global Is_run
    # Is_run = data.is_run
    rospy.loginfo(msg.data)


def get_xy(cap, lower_color, upper_color):
    point_to_displace_in_px = np.zeros(2, dtype=np.int16)
    _, frame = cap.read()
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv, (15, 15), 0)
    mask = cv2.inRange(blur, lower_color, upper_color)
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


def main():
    cam_src_number = 0
    lower_color = np.array([20, 45, 80])
    upper_color = np.array([46, 130, 220])
    cap = cv2.VideoCapture(cam_src_number)
    rospy.init_node('target_xy', anonymous=True)
    r = rospy.Rate(1)
    pub = rospy.Publisher('target_xy_corr', msg.Int16MultiArray, queue_size=25)
    rospy.Subscriber("is_run", msg.Bool, callback)
    rospy.spin()
    while not rospy.is_shutdown():
        if Is_run:
            tmp = get_xy(cap, lower_color, upper_color)
            xy_correction = msg.Int16MultiArray(data=tmp)
            # rospy.loginfo(xy_correction)
            pub.publish(xy_correction)
        else:
            cap.release()
            break
        r.sleep()
    while rospy.is_shutdown():
        cap.release()
        exit()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
