#!/usr/bin/env python
from __future__ import print_function
import os
import time
import math
import rospy
from std_msgs import msg

from dronekit import connect, VehicleMode
from altitude_control import altitude_control, get_alt
from xy_control import xy_position_control

take_off_alt = 3
kp_alt = .5
kp_velx = .5
kp_vely = .5
is_run = True
# connection_string = '/dev/ttyACM0'
connection_string = "tcp:127.0.0.1:5763"

# Connect to the Vehicle..
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
# initialising variables
curr_alt = vehicle.location.global_relative_frame.alt
prev_val_read = vehicle.location.global_relative_frame.alt
prev_time = time.time()
then = time.time()
start_yaw = vehicle.attitude.yaw


def target_set_xy(message):
    global vehicle, is_run, then, curr_alt, prev_val_read, prev_time, start_yaw
    # initialising variables
    curr_alt = vehicle.location.global_relative_frame.alt
    prev_val_read = vehicle.location.global_relative_frame.alt
    prev_time = time.time()
    then = time.time()

    print("callback started")
    curr_time = time.time()
    print(curr_alt)
    alt_data = get_alt(prev_val_read, curr_time - prev_time, curr_alt, vehicle)
    prev_val_read = alt_data[1]
    prev_time = curr_time

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message)
    camera_constant = alt_data[0] / 557
    mav_msg = xy_position_control(message[1] * camera_constant, message[0] * camera_constant, kp_velx, kp_vely,
                                  start_yaw,
                                  vehicle)

    vehicle.send_mavlink(mav_msg)
    now = time.time()
    radius = math.sqrt(message.data[0] ** 2 + message.data[1] ** 2)
    # break loop if it holds altitude for more than 5 secs
    if radius < 5:
        if now - then > 5:
            pub = rospy.Publisher('master/is_run', msg.Bool, queue_size=25)
            is_run_obj = msg.Bool(data=is_run)
            pub.publish(is_run_obj)

    else:
        then = now


def main():
    global vehicle
    print("Basic pre-arm checks")
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(5)
    print(vehicle.mode.name)
    print("Arming motors")
    vehicle.armed = True
    time.sleep(5)
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    altitude_control(take_off_alt, kp_alt, vehicle)
    vehicle.channels.overrides['3'] = 1500
    print(vehicle.location.global_relative_frame.alt)
    print("holding")
    time.sleep(5)

    vehicle.mode = VehicleMode("GUIDED")

    os.spawnl(os.P_NOWAIT, "rosrun master_package get_target_xy.py")

    rospy.init_node('main_node', anonymous=True)
    while is_run:
        print("inside is_run")
        rospy.Subscriber("get_target_xy/target_xy", msg.Int16MultiArray, target_set_xy)
    altitude_control(3, kp_alt, vehicle)

    # while True:
    # run atharva's script, returns y and theta
    # allign yaw

    vehicle.mode = VehicleMode("LAND")
    vehicle.close()

    print("Completed")


if __name__ == "__main__":
    main()
