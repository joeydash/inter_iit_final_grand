#!/usr/bin/env python
from __future__ import print_function
from os import system
import sys
import rospy
from std_msgs import msg

from dronekit import connect, VehicleMode
from altitude_control import control_main, altitude_control
from xy_control import xy_position_control

kp_alt = .5
kp_velx = .5
kp_vely = .5

# initialising variables
curr_alt = vehicle.location.global_relative_frame.alt
prev_val_read = vehicle.location.global_relative_frame.alt
prev_time = time.time()
then = time.time()
flag = 0


def callback(data):
    global curr_alt, prev_val_read, prev_time, then, flag
    print("callback started")
    curr_time = time.time()
    print(curr_alt)
    alt_data = get_alt(prev_val_read, curr_time - prev_time, curr_alt, vehicle)
    prev_val_read = alt_data[1]
    prev_time = curr_time

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    camera_constant = alt_data[0] / 557
    msg = xy_position_control(data[1] * camera_constant, data[0] * camera_constant, kp_velx, kp_vely, curr_yaw, vehicle)

    vehicle.send_mavlink(msg)
    now = time.time()
    radius = math.sqrt(error_xy[0] ** 2 + error_xy[1] ** 2)
    # break loop if it holds altitude for more than 5 secs
    if radius + 5 > radius > radius - 5:
        if now - then > 5:
            pub = rospy.Publisher('master_node/is_run', _Bool, queue_size=25)
            is_run_obj = False
            pub.publish(is_run_obj)
            flag = 1
    else:
        then = now


# connection_string = '/dev/ttyACM0'
connection_string = "tcp:127.0.0.1:5763"

# Connect to the Vehicle..
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

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
altitude_control(3, kp_alt, vehicle)
vehicle.channels.overrides['3'] = 1500
print(vehicle.location.global_relative_frame.alt)
print("holding")
time.sleep(5)

vehicle.mode = VehicleMode("GUIDED")

system("python ../../landing_target/src/landing_target.py")

rospy.init_node('master_node', anonymous=True)
rospy.Subscriber("landing_target/xy_pos", numpy_msg(_Int16), callback)
rospy.spin()

while (flag == 0):
    time.sleep(1)

altitude_control(3, kp_alt, vehicle)

# while True:
# run atharva's script, returns y and theta
# allign yaw

vehicle.mode = VehicleMode("LAND")
vehicle.close()

print("Completed")
