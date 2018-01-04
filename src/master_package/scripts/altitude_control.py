from __future__ import print_function
import math

from dronekit import  VehicleMode, LocationGlobal
import time


def get_alt(prev_val_read, delta_t, curr_alt,vehicle):
    
    curr_val_read = vehicle.location.global_relative_frame.alt
    
    if curr_val_read == prev_val_read:
        curr_alt = curr_alt + vehicle.velocity[2] * (delta_t)
    # correction: if feddback is recieved
    else:
        curr_alt = vehicle.location.global_relative_frame.alt
        prev_val_read = curr_alt

    return [curr_alt, prev_val_read]


# -----------------------------------------------------------------------#
# -----------call this function to control altitude----------------------#
# -----------------------------------------------------------------------#

def altitude_control(target_altitude, kp_alt,vehicle):
    vehicle.mode.name == "LOITER"

    while not vehicle.mode.name == "LOITER":
        pass

    # initailising variables
    prev_val_read = vehicle.location.global_relative_frame.alt
    prev_time = time.time()
    then = time.time()
    curr_alt = vehicle.location.global_relative_frame.alt

    while True:

        curr_time = time.time()
        # prediction: updating curr_alt with velocity*dt if alt feedback is not updated
        alt_data = get_alt(prev_val_read, curr_time - prev_time, curr_alt,vehicle)

        # soring values for next loop
        prev_time = curr_time
        curr_alt = alt_data[0]
        prev_val_read = alt_data[1]

        # calculating error
        error = target_altitude - curr_alt  # in meters
        # correction
        vel_corr = kp_alt * error  # in meters

        # constraining velocity within -1 to 1
        if vel_corr > 1:
            vel_corr = 1
        elif vel_corr < -1:
            vel_corr = -1

        # mapping velocity to throttle
        if vel_corr > 0:
            thr = 1590 + vel_corr * 400
        elif vel_corr < 0:
            thr = 1410 + vel_corr * 400
        else:
            thr = 1500

        print(thr, )
        print(curr_alt)
        vehicle.channels.overrides['3'] = thr

        now = time.time()

        # break loop if it holds altitude for more than 5 secs
        if target_altitude + 0.2 > vehicle.location.global_relative_frame.alt > target_altitude - 0.2:
            if now - then > 5:
                break
        else:
            then = now
