from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, APIException
from pymavlink import mavutil
from simple_pid import PID
import numpy as np
import threading
import argparse
import socket
import serial
import math
import time
import cv2
import sys



def connect_drone():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()
    print('Connecting to vehicle')
    vehicle = connect(args.connect, baud=57600, wait_ready=True)
    return vehicle


def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED mode")
        time.sleep(1)
    print("Vehicle now in GUIDED mode")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed")
        time.sleep(1)
    print("virtual props are spinning")

    ############ TAKEOFF ###################
    vehicle.simple_takeoff(targetHeight)  ##meters

    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached")
    return None


def calibrate_gyro():
    """Request gyroscope calibration."""

    calibration_command = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
        0,  # confirmation
        1,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
        0,  # param 2, 1: magnetometer calibration
        0,  # param 3, 1: ground pressure calibration
        0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
        0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
        0,  # param 6, 2: airspeed calibration
        0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
    )
    vehicle.send_mavlink(calibration_command)


def calibrate_magnetometer():
    """Request magnetometer calibration."""

    # APM requires the MAV_CMD_DO_START_MAG_CAL command, only present in the APM MAVLink dialect


    if vehicle._autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        calibration_command = vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,  # command
            0,  # confirmation
            0,  # param 1, uint8_t bitmask of magnetometers (0 means all).
            1,  # param 2, Automatically retry on failure (0=no retry, 1=retry).
            1,  # param 3, Save without user input (0=require input, 1=autosave).
            0,  # param 4, Delay (seconds).
            1,  # param 5, Autoreboot (0=user reboot, 1=autoreboot).
            0,  # param 6, Empty.
            0,  # param 7, Empty.
        )
    else:
        calibration_command = vehicle.message_factory.command_long_encode(
            0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
            0,  # confirmation
            0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
            1,  # param 2, 1: magnetometer calibration
            0,  # param 3, 1: ground pressure calibration
            0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
            0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0,  # param 6, 2: airspeed calibration
            0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
        )
    vehicle.send_mavlink(calibration_command)


def calibrate_accelerometer():
    """Request accelerometer calibration."""

    calibration_command = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
        0,  # confirmation
        0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
        0,  # param 2, 1: magnetometer calibration
        0,  # param 3, 1: ground pressure calibration
        0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
        1,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
        0,  # param 6, 2: airspeed calibration
        0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
    )
    vehicle.send_mavlink(calibration_command)


def calibrate_accelerometer_simple():
    """Request simple accelerometer calibration."""

    calibration_command = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
        0,  # confirmation
        0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
        0,  # param 2, 1: magnetometer calibration
        0,  # param 3, 1: ground pressure calibration
        0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
        4,
        # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
        0,  # param 6, 2: airspeed calibration
        0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
    )
    vehicle.send_mavlink(calibration_command)


def calibrate_board_level():
    """Request board level calibration."""

    calibration_command = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
        0,  # confirmation
        0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
        0,  # param 2, 1: magnetometer calibration
        0,  # param 3, 1: ground pressure calibration
        0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
        2,
        # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
        0,  # param 6, 2: airspeed calibration
        0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
    )
    vehicle.send_mavlink(calibration_command)


def calibrate_barometer():
    """Request barometer calibration."""

    calibration_command = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,  # command
        0,  # confirmation
        0,  # param 1, 1: gyro calibration, 3: gyro temperature calibration
        0,  # param 2, 1: magnetometer calibration
        1,  # param 3, 1: ground pressure calibration
        0,  # param 4, 1: radio RC calibration, 2: RC trim calibration
        0,  # param 5, 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
        0,  # param 6, 2: airspeed calibration
        0,  # param 7, 1: ESC calibration, 3: barometer temperature calibration
    )
    vehicle.send_mavlink(calibration_command)


def disarm():
    ## Completely disarms vehicle regardless of status ##

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)


def set_servo(channel, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)


def set_speed():
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        0,
        3.5,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)


def set_roi(lat_x, lon_y, alt):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_NAV_ROI,
        0,
        1,
        0,
        0,
        0,
        lat_x,
        lon_y,
        alt,
    )
    vehicle.send_mavlink(msg)


def set_velocity_body(Vx, Vy, Vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # --Position
        Vx, Vy, Vz,  # --Velocity
        0, 0, 0,  # --Accelerations
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def follow_me():
    ########### NOT DONE ##################
    try:
        # Use the python gps package to access the laptop GPS
        gpsd = gps.gps(mode=gps.WATCH_ENABLE)

        # Arm and take off to altitude of 5 meters
        arm_and_takeoff(5)

        while True:

            if vehicle.mode.name != "GUIDED":
                print("User has changed flight modes - aborting follow-me")
                break

                # Read the GPS state from the laptop
            next(gpsd)

            # Once we have a valid location (see gpsd documentation) we can start moving our vehicle around
            if (gpsd.valid & gps.LATLON_SET) != 0:
                altitude = 30  # in meters
                dest = LocationGlobalRelative(gpsd.fix.latitude, gpsd.fix.longitude, altitude)
                print("Going to: %s" % dest)

                # A better implementation would only send new waypoints if the position had changed significantly
                vehicle.simple_goto(dest)

                # Send a new target every two seconds
                # For a complete implementation of follow me you'd want adjust this delay
                time.sleep(2)

    except socket.error:
        print("Error: gpsd service does not seem to be running, plug in USB GPS or run run-fake-gps.sh")
        sys.exit(1)


def get_distance_meters(targetLocation,currentLocation):
    dLat=targetLocation.lat - currentLocation.lat
    dLon=targetLocation.lon - currentLocation.lon

    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5


def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

    vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED":
#        if keyboard.is_pressed('q'):
#            keyboard.send('ctrl+u')

#            ManualMode()
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

        if currentDistance<distanceToTargetLocation*.03:
            print("Reached target waypoint")

            time.sleep(2)
            break

        time.sleep(1)
    return None


def camera():
    try:
        while True:
            ret, frame = cap.read()
            depth_colormap = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
            # Wait for a coherent pair of frames: depth and color

            #        color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            # depth_image = np.asanyarray(depth_frame.get_data())
            #        color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.15), cv2.COLORMAP_JET)

            # image = depth_colormap[0:480, 20:620]
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            l_blue = np.array([70, 20, 20])
            h_blue = np.array([150, 255, 255])
            mask = cv2.inRange(image, l_blue, h_blue)
            countblue = cv2.countNonZero(mask)
            if countblue > 10000:

                coordinates = cv2.findNonZero(mask)
                centroid = np.mean(coordinates, axis=0)
                la, lb = map(list, zip(*centroid))
                l1 = [round(x) for x in la]
                l2 = [round(x) for x in lb]
                te1 = np.array(l1)
                te2 = np.array(l2)

                cv2.circle(depth_colormap, (te1, te2), 10, (255, 255, 255), -1)
                cv2.line(img=depth_colormap, pt1=(te1, te2), pt2=(320, 240), color=(0, 255, 0), thickness=5, lineType=8,
                         shift=0)

                double_angle = (math.atan2(240 - te2, 320 - te1) * 180 / 3.14)
                x = math.floor(double_angle)
                if x < 0:
                    x = 360 + x

                cv2.putText(depth_colormap, str(x), (200, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            #	if (depth_colormap>= l_blue).all() and (depth_colormap<=h_blue).all():
            #		cv2.putText(images, "BLUUEEE", (200, 300),
            #		cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            #            cv2.line(img=depth_colormap, pt1=(0, 100), pt2=(640, 100), color=(0, 255, 0), thickness=5, lineType=8, shift=0)
            #            cv2.line(img=depth_colormap, pt1=(0, 380), pt2=(640, 380), color=(0, 255,0), thickness=5, lineType=8, shift=0)
            #            cv2.line(img=depth_colormap, pt1=(150, 0), pt2=(150, 480), color=(0, 255, 0), thickness=5, lineType=8, shift=0)
            #            cv2.line(img=depth_colormap, pt1=(490, 0), pt2=(490, 480), color=(0, 255, 0), thickness=5, lineType=8, shift=0)
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', depth_colormap)
            cv2.waitKey(1)


    finally:

        # Stop streaming
        cap.release()
        cv2.destroyAllWindows()


def goto_camera():
    thread1 = threading.Thread(target=camera)
    thread1.start()

    thread2 = threading.Thread(target=goto(wp1))
    thread2.start()


def stereo_cam():
    while True:
        ret, frame = cap.read()
        depth_colormap = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_blue = np.array([70, 20, 20])
        h_blue = np.array([150, 255, 255])
        mask = cv2.inRange(image, l_blue, h_blue)
        countblue = cv2.countNonZero(mask)
        if countblue > 15000:
            coordinates = cv2.findNonZero(mask)
            centroid = np.mean(coordinates, axis=0)
            la, lb = map(list, zip(*centroid))
            l1 = [round(x) for x in la]
            l2 = [round(x) for x in lb]
            n1 = np.array(l1)
            n2 = np.array(l2)
            cv2.circle(depth_colormap, (n1, n2), 10, (255, 255, 255), -1)
            cv2.line(img=depth_colormap, pt1=(n1, n2), pt2=(320, 240), color=(0, 255, 0), thickness=5, lineType=8,
                     shift=0)
            double_angle = (math.atan2(240 - n2, 320 - n1) * 180 / 3.14)
            x = math.floor(double_angle)
            if x < 0:
                x = 360 + x
            cv2.putText(depth_colormap, str(x), (200, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if n1 > 280:
                set_velocity_body(0, 0, 0)
                condition_yaw(1, True)
            if n1 < 360:
                set_velocity_body(0, 0, 0)
                condition_yaw(359, True)
            if time.time() > start + PERIOD_OF_TIME:
                vehicle.mode = VehicleMode("LAND")
                print("LANDING")
            else:
                goto(wp1)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', depth_colormap)
        cv2.waitKey(1)


def stereo_gps():
    while True:
        while True:
            ret, frame = cap.read()
            depth_colormap = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            l_blue = np.array([70, 20, 20])
            h_blue = np.array([150, 255, 255])
            mask = cv2.inRange(image, l_blue, h_blue)
            countblue = cv2.countNonZero(mask)
            if countblue > 20000:
                coordinates = cv2.findNonZero(mask)
                centroid = np.mean(coordinates, axis=0)
                la, lb = map(list, zip(*centroid))
                l1 = [round(x) for x in la]
                l2 = [round(x) for x in lb]
                n1 = np.array(l1)
                n2 = np.array(l2)
                cv2.circle(depth_colormap, (n1, n2), 10, (255, 255, 255), -1)
                cv2.line(img=depth_colormap, pt1=(n1, n2), pt2=(320, 240), color=(0, 255, 0), thickness=5, lineType=8,
                         shift=0)
                double_angle = (math.atan2(240 - n2, 320 - n1) * 180 / 3.14)
                x = math.floor(double_angle)
                if x < 0:
                    x = 360 + x
                cv2.putText(depth_colormap, str(x), (200, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if n1 > 280:
                    set_velocity_body(0, 0, 0)
                    condition_yaw(1, True)
                if n1 < 360:
                    set_velocity_body(0, 0, 0)
                    condition_yaw(359, True)
                #            if time.time() > start + PERIOD_OF_TIME:
                #                vehicle.mode = VehicleMode("LAND")
                #                print("LANDING")
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', depth_colormap)
                cv2.waitKey(1)
            if countblue < 20000:
                break
        while True:
            ret, frame = cap.read()
            depth_colormap = cv2.resize(frame, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            l_blue = np.array([70, 20, 20])
            h_blue = np.array([150, 255, 255])
            mask = cv2.inRange(image, l_blue, h_blue)
            countblue = cv2.countNonZero(mask)
            print("Set default/target airspeed to 3")
            vehicle.airspeed = 6

            print("Going towards first point for 30 seconds ...")
            wp1 = LocationGlobalRelative(34.44489, -119.66219, 10)
            vehicle.simple_goto(wp1)
            time.sleep(.1)
            if countblue > 20000:
                break


def sonar():
    serial_data = serial.Serial('/dev/ttyACM0', 9600)
    while True:

        dist = serial_data.readline()
        s1 = dist[0:1]

        s2 = dist[1:2]

        s3 = dist[2:3]

        s4 = dist[3:4]
        ###################### X AXIS #########################
        if s1 == "T" and s2 == "T" and s3 == "T":
            set_velocity_body(1, 0, 0)
        elif s1 == "T" and s2 == "T":
            set_velocity_body(0, 0, 0)
        elif s1 == "T" and s3 == "T":
            set_velocity_body(1, 1, 0)
        elif s2 == "T" and s3 == "T":
            set_velocity_body(1, -1, 0)
        elif s1 == "T":
            set_velocity_body(0, 1, 0)
        elif s2 == "T":
            set_velocity_body(0, -1, 0)
        elif s3 == "T":
            set_velocity_body(1, 0, 0)
        else:
            set_velocity_body(0, 0, 0)


def go_forward():
    set_velocity_body(2,0,0)

def go_backward():
    set_velocity_body(-2,0,0)

def go_right():
    set_velocity_body(0,2,0)

def go_left():
    set_velocity_body(0,-2,0)

def go_up():
    set_velocity_body(0,0,-1)

def go_down():
    set_velocity_body(0,0,1)

def yaw_left():
    condition_yaw(350, relative=True)

def yaw_right():
    condition_yaw(10, relative=True)

def Set_Gimbal():
    Pitch = int(entry4.get())
    Roll = int(entry3.get())
    Yaw = int(entry5.get())
    vehicle.gimbal.rotate(Pitch, Roll, Yaw)


def Return_To_Launch():
    vehicle.mode = VehicleMode("RTL")
    while vehicle.mode != 'RTL':
        print("Waiting for drone to enter RTL mode")
        time.sleep(1)
    print("Vehicle in RTL mode")


def Land_Here():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode != 'LAND':
        print("Waiting for drone to enter LAND mode")
        time.sleep(1)
    print("Vehicle in LAND mode")


def time_of_flight(x):
    start = time.time()
    PERIOD_OF_TIME = x
    if time.time() > start + PERIOD_OF_TIME:
        vehicle.mode = VehicleMode("LAND")
        print("LANDING")
    return


def pid():
    pidx = PID(0.01, 0.001, 0.002, setpoint=320)
    pidy = PID(0.01, 0.001, 0.002, setpoint=240)

    pidx.output_limits = (-10, 10)
    pidy.output_limits = (-10, 10)

    pidx.sample_time = 1
    pidy.smaple_time = 1
    while True:
        ret, frame = cap.read()
        img1 = frame
        image = frame
        l_red1 = np.array([92, 120, 70])
        h_red1 = np.array([110, 255, 255])

        l_red = np.array([93, 120, 70])
        h_red = np.array([110, 255, 225])

        l_red_rgb = np.array([70, 70, 160])
        h_red_rgb = np.array([100, 100, 255])

        frame = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(frame, l_red1, h_red1)
        mask2 = cv2.inRange(frame, l_red, h_red)
        mask3 = cv2.inRange(frame, l_red_rgb, h_red_rgb)
        mask1 = mask1 + mask2 + mask3
        masked_frame = cv2.bitwise_and(image, image, mask=mask1)
        coordinates = cv2.findNonZero(mask1)
        countblue = cv2.countNonZero(mask1)
        if countblue > 1500:
            centroid = np.mean(coordinates, axis=0)
            la, lb = map(list, zip(*centroid))
            l1 = [round(x) for x in la]
            l2 = [round(x) for x in lb]
            n1 = np.array(l1)
            n2 = np.array(l2)
            cv2.line(img=image, pt1=(n1, n2), pt2=(320, 240), color=(0, 255, 0), thickness=5, lineType=8, shift=0)
            cv2.circle(image, (n1, n2), 10, (255, 255, 255), -1)
            x = pidx(n1) / 5
            y = pidy(n2) / 5
            if x > .8:
                x = .4
            if y > .8:
                y = .4
            set_velocity_body(y, x, 0)
            print(x, y)
            if x < .1 and y < .2:
                set_velocity_body(y, x, .2)
            if vehicle.location.global_relative_frame.alt < 2:
                vehicle.mode = VehicleMode("LAND")
                while True:
                    print("Landing")
                    time.sleep(2)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', masked_frame)
        cv2.waitKey(1)



vehicle = connect_drone()



#print("\nPrint all parameters (iterate `vehicle.parameters`):")
#for key, value in vehicle.parameters.iteritems():
#    fk = ''.join(str(x) for x in key)



skeys = str(vehicle.parameters.keys())
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('192.168.1.219', 1236))
s.listen(5)

while True:
    clientsocket, address = s.accept()
    print("Connection from {address} has been established")
    skeys = str(vehicle.parameters.keys())
    svalues = str(vehicle.parameters.values())
    kstring = ("parameters" + skeys + '\n' + '\n').encode('utf-8')
    sstring = ("values" + svalues + '\n' + '\n').encode('utf-8')
    msg = clientsocket.recv(1024)
    x = msg.decode("utf-8")
    print(x)
    if x == "parameters":
        clientsocket.send(bytes(kstring))
    if x == "values":
        clientsocket.send(bytes(sstring))
    if "action" in x:
        print(x)
        splitting = x.split()
        param_name = splitting[1]
        param_value = splitting[2]
        int_value = float(param_value)
        print("vehicle.parameters['%s']=%s" % (param_name, int_value))
        print(param_name)
        print(int_value)
        time.sleep(5)
        vehicle.parameters[param_name] = int_value
