#!/usr/bin/env python3

import sys
import time
from datetime import datetime
import math
import os
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET

import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

import actionlib
from route_control.msg import StartMissionAction, StartMissionResult

import olympe
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged, moveToChanged, FlyingStateChanged, moveByChanged
from olympe.messages.ardrone3.Piloting import CancelMoveTo, CancelMoveBy
from olympe.messages.ardrone3.Piloting import moveTo, moveBy
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.common.Calibration import MagnetoCalibration
from olympe.messages.common.CalibrationState import MagnetoCalibrationRequiredState, MagnetoCalibrationStartedChanged
from olympe.enums.gimbal import control_mode, frame_of_reference
from olympe.messages.gimbal import set_target
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status
from olympe.enums.camera import  camera_mode
from olympe.messages.camera import set_photo_mode, take_photo, set_camera_mode, photo_progress, stop_photo, camera_mode as c_mode
from olympe.messages.rth import return_to_home
from olympe.media import download_media, indexing_state, delete_media, delete_all_media
from olympe.messages.user_storage import start_monitoring, stop_monitoring

from node import Node
from scan_area import ScanArea
from event_listener import EventListener
from controller import Controller
from helpers import check_connection
from exceptions import PauseException

# DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
# # Skycontroller
# # DRONE_IP = os.environ.get("DRONE_IP", "192.168.53.1")

# ANAFI_URL = "http://{}".format(DRONE_IP)

# ANAFI_MEDIA_API_URL = ANAFI_URL + "/api/v1/media/medias/"
# Skycontroller
# ANAFI_MEDIA_API_URL = ANAFI_URL + ":180/api/v1/media/medias/"


# def position_changed_callback(event, controller):
#     latitude = event.args["latitude"]
#     longitude = event.args["longitude"]
#     altitude = event.args["altitude"]
#     print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")


def handle_drone_connect(request):
    check_connection(drone)
    return [drone.connection_state(), "Connected..."]


def handle_drone_calibrate(request):
    controller.calibrate()
    return [drone.get_state(MagnetoCalibrationRequiredState), "Magnetometer Calibration Finished"]


def handle_drone_start_mission(data):
    last_visited_node = (data.x, data.y)
    # return
    global paused

    if not drone.connection_state():
        rospy.loginfo("Drone not connected...")
        return
    elif drone.get_state(MagnetoCalibrationRequiredState)['required'] == 1:
        rospy.logwarn("Magneto calibration required....")
        return

    print("Creating Route...")
    takeoff_coord = drone_gps.get_current_position()
    # takeoff_node = Node(takeoff_coord[1], takeoff_coord[0], takeoff_coord[2])

    print(str(takeoff_coord)
          + " Battery: " + str(drone.get_state(BatteryStateChanged)['percent']) + "%")

    polygon = ScanArea(vertices)
    route, rotated_route, vertical_increment = polygon.create_route(
        altitude, intersection_ratio, route_angle, rotated_route_angle)

    print("routes have been created")

    drone(start_monitoring(0))

    time.sleep(1)
    while not paused:
        try:

            print("setting camera...")
            print(drone.get_state(c_mode))
            expectation = drone(set_camera_mode(
                0, value=camera_mode.photo)).wait()
            expectation.explain()

            # if paused:
            #     print("Mission Paused")
            #     controller.set_pause(False)
            #     paused = False
            #     return [False, "Mission Paused"]

            controller.takeoff()

            if paused:
                print("Mission Paused")
                controller.set_pause(False)
                paused = False
                rospy.loginfo("Mission Paused")
                return

            drone(moveTo(takeoff_coord[0], takeoff_coord[1], takeoff_coord[2] + altitude, MoveTo_Orientation_mode.NONE, 0)).wait().success()
            time.sleep(2)

            # if paused:
            #     print("Mission Paused")
            #     controller.set_pause(False)
            #     paused = False
            #     return [False, "Mission Paused"]

            controller.set_gimbal_angle(gimbal_angle=gimbal_angle)

        except (Exception, PauseException) as e:
            print(repr(e))
            if "Paused" in str(e):
                print("Mission Paused")
                controller.set_pause(False)
                paused = False
                rospy.loginfo("Mission Paused")
                return

            print("Camera setting and takeoff exception: " + repr(e))
            check_connection(drone)
            continue
        else:
            break

    takeoff_date = datetime.now().isoformat()
    rospy.loginfo(
        "Takeoff date and time in ISO 8601 format: " + str(takeoff_date))
    time.sleep(5)

    try:
        # controller.setup_photo_gpslapse_mode(vertical_increment)

        alt = drone_gps.get_current_position()[2]-takeoff_coord[2]

        print("following the first route...")
        controller.follow_route(route=route, altitude=alt,
                                vertical_increment=vertical_increment, last_visited_node=last_visited_node)
        # if paused:
        #     print("Mission Paused")
        #     controller.set_pause(False)
        #     paused = False
        #     return [False, "Mission Paused"]

        expectation = drone(set_camera_mode(
                0, value=camera_mode.photo)).wait()
        assert expectation.success(), expectation.explain()

        print("following the second route...")
        controller.follow_route(route=rotated_route, altitude=alt,
                                vertical_increment=vertical_increment, last_visited_node=last_visited_node)

        # if paused:
        #     print("Mission Paused")
        #     controller.set_pause(False)
        #     paused = False
        #     return [False, "Mission Paused"]

        # Set gimbal to look upward while landing
        controller.set_gimbal_angle(gimbal_angle=90)

        # Return to take off point
        controller.move_to(takeoff_coord[0], takeoff_coord[1], alt, drone_gps)
        controller.land()

        land_date = datetime.now().isoformat()
        rospy.loginfo(
            "Land date and time in ISO 8601 format: " + str(land_date))

        print(str(drone_gps.get_current_position())
              + " Battery: " + str(drone.get_state(BatteryStateChanged)['percent']) + "%")
    except (Exception, PauseException) as e:
        print(repr(e))
        if "Paused" in str(e):
            print("Mission Paused")
            controller.set_pause(False)
            paused = False
            rospy.loginfo("Mission Paused")
            return

    drone(stop_monitoring())

    drone_gps.stop()
    drone.disconnect()
    event_listener.unsubscribe()

    print("Mission Finished: ")
    # mission_server.set_succeeded([True, "Mission Finished"])
    controller.set_pause(False)
    paused = False
    return [True, "Mission Finished"]


def handle_drone_land(request):
    controller.land()

    return [True, "Landing..."]


def handle_drone_rth(request):
    controller.rth()
    return [True, "Returning to Home..."]


def handle_drone_pause(request):
    global paused
    flying_state = str(drone.get_state(FlyingStateChanged)['state'])
    if "takingoff" in flying_state or "motor_ramping" in flying_state:
        controller.land()
    elif "landing" in flying_state:
        controller.takeoff()
    else:
        drone(CancelMoveTo()).wait().success()
        drone(stop_photo(cam_id=0))

    if not paused:
        controller.set_pause(True)
        paused = True
        return [True, "Paused."]
    
    controller.set_pause(False)
    paused = False
    return [True, "Resuming..."]


def handle_download_photos(request):
    dir_name = ""
    x = int(mission_id)
    while True:
        dir_name = ("project" + (str(x) if x != 0 else '') + "-" + str(math.degrees(
            route_angle)) + "-" + str(math.degrees(rotated_route_angle))).strip()
        if not os.path.exists(dir_name):
            break
        else:
            x = x + 1

    num_photos, project_folder = controller.download_photos(dir_name=dir_name)
    return [True, f"{num_photos} downloaded to {project_folder}"]


if __name__ == '__main__':

    print(sys.argv)
    if sys.argv[1] == "test":
        mission_id = 0
        drone_id = 1
        drone_ip_address = "10.202.0.1"
        altitude = 200
        intersection_ratio = 0.8
        gimbal_angle = -90
        route_angle = math.pi * 0 / 180
        rotated_route_angle = math.pi * 20 / 180

        vertices = [(2.37, 48.8802), (2.372, 48.8802),
                    (2.372, 48.88), (2.37, 48.88)]
    elif sys.argv[1] == "test2":
        mission_id = 0
        drone_id = 1
        drone_ip_address = "10.202.0.1"
        altitude = 30
        intersection_ratio = 0.8
        gimbal_angle = -90
        route_angle = math.pi * 20 / 180
        rotated_route_angle = math.pi * 20 / 180

        vertices = [(2.369415, 48.880494), (2.369415, 48.880966), (2.371684, 48.880966), (2.371684, 48.880494)]
    else:
        if len(sys.argv) < 8:
            print("Usage: <altitude> <rotation_angle> <intersection_ratio> <vertex1_lon> <vertex1_lat> ... <vertexN_lat>")
            sys.exit(1)

        mission_id = float(sys.argv[1])
        drone_id = sys.argv[2]
        drone_ip_address = sys.argv[3]
        altitude = float(sys.argv[4])
        intersection_ratio = float(sys.argv[5])
        gimbal_angle = float(sys.argv[6])
        route_angle = math.pi * float(sys.argv[7]) / 180
        rotated_route_angle = math.pi * float(sys.argv[8]) / 180

        vertices = [(float(sys.argv[i]), float(sys.argv[i+1]))
                    for i in range(9, len(sys.argv), 2)]

    # Create Node instances for each vertex
    # nodes = [Node(lon, lat, 0) for lon, lat in vertices]

    rospy.init_node(f'drone_controller_node_{drone_id}')
    rospy.on_shutdown(exit)

    print("imports are done")

    anafi_url = "http://{}".format(drone_ip_address)

    anafi_media_api_url = ""

    if drone_ip_address == "10.202.0.1":
        anafi_media_api_url = anafi_url + "/api/v1/media/medias/"
    elif drone_ip_address == "192.168.53.1":
        anafi_media_api_url = anafi_url + ":180/api/v1/media/medias/"
    else:
        # FAKE DRONE
        pass

    drone = olympe.Drone(drone_ip_address)

    event_listener = EventListener(drone)
    event_listener.subscribe()

    controller = Controller(drone=drone, anafi_url=anafi_url,
                            media_api_url=anafi_media_api_url)

    drone_gps = controller.get_drone_gps()

    paused = False

    rospy.loginfo("Ready to connect...")

    rospy.Service(f"drone_{drone_id}/connect", Trigger, handle_drone_connect)
    rospy.Service(f"drone_{drone_id}/calibrate",
                  Trigger, handle_drone_calibrate)
    rospy.Subscriber(f"drone_{drone_id}/start_mission",
                     Point, handle_drone_start_mission)
    rospy.Service(f"drone_{drone_id}/pause_mission",
                  Trigger, handle_drone_pause)
    rospy.Service(f"drone_{drone_id}/land", Trigger, handle_drone_land)
    rospy.Service(f"drone_{drone_id}/rth", Trigger, handle_drone_rth)
    rospy.Service(f"drone_{drone_id}/download_photos",
                  Trigger, handle_download_photos)
    # mission_server = actionlib.SimpleActionServer("drone/start_mission", StartMissionAction, handle_drone_start_mission, False)
    # mission_server.start()

    rospy.spin()
