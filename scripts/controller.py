import time
from datetime import datetime
import math
import random
import os
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET

import rospy

import olympe
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged, moveToChanged, FlyingStateChanged, moveByChanged
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveTo, moveBy, CancelMoveTo
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.common.Calibration import MagnetoCalibration
from olympe.messages.common.CalibrationState import MagnetoCalibrationRequiredState, MagnetoCalibrationStartedChanged
from olympe.enums.gimbal import control_mode, frame_of_reference
from olympe.messages.gimbal import set_target
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.enums.ardrone3.PilotingState import MoveToChanged_Status
from olympe.enums.camera import photo_file_format, photo_format
from olympe.messages.camera import set_photo_mode, take_photo, set_camera_mode, photo_progress, stop_photo, photo_mode
from olympe.messages.rth import return_to_home
from olympe.media import download_media, indexing_state, delete_media, delete_all_media

from helpers import calculate_bearing_angle, check_connection, calculate_geographic_distance
from drone_gps import DroneGPS
from exceptions import PauseException

XMP_TAGS_OF_INTEREST = (
    "CameraRollDegree",
    "CameraPitchDegree",
    "CameraYawDegree",
    "CaptureTsUs",

    "GPSLatitude",
    "GPSLongitude",
    "GPSAltitude",
)



class Controller:

    def __init__(self, drone, anafi_url, media_api_url):
        self.current_position = (0, 0)

        self.drone = drone
        self.drone_gps = DroneGPS(drone=drone)
        self.cancelled = False
        self.anafi_url = anafi_url
        self.media_api_url = media_api_url

    def get_drone_gps(self):
        return self.drone_gps

    def setup_photo_gpslapse_mode(self, capture_interval):
        if self.cancelled:
            raise PauseException("Paused")
        check_connection(self.drone)
        self.drone(set_camera_mode(cam_id=0, value="photo")).wait()
        self.drone(
            set_photo_mode(
                cam_id=0,
                mode="gps_lapse",
                format="rectilinear",
                file_format="jpeg",
                burst="burst_14_over_1s",
                bracketing="preset_1ev",
                capture_interval=capture_interval * 0.9,
            )
        ).wait()

    def download_photos(self, dir_name):
        if self.cancelled:
            raise PauseException("Paused")
        check_connection(self.drone)
        num_media = 0
        download_folder = ""

        if not os.path.exists(dir_name):
            download_folder = "./" + dir_name + "/images"
            os.mkdir(dir_name)
            os.mkdir(download_folder)
            rospy.loginfo("Project folder created: %s", download_folder)

        time.sleep(15)
        self.drone.media(indexing_state(state="indexed")).wait(
            _timeout=100)  # FIXME: fails on some drones
        media_id = olympe.Media.list_media(self.drone.media)
        num_media = len(media_id)

        if num_media > 0:
            # download the photos associated with this media id
            self.drone.media.download_dir = download_folder

            media_info = olympe.Media.media_info(
                self.drone.media, media_id=media_id[-1])
            run_id = media_info.run_id

            for mid in range(num_media - 1, -1, -1):
                media_info = olympe.Media.media_info(
                    self.drone.media, media_id=media_id[mid])
                rospy.loginfo(media_info.run_id)
                rid = media_info.run_id
                if rid != run_id:
                    num_media = num_media - mid - 1
                    mid += 1
                    break

            rospy.loginfo("Downloading %i media", num_media)
            media_id = media_id[mid:]
            media_count = 1
            for media in media_id:
                media_info = olympe.Media.media_info(
                    self.drone.media, media_id=media)
                rospy.logwarn(media_info)
                rospy.loginfo("Media %i/%i: downloading %.1fMB",
                              media_count, num_media, media_info.size/(2**20))
                media_download = self.drone(download_media(media))
                resources = media_download.as_completed(timeout=100)
                rospy.loginfo("Media %i/%i: downloaded %.1fMB",
                              media_count, num_media, media_info.size/(2**20))

                for resource in resources:
                    if not resource.success():
                        rospy.logerr("Failed to download %s",
                                     str(resource.resource_id))
                        continue

                media_count += 1

                # if cut_media:
                #     drone(delete_all_media())

            # else:
            #     rospy.loginfo("No media found")
            #     rospy.loginfo(self.drone.media.indexing_state)
        else:
            rospy.logwarn("Media is not indexed :(")
        return num_media, dir_name

    def take_photo_single(self):
        if self.cancelled:
            raise PauseException("Paused")
        check_connection(self.drone)
        print("taking photo")
        photo_saved = self.drone(photo_progress(
            result="photo_saved", _policy="wait"))
        expectation = self.drone(take_photo(cam_id=0)).wait()
        assert expectation.success(), expectation.explain()
        photo_saved.wait()
        media_id = photo_saved.received_events().last().args["media_id"]

        media_info_response = requests.get(self.media_api_url + media_id)
        media_info_response.raise_for_status()
        download_dir = tempfile.mkdtemp()
        for resource in media_info_response.json()["resources"]:
            image_response = requests.get(
                self.anafi_url + resource["url"], stream=True)
            download_path = os.path.join(download_dir, resource["resource_id"])
            image_response.raise_for_status()
            with open(download_path, "wb") as image_file:
                shutil.copyfileobj(image_response.raw, image_file)

            with open(download_path, "rb") as image_file:
                image_data = image_file.read()
                image_xmp_start = image_data.find(b"<x:xmpmeta")
                image_xmp_end = image_data.find(b"</x:xmpmeta")
                image_xmp = ET.fromstring(
                    image_data[image_xmp_start: image_xmp_end + 12])
                for image_meta in image_xmp[0][0]:
                    xmp_tag = re.sub(r"{[^}]*}", "", image_meta.tag)
                    xmp_value = image_meta.text

                    if xmp_tag in XMP_TAGS_OF_INTEREST:
                        print(resource["resource_id"], xmp_tag, xmp_value)

    def setup_photo_single_mode(self):
        if self.cancelled:
            raise PauseException("Paused")
        check_connection(self.drone)
        expectation = self.drone(set_camera_mode(cam_id=0, value="photo")).wait()
        assert expectation.success(), expectation.explain()
        expectation = self.drone(
            set_photo_mode(
                cam_id=0,
                mode="single",
                format="rectilinear",
                file_format="jpeg",
                burst="burst_14_over_1s",
                bracketing="preset_1ev",
                capture_interval=0.0,
            )
        ).wait()
        assert expectation.success(), expectation.explain()

    def land(self):
        if self.cancelled:
            raise PauseException("Paused")
        print("Landing....")
        self.drone(Landing()).wait().success()

    def takeoff(self):
        if self.cancelled:
            raise PauseException("Paused")
        print("Takingoff....")
        self.drone(TakeOff()).wait().success()

    def rth(self):
        if self.cancelled:
            raise PauseException("Paused")
        print("Returning to Home...")
        self.drone(return_to_home()).wait().success()

    def move_to(self, lat, lon, alt, drone_gps):
        if self.cancelled:
            raise PauseException("Paused")
        print("moving...")

        check_connection(self.drone)

        while True:
            try:
                heading = math.degrees(calculate_bearing_angle(
                    drone_gps.get_current_position(), (lat, lon, 0)))
                    
                self.drone(moveTo(lat, lon, alt, MoveTo_Orientation_mode.HEADING_START, heading)
                           # >> FlyingStateChanged(state="hovering", _timeout=5)
                           >> moveToChanged(status='DONE')
                           ).wait().success()

                while not drone_gps.has_reached_target(lat, lon) and not self.cancelled:
                    if not self.cancelled and self.drone.get_state(moveToChanged())['status'] == MoveToChanged_Status.CANCELED:
                        raise Exception
                    print("Drone has not reached the target yet. Waiting...")
                    time.sleep(2)
            except Exception as e:
                print("Connection failed! Reconnecting and moving! " + repr(e))
                check_connection(self.drone)
                continue  # Retry the operation
            else:
                break  # Exit the loop if successful

        print(f"moving end {lat} {lon}")

    def follow_route(self, route, altitude, vertical_increment, last_visited_node):
        if self.cancelled:
            raise PauseException("Paused")
        is_gpslapse_on = False
        self.setup_photo_gpslapse_mode(vertical_increment)

        can_follow = last_visited_node == (500, 500)

        for i, point in enumerate(route):
            # point = point[::-1]

            if not can_follow:
                if last_visited_node == point:
                   can_follow = True
                   if i % 2 != 0:
                        continue
                else:
                    continue

            if self.cancelled:
                return PauseException("Paused")

            print(f"point {i+1}/{len(route)}: {point}")
            lat = point[0]
            lon = point[1]
            self.move_to(lat, lon, altitude, self.drone_gps)

            # if is_gpslapse_on:
            #     rospy.loginfo(self.drone.get_state(photo_mode)[0]['mode'])
            #     expectation = self.drone(stop_photo(cam_id=0)).wait()
            #     assert expectation.success(), expectation.explain
            #     self.setup_photo_single_mode()
            #     self.take_photo_single()
            #     self.setup_photo_gpslapse_mode(vertical_increment)

            self.drone(take_photo(cam_id=0))
            time.sleep(4)
            
            if is_gpslapse_on:
                print("GPS LAPSE ON SET TO OFF")
                self.drone(stop_photo(cam_id=0))
                # .wait()

            # time.sleep(4)

            is_gpslapse_on = not is_gpslapse_on

            

        return True

    def fake_move_to(self, lat, lon):
        if self.cancelled:
            raise PauseException("Paused")
        print("moving...")


        delta_lat = lat - self.current_position[0]
        delta_lon = lon - self.current_position[1]

        distance = calculate_geographic_distance(self.current_position, (lat, lon))

        if distance > 0:

            lat_vector = delta_lat / distance
            lon_vector = delta_lon / distance
            print(str((*self.current_position, 0)))

            movement_speed = random.randint(10, 15) # m/s
            depth_limit = 1000

            time_step = 2

            while not self.cancelled and distance > 0.01 and depth_limit > 0:
                print("Drone has not reached the target yet. Waiting...")

                depth_limit -= 1
                
                movement_distance = movement_speed * time_step

                if movement_distance > distance:
                    movement_distance = distance
                    time_step = movement_distance / movement_speed
                
                lat_displacement = movement_distance * lat_vector
                lon_displacement = movement_distance * lon_vector
                
                self.current_position = (self.current_position[0] + lat_displacement,
                                    self.current_position[1] + lon_displacement)
                
                time.sleep(time_step)
                
                distance = calculate_geographic_distance(self.current_position, (lat, lon))
                
                print(str((*self.current_position, 0)))

        print("Destination reached...")
                            
        print(f"moving end {lat} {lon}")

    def fake_follow_route(self, route, last_visited_node):
        if self.cancelled:
            raise PauseException("Paused")
        is_gpslapse_on = False

        can_follow = last_visited_node == (500, 500)

        for i, point in enumerate(route):
            # point = point[::-1]

            if not can_follow:
                if last_visited_node == point:
                   can_follow = True
                   if i % 2 != 0:
                        continue
                else:
                    continue

            if self.cancelled:
                return PauseException("Paused")

            print(f"point {i+1}/{len(route)}: {point}")
            lat = point[0]
            lon = point[1]

            self.fake_move_to(lat, lon)

            print("Taking photo")
            time.sleep(4)
            
            if is_gpslapse_on:
                print("GPS LAPSE ON SET TO OFF")

            is_gpslapse_on = not is_gpslapse_on

            

        return True


    def calibrate(self):
        
        while True:
            try:
                print(self.drone.get_state(MagnetoCalibrationRequiredState))
                if self.drone.get_state(MagnetoCalibrationRequiredState)['required'] == 1 and self.drone.get_state(MagnetoCalibrationStartedChanged)['started'] == 0:
                    assert self.drone(MagnetoCalibration(1)).wait().success()

                while self.drone.get_state(MagnetoCalibrationRequiredState)['required'] == 1:
                    if self.cancelled:
                        raise PauseException("Paused")
                    time.sleep(10)

            except:
                check_connection(self.drone)
                continue
            else:
                break

    def set_pause(self, is_paused):
        self.cancelled = is_paused

    def set_gimbal_angle(self, gimbal_angle):
        if self.cancelled:
            raise PauseException("Paused")
        print(f"setting gimball angle to {gimbal_angle} degrees....")
        assert self.drone(set_target(gimbal_id=0, control_mode=control_mode.position, yaw_frame_of_reference=frame_of_reference.relative, yaw=0.0,
                                     pitch_frame_of_reference=frame_of_reference.relative, pitch=gimbal_angle, roll_frame_of_reference=frame_of_reference.relative,
                                     roll=0.0)).wait().success()

    def set_takeoff_point(self, takeoff_point):
        self.current_position = takeoff_point