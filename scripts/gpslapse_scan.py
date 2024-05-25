#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import olympe
import time
import os
from olympe.messages.ardrone3.PilotingState import GpsLocationChanged
from olympe.messages.ardrone3.Piloting import *
from olympe.enums.gimbal import control_mode, frame_of_reference
from olympe.messages.gimbal import set_target
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.enums.camera import photo_file_format,photo_format,camera_mode,photo_mode
from olympe.messages.camera import set_photo_mode,take_photo,set_camera_mode,photo_progress, stop_photo
from olympe.messages.rth import return_to_home
from olympe.media import download_media, indexing_state, delete_media, delete_all_media
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
from node import Node
from sparse_route_planning import create_sparse_route
from orthophoto_generator import generate_orthophoto

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")

ANAFI_URL = "http://{}/".format(DRONE_IP)

ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias/"

XMP_TAGS_OF_INTEREST = (
    "CameraRollDegree",
    "CameraPitchDegree",
    "CameraYawDegree",
    "CaptureTsUs",

    "GPSLatitude",
    "GPSLongitude",
    "GPSAltitude",
)
#DroneGPS Startsset_camera_mode
#############################################################################################
class DroneGPS:
    def __init__(self, drone):
        self.drone = drone
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.subscription = None

    def position_changed_callback(self, event):
        self.latitude = event.args["latitude"]
        self.longitude = event.args["longitude"]
        self.altitude = event.args["altitude"]
        print(f"Updated Position: Latitude: {self.latitude}, Longitude: {self.longitude}, Altitude: {self.altitude}").wait().success()

    def start(self):
        # self.subscription = self.drone.subscribe(self.position_changed_callback, PositionChanged())
        pass

    def has_reached_target(self, target_latitude, target_longitude, tolerance=0.0001):
        """
        Check if the current GPS position is within a certain tolerance of the target position.
        The default tolerance is roughly equivalent to 11 meters.
        """
        print(self.get_current_position())
        if self.latitude is None or self.longitude is None:
            return False
        lat_diff = abs(self.latitude - target_latitude)
        lon_diff = abs(self.longitude - target_longitude)
        return lat_diff < tolerance and lon_diff < tolerance   


    def stop(self):
        # if self.subscription is not None:
        #     self.drone.unsubscribe(self.subscription)
        pass
        
    def get_current_position(self):
        gps_ret = drone.get_state(GpsLocationChanged)
        self.latitude = gps_ret["latitude"]
        self.longitude = gps_ret["longitude"]
        self.altitude = gps_ret["altitude"]
        if self.latitude is not None and self.longitude is not None:
            return self.latitude, self.longitude, self.altitude
        else:
            return "GPS position not available"
#DroneGPS ENDS
##########################################################################
def position_changed_callback(event, controller):
    latitude = event.args["latitude"]
    longitude = event.args["longitude"]
    altitude = event.args["altitude"]
    print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")


def take_photo_single(drone):
    print("taking photo")
    photo_saved = drone(photo_progress(result="photo_saved", _policy="wait"))
    # drone(take_photo(cam_id=0)).wait()
    photo_saved.wait()
    media_id = photo_saved.received_events().last().args["media_id"]


    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    download_dir = tempfile.mkdtemp()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])
        image_response.raise_for_status()
        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)

       
        with open(download_path, "rb") as image_file:
            image_data = image_file.read()
            image_xmp_start = image_data.find(b"<x:xmpmeta")
            image_xmp_end = image_data.find(b"</x:xmpmeta")
            image_xmp = ET.fromstring(image_data[image_xmp_start : image_xmp_end + 12])
            for image_meta in image_xmp[0][0]:
                xmp_tag = re.sub(r"{[^}]*}", "", image_meta.tag)
                xmp_value = image_meta.text
                
                if xmp_tag in XMP_TAGS_OF_INTEREST:
                    print(resource["resource_id"], xmp_tag, xmp_value)


def setup_photo_gpslapse_mode(drone):
    drone(set_camera_mode(cam_id=0, value="photo")).wait()
    drone(
        set_photo_mode(
            cam_id=0,
            mode="gps_lapse",
            format="rectilinear",
            file_format="jpeg",
            burst="burst_14_over_1s",
            bracketing="preset_1ev",
            capture_interval=vertical_increment * 0.9,
        )
    ).wait()

def download_photos():

    num_media = 0
    download_folder = ""
    if drone.media(indexing_state(state="indexed")).wait(_timeout=10).success():  # FIXME: fails on some drones
        media_id = olympe.Media.list_media(drone.media)
        num_media = len(media_id)

        if num_media > 0:
            x = 0
            while True:
                dir_name = ("project" + (str(x) if x != 0 else '')).strip()
                if not os.path.exists(dir_name):
                    download_folder = "./" + dir_name + "/images"
                    os.mkdir(dir_name)
                    os.mkdir(download_folder)
                    rospy.loginfo("Folder '%s' created", download_folder)
                    break
                else:
                    x = x + 1

           
            drone.media.download_dir = download_folder  # download the photos associated with this media id

            rospy.loginfo("Downloading %i media", num_media)

            media_count = 1
            for media in media_id:
                media_info = olympe.Media.media_info(drone.media, media_id = media)
                rospy.loginfo("Media %i/%i: downloading %.1fMB", media_count, num_media, media_info.size/(2**20))
                media_download = drone(download_media(media))
                resources = media_download.as_completed(timeout=100)
                rospy.loginfo("Media %i/%i: downloaded %.1fMB", media_count, num_media, media_info.size/(2**20))

                for resource in resources:
                    if not resource.success():
                        rospy.logerr("Failed to download %s", str(resource.resource_id))
                        continue

                media_count += 1

            # if cut_media:
            #     drone(delete_all_media())
            
        else:
            rospy.loginfo("No media found")
    else:
        rospy.logwarn("Media is not indexed :(")
    return num_media, dir_name

def take_photo_single(drone):
    print("taking photo")
    photo_saved = drone(photo_progress(result="photo_saved", _policy="wait"))
    drone(take_photo(cam_id=0)).wait()
    photo_saved.wait()
    media_id = photo_saved.received_events().last().args["media_id"]


    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    download_dir = tempfile.mkdtemp()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])
        image_response.raise_for_status()
        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)

       
        with open(download_path, "rb") as image_file:
            image_data = image_file.read()
            image_xmp_start = image_data.find(b"<x:xmpmeta")
            image_xmp_end = image_data.find(b"</x:xmpmeta")
            image_xmp = ET.fromstring(image_data[image_xmp_start : image_xmp_end + 12])
            for image_meta in image_xmp[0][0]:
                xmp_tag = re.sub(r"{[^}]*}", "", image_meta.tag)
                xmp_value = image_meta.text
                
                if xmp_tag in XMP_TAGS_OF_INTEREST:
                    print(resource["resource_id"], xmp_tag, xmp_value)


def setup_photo_single_mode(drone):
    drone(set_camera_mode(cam_id=0, value="photo")).wait()
    drone(
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

def land():
    print("Landing....")
    drone(Landing()).wait().success()

def takeoff():
    print("Takingoff....")
    drone(TakeOff()).wait().success()

def command_callback(msg):
    if msg.data == "takeoff":
        takeoff()

def move_to(lat,lon,alt,drone_gps):
    print("moving...") 
    drone(moveTo(lat, lon, alt, MoveTo_Orientation_mode.NONE, 0)).wait().success()
    while not drone_gps.has_reached_target(lat, lon):
        print("Drone has not reached the target yet. Waiting...")
        time.sleep(5) 
    print("moving end...")

if __name__ == '__main__':
    rospy.init_node('drone_command_node')

    altitude = float(input("Enter the flight altitude: ").strip() or 20)

    loc00= Node(float(input("Enter Corner 1 Latitude: ").strip() or 48.8802), float(input("Enter Corner 1 Longitude: ").strip() or 2.37), 170)
    loc01= Node(float(input("Enter Corner 2 Latitude: ").strip() or 48.8802), float(input("Enter Corner 2 Longitude: ").strip() or 2.372), 170)
    loc10= Node(float(input("Enter Corner 3 Latitude: ").strip() or 48.8810), float(input("Enter Corner 3 Longitude: ").strip() or 2.37), 170)
    loc11= Node(float(input("Enter Corner 4 Latitude: ").strip() or 48.8810), float(input("Enter Corner 4 Longitude: ").strip() or 2.372), 170)

    resolution = float(input("Enter orthophoto resolution cm/pixel: ").strip() or 1)

    print("imports are done")
   
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    drone_gps = DroneGPS(drone)
    drone_gps.start()
    time.sleep(2)
    print("Creating Route...")
    takeoff_coord = drone_gps.get_current_position()
    takeoff_n = Node(takeoff_coord[0], takeoff_coord[1], takeoff_coord[2])
    # loc00= Node(48.8802, 2.3696, 170)
    # loc01= Node(48.8802, 2.3700, 170)
    # loc10= Node(48.8810, 2.3696, 170)
    # loc11= Node(48.8810, 2.3700, 170)

    corner_list = [loc00, loc01, loc10, loc11]
    route, vertical_increment = create_sparse_route(takeoff_n, altitude, corner_list)
    print("route has been created")

    time.sleep(1)
    print("setting camera...")
    drone(set_camera_mode(0,value=camera_mode.photo))
    # drone(
    #     set_photo_mode(
    #         cam_id=0,
    #         mode="gps_lapse",
    #         format="rectilinear",
    #         file_format="jpeg",
    #         burst="burst_14_over_1s",
    #         # the following parameters are ignored in photo burst mode but we
    #         # must provide valid values for them anyway
    #         bracketing="preset_1ev",
    #         capture_interval=15.0,
    #     )
    # ).wait()

    takeoff()
    drone(moveBy(0, 0, -altitude, 0)).wait().success()
    time.sleep(2)
    print("setting gimball....")
    drone(set_target(gimbal_id=0,control_mode= control_mode.position, yaw_frame_of_reference=frame_of_reference.relative, yaw=0.0, 
                     pitch_frame_of_reference=frame_of_reference.relative, pitch=-90, roll_frame_of_reference=frame_of_reference.relative,
                     roll=0.0)).wait()
    
    time.sleep(5)
   
    setup_photo_gpslapse_mode(drone)

    is_gpslapse_on = False
    route = route[1:]
    for i,point in enumerate(route):
        
        print(f"point {i+1}/{len(route)}: {point.coordinates}")
        lat = point.coordinates[0]
        lon = point.coordinates[1]
        alt = drone_gps.get_current_position()[2]-takeoff_coord[2]
        move_to(lat, lon, alt,drone_gps)

        if is_gpslapse_on:
            assert drone(stop_photo(cam_id=0)).wait().success()
            setup_photo_single_mode(drone)
            take_photo_single(drone)
            setup_photo_gpslapse_mode(drone)

        else:
            drone(take_photo(cam_id=0)).wait()
            time.sleep(2)

        is_gpslapse_on = not is_gpslapse_on

        time.sleep(2)



    # assert drone(stop_photo(cam_id=0)).wait().success()


    # drone(moveBy(0, 0, -40, 0)).wait().success()
    # move_to(48.88064952124071, 2.370018686716253, 140.9792938232422,drone_gps)

    drone(return_to_home()).wait()
    time.sleep(20)  
    land()

    num_photos, project_folder = download_photos()


    print("Fetching current position...")
    print(drone_gps.get_current_position())
    drone_gps.stop()
    drone.disconnect()

    if num_photos > 0:
        generate_orthophoto(project_folder, resolution)
        