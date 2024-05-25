import os
import sys
import time
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
import subprocess
from UI.database import Mission, get_mission_by_id

import pexpect

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Point


class DroneController(QThread):
    started = pyqtSignal(str)
    services_started = pyqtSignal(QThread)
    progress_text = pyqtSignal(str)
    progress = pyqtSignal(tuple, int)
    update_coord = pyqtSignal(float, float, int)
    update_status = pyqtSignal(str, int, int)
    update_battery = pyqtSignal(float, int, int)
    finished = pyqtSignal(str, int)

    def __init__(self, vertices, mission_id=1, drone_id=1, drone_ip_address="10.202.0.1", flight_altitude=100, rotation_angle=20, intersection_ratio=0.8, gimbal_angle=-90, route_angle=0, rotated_route_angle=20):
        super().__init__()

        self.mission_id = mission_id
        self.drone_id = drone_id
        self.drone_ip_address = drone_ip_address

        self.vertices = vertices
        self.flight_altitude = flight_altitude
        self.rotation_angle = rotation_angle
        self.intersection_ratio = intersection_ratio
        self.gimbal_angle = gimbal_angle
        self.route_angle = route_angle
        self.rotated_route_angle = rotated_route_angle

        self.roscore_process = subprocess.Popen(['roscore'])
        self.progress_text.emit("Starting roscore...")
        time.sleep(5)
        self.progress_text.emit("Started roscore")
        rospy.init_node('drone_controller')

        self.process = None
        # rospy.init_node(f'drone_client_{self.drone_id}')

    def run(self):
        self.started.emit(f"Mission #{self.mission_id} ---- START")

        project_folder = ""

        coords = [str(coord) for vertex in self.vertices for coord in vertex]

        rosrun_command = ["rosrun", "route_control", "main.py", str(self.mission_id), str(self.drone_id), str(self.drone_ip_address),
                          str(self.flight_altitude), str(
                              self.intersection_ratio),
                          str(self.gimbal_angle), str(self.route_angle), str(self.rotated_route_angle), *coords]

        command = " ".join(rosrun_command)

        if not os.path.exists('logs'):
            os.mkdir('logs')

        self.process = pexpect.spawn(command, encoding='utf-8', timeout=300)
        self.process.logfile = open(
            f"logs/mission_{str(self.mission_id)}-drone_{str(self.drone_id)}.log", "w")

        battery_percent = None

        self.process.expect("Ready to connect")

        rospy.wait_for_service(f'/drone_{self.drone_id}/connect')
        self.connect_service = rospy.ServiceProxy(
            f'/drone_{self.drone_id}/connect', Trigger)

        rospy.wait_for_service(f'/drone_{self.drone_id}/calibrate')
        self.calibrate_service = rospy.ServiceProxy(
            f"drone_{self.drone_id}/calibrate", Trigger)

        # rospy.wait_for_service("drone/start_mission")
        self.start_mission_service = rospy.Publisher(
            f"drone_{self.drone_id}/start_mission", Point, queue_size=10)

        rospy.wait_for_service(f"drone_{self.drone_id}/pause_mission")
        self.pause_mission_service = rospy.ServiceProxy(
            f"drone_{self.drone_id}/pause_mission", Trigger)

        rospy.wait_for_service(f"drone_{self.drone_id}/land")
        self.land_service = rospy.ServiceProxy(
            f"drone_{self.drone_id}/land", Trigger)

        rospy.wait_for_service(f"drone_{self.drone_id}/rth")
        self.rth_service = rospy.ServiceProxy(
            f"drone_{self.drone_id}/rth", Trigger)

        rospy.wait_for_service(f"drone_{self.drone_id}/download_photos")
        self.download_photos_service = rospy.ServiceProxy(
            f"drone_{self.drone_id}/download_photos", Trigger)

        self.services_started.emit(self)

        while not self.process.eof():

            line = str(self.process.readline())

            if line.startswith("Usage:"):
                self.process.kill()
                self.roscore_process.kill()
                self.progress_text.emit("Parameter error!")
                break
            elif line.startswith("Mission Finished"):
                # >>> b'Mission Finished: project\r\n'
                # project_folder = line[len("Mission Finished: ")].strip()
                self.finished.emit(
                    f"Mission #{self.mission_id} ---- END", self.mission_id)

            elif "Project folder created: " in line:
                project_folder = line.split(':')[-1].strip()
                print(project_folder)
                mission = get_mission_by_id(self.mission_id)
                mission.set_project_folder(project_folder)
            elif line.startswith("("):
                try:
                    coordinates = line.split(',')
                    latitude = float(coordinates[0][1:])
                    longitude = float(coordinates[1])
                except Exception as e:
                    print(repr(e))
                else:
                    self.update_coord.emit(latitude, longitude, self.drone_id)
            elif line.startswith("moving end"):
                words = line.split(" ")
                lat = float(words[-2])
                lon = float(words[-1])
                self.progress.emit((lat, lon), self.drone_id)
            elif line.startswith("Connection State: "):
                connection_state = True if line[len(
                    "Connection State: "):].strip() == "True" else False
                print("Is connected: ", connection_state)
            elif "Calibration Required: " in line:
                is_calibration_required = bool(int(line.strip()[-1]))
                print("Is calibration required: ", is_calibration_required)
            elif "Calibration Started: " in line:
                is_calibration_started = bool(int(line.strip()[-1]))
                print("Is calibration started: ", is_calibration_started)
            elif "Axis to Calibrate: " in line:
                axis_to_calibrate = bool(int(line.strip()[-1]))
                print("Axis to calibrate: ", axis_to_calibrate)
            elif "flying_state: " in line:
                status = str(line.split(".")[-1]).strip().capitalize()
                self.update_status.emit(status, self.mission_id, self.drone_id)
            elif "Battery State: " in line:
                battery_percent = float(line.split(":")[-1])
                self.update_battery.emit(
                    battery_percent, self.mission_id, self.drone_id)

            # (48.880642477419514, 2.3696386128612215, 226.89425659179688)'
            print(">>> " + line)

        self.process.wait()

        self.finished.emit(
            f"Mission #{self.mission_id} ---- END", self.mission_id)

    def terminate(self):
        self.progress_text.emit(
            f"Shutting down drone_controller_node_{self.drone_id}")

        self.process.sendcontrol('c')
        # time.sleep(5)

        # while not self.process.closed:
        #     self.process.kill(9)
        #     terminated = self.process.terminate(force=True)
        #     self.process.close(force=True)


        time.sleep(5)
        self.progress_text.emit(
            f"Shut down drone_controller_node_{self.drone_id}")

    def connect_drone(self):
        print("Attempting to connect")
        print(self.connect_service(TriggerRequest()))

    def calibrate_drone(self):
        print(self.calibrate_service(TriggerRequest()))

    def start_mission(self, last_visited_node):
        # self.startt = True
        # print(self.start_mission_service.publish())
        self.start_mission_service.publish(
            last_visited_node[0], last_visited_node[1], 0)
        while self.start_mission_service.get_num_connections() == 0:
            rospy.sleep(1)

    def pause_mission(self):
        print(self.pause_mission_service(TriggerRequest()))

    def land_drone(self):
        print(self.land_service(TriggerRequest()))

    def rth_drone(self):
        print(self.rth_service(TriggerRequest()))

    def download_photos(self):
        print(self.download_photos_service(TriggerRequest()))
