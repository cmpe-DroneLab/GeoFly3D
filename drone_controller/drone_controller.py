import os
import sys
import time
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
import subprocess
from UI.database import session, Mission

import pexpect

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Empty

class DroneController(QThread):
    started = pyqtSignal(str)
    services_started = pyqtSignal(QThread)
    progress_text = pyqtSignal(str)
    progress = pyqtSignal(tuple)
    update_coord = pyqtSignal(float, float)
    update_status = pyqtSignal(str)
    update_battery = pyqtSignal(float)
    finished = pyqtSignal(str, int)

    def __init__(self, vertices, mission_id=1, flight_altitude=100, rotation_angle=20, intersection_ratio=0.8, gimbal_angle=-90, route_angle=0, rotated_route_angle=20):
        super().__init__()

        # self.optimal_route = optimal_route
        # self.rotated_route = rotated_route
        self.vertices = vertices
        self.flight_altitude = flight_altitude
        self.rotation_angle = rotation_angle
        self.intersection_ratio = intersection_ratio
        self.mission_id = mission_id
        self.gimbal_angle = gimbal_angle
        self.route_angle = route_angle
        self.rotated_route_angle = rotated_route_angle

        self.startt = False

        rospy.init_node('drone_controller')
        

    def run(self):
        self.started.emit(f"Mission #{self.mission_id} ---- START")

        project_folder = ""

        roscore_process = subprocess.Popen(['roscore'])
        self.progress_text.emit("Starting roscore...")
        time.sleep(5)
        self.progress_text.emit("Started roscore")

        coords = [str(coord) for vertex in self.vertices for coord in vertex]

        # rosrun_command = f"rosrun route_control main.py {self.flight_altitude} {self.rotation_angle} {self.intersection_ratio} {coords}"
        rosrun_command = ["rosrun", "route_control", "main.py", str(self.mission_id),
                           str(self.flight_altitude), str(self.intersection_ratio),
                             str(self.gimbal_angle), str(self.route_angle), str(self.rotated_route_angle), *coords]

        command = " ".join(rosrun_command)

        if not os.path.exists('logs'):
            os.mkdir('logs')

        process = pexpect.spawn(command, encoding='utf-8')
        process.logfile = open(f"logs/output{str(self.mission_id)}.log", "w")
        
        battery_percent = None

        process.expect("Ready to connect")

        rospy.wait_for_service('/drone/connect')
        self.connect_service = rospy.ServiceProxy('/drone/connect', Trigger)

        rospy.wait_for_service('/drone/calibrate')
        self.calibrate_service = rospy.ServiceProxy("drone/calibrate", Trigger)

        # rospy.wait_for_service("drone/start_mission")
        self.start_mission_service = rospy.Publisher("drone/start_mission", Empty, queue_size=10)

        rospy.wait_for_service("drone/pause_mission")
        self.pause_mission_service = rospy.ServiceProxy("drone/pause_mission", Trigger)

        rospy.wait_for_service("drone/land")
        self.land_service = rospy.ServiceProxy("drone/land", Trigger)

        rospy.wait_for_service("drone/rth")
        self.rth_service = rospy.ServiceProxy("drone/rth", Trigger)

        rospy.wait_for_service("drone/download_photos")        
        self.download_photos_service = rospy.ServiceProxy("drone/download_photos", Trigger)        


        self.services_started.emit(self)


        while not process.eof():

            line = str(process.readline())

            if line.startswith("Usage:"):
                process.kill()
                roscore_process.kill()
                self.progress_text.emit("Parameter error!")
                break
            # elif line.startswith("Mission Finished"):
            #     # >>> b'Mission Finished: project\r\n'
            #     project_folder = line[len("Mission Finished: ")].strip()
            elif "Project folder created: " in line:
                project_folder = line.split(':')[-1].strip()
                print(project_folder)
                mission = session.query(Mission).filter_by(mission_id=self.mission_id).first()
                mission.project_folder = project_folder
                session.commit()
            elif line.startswith("("):
                try:
                    coordinates = line.split(',')
                    latitude = float(coordinates[0][1:])
                    longitude = float(coordinates[1])
                except Exception as e:
                    print(repr(e))
                else:
                    self.update_coord.emit(latitude, longitude)
            elif line.startswith("moving end"):
                words = line.split(" ")
                lat = float(words[-2])
                lon = float(words[-1])
                self.progress.emit((lat, lon))
            elif line.startswith("Connection State: "):
                 connection_state = True if line[len("Connection State: "):].strip() == "True" else False
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
                self.update_status.emit(status)
            elif "Battery State: " in line:
                battery_percent = float(line.split(":")[-1])
                self.update_battery.emit(battery_percent)
                


            # (48.880642477419514, 2.3696386128612215, 226.89425659179688)'
            print(">>> " + line)

    
        process.wait()
        
    

        self.finished.emit(f"Mission #{self.mission_id} ---- END", self.mission_id)
    
    def connect_drone(self):
        print(self.connect_service(TriggerRequest()))

    def calibrate_drone(self):
        print(self.calibrate_service(TriggerRequest()))

    def start_mission(self):
        # self.startt = True
        # print(self.start_mission_service.publish())
        self.start_mission_service.publish()
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
        