import os
import sys
import time
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import subprocess

import pexpect

class DroneController(QThread):
    started = pyqtSignal(str)
    progress_text = pyqtSignal(str)
    update_coord = pyqtSignal(float, float, float)
    finished = pyqtSignal(str, str, int)

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

        while not process.eof():
            line = str(process.readline())

            if line.startswith("Usage:"):
                process.kill()
                roscore_process.kill()
                self.progress_text.emit("Parameter error!")
                break
            elif line.startswith("Mission Finished"):
                # >>> b'Mission Finished: project\r\n'
                project_folder = line[len("Mission Finished: ")].strip()
            elif line.startswith("("):
                try:
                    coordinates = line.split(',')
                    latitude = float(coordinates[0][1:])
                    longitude = float(coordinates[1])
                    battery_percent = float(line.strip().split("Battery: ")[-1][:-1])
                except Exception as e:
                    print(repr(e))
                else:
                    self.update_coord.emit(latitude, longitude, battery_percent)
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


            # (48.880642477419514, 2.3696386128612215, 226.89425659179688)'
            print(">>> " + line)

    
        process.wait()
        
    

        self.finished.emit(f"Mission #{self.mission_id} ---- END", project_folder, self.mission_id)