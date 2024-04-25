import sys
import time
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import subprocess

import pexpect

class DroneController(QThread):
    started = pyqtSignal(str)
    progress_text = pyqtSignal(str)
    finished = pyqtSignal(str, str, int)

    def __init__(self, vertices, flight_altitude, rotation_angle=20, intersection_ratio=0.8, mission_id=1):
        super().__init__()

        # self.optimal_route = optimal_route
        # self.rotated_route = rotated_route
        self.vertices = vertices
        self.flight_altitude = flight_altitude
        self.rotation_angle = rotation_angle
        self.intersection_ratio = intersection_ratio
        self.mission_id = mission_id

    def run(self):
        self.started.emit(f"Mission #{self.mission_id} ---- START")

        project_folder = ""

        roscore_process = subprocess.Popen(['roscore'])
        self.progress_text.emit("Starting roscore...")
        time.sleep(5)
        self.progress_text.emit("Started roscore")

        coords = [str(coord) for vertex in self.vertices for coord in vertex]

        # rosrun_command = f"rosrun route_control main.py {self.flight_altitude} {self.rotation_angle} {self.intersection_ratio} {coords}"
        rosrun_command = ["rosrun", "route_control", "main.py", str(self.flight_altitude), str(self.rotation_angle), str(self.intersection_ratio), *coords]

        command = " ".join(rosrun_command)
        process = pexpect.spawn(command)
        while not process.eof():
            line = str(process.readline())

            if line.startswith("Usage:"):
                process.kill()
                roscore_process.kill()
                self.progress_text.emit("Parameter error!")
                break
            elif line.startswith("b'Mission Finished"):
                # >>> b'Mission Finished: project\r\n'
                project_folder = line[len("b'Mission Finished: "):-5]
                
            print(">>> " + line)

    
        process.wait()
        
    

        self.finished.emit(f"Mission #{self.mission_id} ---- END", project_folder, self.mission_id)