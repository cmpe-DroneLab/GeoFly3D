import time
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import subprocess

class DroneController(QThread):
    started = pyqtSignal(str)
    progress_text = pyqtSignal(str)
    finished = pyqtSignal(str)

    def __init__(self, vertices, flight_altitude, rotation_angle=20, ortophoto_resolution=1, intersection_ratio=0.8, mission_id=1):
        super().__init__()

        # self.optimal_route = optimal_route
        # self.rotated_route = rotated_route
        self.vertices = vertices
        self.flight_altitude = flight_altitude
        self.rotation_angle = rotation_angle
        self.ortophoto_resolution = ortophoto_resolution
        self.intersection_ratio = intersection_ratio
        self.mission_id = mission_id

    def run(self):
        self.started.emit(f"Mission #{self.mission_id} ---- START")

        # for i in range(10):
        #     time.sleep(1)
        #     self.progress_text.emit(f"Mission progress: {i*10}%")

        roscore_process = subprocess.Popen(['roscore'])
        self.progress_text.emit("Starting roscore...")
        time.sleep(5)
        self.progress_text.emit("Started roscore")

        coords = " ".join([str(coord) for vertex in self.vertices for coord in vertex])

        rosrun_command = f"rosrun route_control main.py {self.flight_altitude} {self.rotation_angle} {self.ortophoto_resolution} {self.intersection_ratio} {coords}"
        print(rosrun_command)
        process = subprocess.Popen(rosrun_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, bufsize=1, universal_newlines=True)

        for line in iter(process.stdout.readline, b""):
            line = line.strip()
            print(line)
            if line.startswith("Usage:"):
                process.kill()
                roscore_process.kill()
                self.progress_text.emit("Parameter error!")
                break
        else:            
            process.stdout.close()
            process.wait()
        
    

        self.finished.emit(f"Mission #{self.mission_id} ---- END")