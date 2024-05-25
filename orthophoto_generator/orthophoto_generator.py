import os
import time
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
import pexpect

class OrthophotoGenerator(QThread):
    started = pyqtSignal(str)
    progress_text = pyqtSignal(str)
    finished = pyqtSignal(str)

    def __init__(self, project_folder, orthophoto_resolution=1, mission_id=1):
        super().__init__()

        self.project_folder = project_folder
        self.orthophoto_resolution = orthophoto_resolution
        self.mission_id = mission_id

    def run(self):
        self.started.emit(f"Process #{self.mission_id} ---- START")

        cwd = os.getcwd()
        options = ""
        command = "docker run -ti --rm -v " + cwd + ":/projects opendronemap/odm --project-path /projects " + str(self.project_folder) +" --orthophoto-resolution " + str(self.orthophoto_resolution) + options
        print(command)

        process = pexpect.spawn(command)
        while not process.eof():
            line = str(process.readline())
            print(">>> " + line)

        process.wait()

        self.finished.emit(f"Process #{self.mission_id} ---- END")