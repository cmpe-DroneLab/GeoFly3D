import time
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot

class DroneController(QThread):
    started = pyqtSignal(str)
    progress_text = pyqtSignal(str)
    finished = pyqtSignal(str)

    def __init__(self, mission_id=1):
        super().__init__()
        self.mission_id = mission_id

    def run(self):
        self.started.emit(f"Mission #{self.mission_id} ---- START")

        for i in range(10):
            time.sleep(1)
            self.progress_text.emit(f"Mission progress: {i*10}%")

        self.finished.emit(f"Mission #{self.mission_id} ---- END")