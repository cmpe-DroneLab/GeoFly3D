from PyQt6.QtWidgets import QWidget

import UI.Midflight.mid_design
from UI.database import Drone, session, Mission

from drone_controller import DroneController

class Mid(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Midflight.mid_design.Ui_Form()
        self.ui.setupUi(self)

        self.has_taken_off = False
        self.threads = {}
    
    def take_off(self, vertices, flight_altitude, mission_id):
        if self.has_taken_off:
            return
        
        self.mission_id = mission_id

        drone_controller_thread = DroneController(vertices=vertices, flight_altitude=flight_altitude)

        drone_controller_thread.started.connect(print)
        drone_controller_thread.progress_text.connect(print)

        self.threads[1] = drone_controller_thread
        drone_controller_thread.start()
        self.has_taken_off = True

        # DB UPDATE
        # status midflight
        mission = session.query(Mission).filter_by(mission_id=mission_id).first()
        mission.mission_status = "Mid Flight"
        session.commit()

        return drone_controller_thread



