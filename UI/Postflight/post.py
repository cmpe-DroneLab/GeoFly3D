from PyQt6.QtCore import QSize
from PyQt6.QtWidgets import QWidget, QListWidgetItem

import UI.Postflight.post_design
from UI.database import session, Mission, Drone
from UI.drone import Ui_Form

from orthophoto_generator import OrthophotoGenerator


class Post(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Postflight.post_design.Ui_Form()
        self.ui.setupUi(self)

        self.mission = None
        self.mission_id = None
        self.project_folder = None
        self.is_clicked = False
        self.threads = {}

        self.ui.btn_process.clicked.connect(self.generate_orthophoto)

    # Loads mission information from database into relevant fields
    def load_mission(self, project_folder, mission_id):

        if mission_id == 0:
            exit(-1)
        else:
            self.mission = session.query(Mission).filter_by(mission_id=mission_id).first()

        self.mission_id = self.mission.mission_id
        self.project_folder = project_folder

        # Set mission id in the header box
        self.ui.id_label.setText(str(self.mission_id))

        # Set mission information box
        self.ui.scanned_area_value.setText(str(self.mission.scanned_area))
        self.ui.mission_time_value.setText(str(self.mission.estimated_mission_time))

        # Load drones
        self.refresh_drone_list()

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()

        # Find all drones matching the Mission
        drones = session.query(Drone).filter_by(mission_id=self.mission_id).all()
        for drone in drones:
            self.add_drone_to_list(drone)

    # Adds given drone to the Drone List
    def add_drone_to_list(self, drone):
        new_drone_item = QListWidgetItem()
        new_drone_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), 80))
        new_drone_widget = QWidget()
        new_drone_ui = Ui_Form()
        new_drone_ui.setupUi(new_drone_widget)
        new_drone_ui.id_text.setText(str(drone.drone_id))
        new_drone_ui.model_text.setText(drone.model)
        new_drone_ui.spare_batt_text.setText(str(drone.battery_no))
        self.ui.listWidget.addItem(new_drone_item)
        self.ui.listWidget.setItemWidget(new_drone_item, new_drone_widget)

    def generate_orthophoto(self):
        if self.is_clicked:
            return

        ortho_gen_thread = OrthophotoGenerator(project_folder=self.project_folder, orthophoto_resolution=1)

        ortho_gen_thread.started.connect(print)
        ortho_gen_thread.progress_text.connect(print)
        ortho_gen_thread.finished.connect(self.finish_process)

        self.threads[1] = ortho_gen_thread
        ortho_gen_thread.start()
        self.is_clicked = True

    def finish_process(self, msg):
        print(msg)
        self.threads.clear()
        self.is_clicked = False