import UI.Postflight.post_design

from PyQt6.QtCore import QSize
from PyQt6.QtWidgets import QWidget, QListWidgetItem
from UI.database import get_mission_by_id
from UI.ListItems.drone_mid import Ui_Form
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
    def load_mission(self, mission_id):

        if mission_id == 0:
            exit(-1)
        else:
            self.mission = get_mission_by_id(mission_id)

        self.mission_id = self.mission.mission_id
        self.project_folder = self.mission.project_folder

        # Set mission id in the header box
        self.ui.gb_mission.setTitle("Mission # " + str(self.mission.mission_id))

        # Set mission information box
        self.ui.scanned_area_value.setText(str(self.mission.selected_area))
        self.ui.estimated_mission_time_value.setText(str(self.mission.estimated_mission_time))
        self.ui.actual_mission_time_value.setText(str(self.mission.actual_mission_time))

        # Load drones
        self.refresh_drone_list()

    # Gets all matching drones from the database, adds them to the Drone List
    def refresh_drone_list(self):
        self.ui.listWidget.clear()

        # Find all drones matching the Mission
        drones = self.mission.get_drones()
        for drone in drones:
            self.add_drone_to_list(drone)

    # Adds given drone to the Drone List
    def add_drone_to_list(self, drone):
        new_drone_item = QListWidgetItem()
        new_drone_widget = QWidget()
        new_drone_ui = Ui_Form()
        new_drone_ui.setupUi(new_drone_widget)
        new_drone_ui.id_text.setText(str(drone.drone_id))
        new_drone_ui.model_text.setText(drone.model)
        new_drone_ui.battery_text.setText(str(drone.battery_no))

        # Calculate the height of the new_drone_widget
        new_drone_widget.adjustSize()
        widget_height = new_drone_widget.sizeHint().height()

        # Set the size hint for the item
        new_drone_item.setSizeHint(QSize(self.ui.listWidget.lineWidth(), widget_height))

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
