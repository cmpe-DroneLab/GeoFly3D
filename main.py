import sys

from PyQt6.QtWidgets import QApplication, QMainWindow, QListWidgetItem

import UI.main_design
from UI.database import Mission, session
from UI.Preflight1.pre1 import Pre1
from UI.Preflight2.pre2 import Pre2, invert_coordinates
from UI.Preflight3.pre3 import Pre3
from UI.Midflight.mid import Mid
from UI.Postflight.post import Post

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = UI.main_design.Ui_MainWindow()
        self.ui.setupUi(self)

        # Drop and Recreate all tables
        #Base.metadata.drop_all(engine)
        #Base.metadata.create_all(engine)

        self.pre1 = Pre1()
        self.pre2 = Pre2()
        self.pre3 = Pre3()
        self.mid = Mid()
        self.post = Post()

        self.ui.stackedWidget.addWidget(self.pre1)
        self.ui.stackedWidget.addWidget(self.pre2)
        self.ui.stackedWidget.addWidget(self.pre3)
        self.ui.stackedWidget.addWidget(self.mid)
        self.ui.stackedWidget.addWidget(self.post)

        self.pre1.ui.btn_create_mission.clicked.connect(self.create_mission_clicked)
        self.pre1.ui.btn_edit_mission.clicked.connect(self.edit_mission_clicked)
        self.pre1.ui.listWidget.itemDoubleClicked.connect(self.edit_mission_clicked)

        self.pre2.ui.btn_cancel.clicked.connect(self.cancel_mission_clicked)
        self.pre2.ui.btn_save.clicked.connect(self.save_mission_clicked)
        self.pre2.ui.btn_start.clicked.connect(self.start_mission_clicked)

        self.pre3.ui.btn_return_back.clicked.connect(self.return_back_clicked)
        self.pre3.ui.btn_take_off.clicked.connect(self.take_off_clicked)

    def create_mission_clicked(self):
        mission_id = 0
        self.pre2.load_mission(mission_id)
        self.ui.stackedWidget.setCurrentIndex(1)

    def edit_mission_clicked(self):
        selected_item = self.pre1.ui.listWidget.selectedItems()[0]
        mission_id = int(selected_item.text().split(":")[1].split(",")[0].strip())
        mission_status = selected_item.text().split(":")[2].strip().lower()

        if mission_status == "draft":
            self.pre2.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(1)
        elif mission_status == "post_flight":
            print("Post")
            self.ui.stackedWidget.setCurrentIndex(4)

    def cancel_mission_clicked(self):
        self.pre1.refresh_mission_list()
        self.ui.stackedWidget.setCurrentIndex(0)

    def save_mission_clicked(self):
        self.pre1.refresh_mission_list()
        self.ui.stackedWidget.setCurrentIndex(0)

    def return_back_clicked(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    def start_mission_clicked(self):

        self.pre2.save_mission()
        self.pre2.coords_lon_lat = invert_coordinates(self.pre2.coords)

        # Copy mission information
        self.pre3.ui.mission_time_value.setText(self.pre2.ui.mission_time_value.text())
        self.pre3.ui.selected_area_value.setText(self.pre2.ui.selected_area_value.text())
        self.pre3.ui.batt_required_value.setText(self.pre2.ui.batt_required_value.text())
        self.pre3.ui.batt_provided_value.setText(self.pre2.ui.batt_provided_value.text())

        # Pass the route and altitude and create map
        altitude_val = self.pre2.ui.slider_altitude.value()
        route_coords = self.pre2.coords_lon_lat[:-1]
        self.pre3.setup_map(route_coords, altitude_val)

        self.ui.stackedWidget.setCurrentIndex(2)

    def take_off_clicked(self):

        vertices = self.pre3.coords
        altitude = self.pre3.altitude

        mission_thread = self.mid.take_off(vertices=vertices, flight_altitude=altitude)
        mission_thread.finished.connect(self.go_to_post)

        self.ui.stackedWidget.setCurrentIndex(3)

    def go_to_post(self, msg, project_folder):
        print(msg)
        self.mid.threads.clear()
        self.mid.has_taken_off = False
        self.post.setup(project_folder)

        self.ui.stackedWidget.setCurrentIndex(4)
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
