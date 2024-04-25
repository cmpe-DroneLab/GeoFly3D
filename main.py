import sys

from PyQt6.QtWidgets import QApplication, QMainWindow, QListWidgetItem

import UI.main_design
from UI.database import Mission, session
from UI.Preflight1.pre1 import Pre1
from UI.Preflight2.pre2 import Pre2
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

        self.pre1.ui.btn_create_mission.clicked.connect(self.go_to_pre2)
        self.pre1.ui.btn_edit_mission.clicked.connect(self.go_to_pre2)
        self.pre1.ui.listWidget.itemDoubleClicked.connect(self.go_to_pre2)

        self.pre2.ui.btn_cancel.clicked.connect(self.go_to_pre1)
        self.pre2.ui.btn_save.clicked.connect(self.go_to_pre1)
        self.pre2.ui.btn_start.clicked.connect(self.go_to_pre3)
        self.pre3.ui.btn_return_back.clicked.connect(self.go_to_pre2)
        self.pre3.ui.btn_take_off.clicked.connect(self.go_to_mid)

    def go_to_pre1(self):
        self.pre1.refresh_mission_list()
        self.ui.stackedWidget.setCurrentIndex(0)

    def go_to_pre2(self, item):
        if isinstance(item, QListWidgetItem):
            mission_id = int(item.text().split(":")[1].split(",")[0].strip())  # Varolan görev
        else:
            new_mission = Mission(
                mission_status="Draft",
                center_lat=41.0859528,
                center_lon=29.0443435,
                coordinates=None,
                mission_drones=[],
                estimated_mission_time=0,
                actual_mission_time=0,
                required_battery_capacity=0,
                selected_area=0,
                scanned_area=0,
                altitude=100
            )
            session.add(new_mission)
            session.commit()
            mission_id = new_mission.mission_id
        self.pre2.load_mission(mission_id)
        self.ui.stackedWidget.setCurrentIndex(1)

    def go_to_pre3(self):
        self.pre2.coords_lon_lat = self.pre2.invert_coordinates(self.pre2.coords)

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

    def go_to_mid(self):

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
