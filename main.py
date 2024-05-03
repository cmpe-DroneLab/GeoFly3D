import json
import sys
import random
import UI.main_design

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QMainWindow
from UI.Preflight1.pre1 import Pre1
from UI.Preflight2.pre2 import Pre2
from UI.Preflight3.pre3 import Pre3
from UI.Midflight.mid import Mid
from UI.Postflight.post import Post
from UI.database import session, Mission
from UI.helpers import ServerThread, calculate_sw_ne_points, invert_coordinates


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = UI.main_design.Ui_MainWindow()
        self.ui.setupUi(self)

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
        self.pre1.mission_deleted.connect(self.delete_mission_clicked)

        self.pre2.ui.btn_cancel.clicked.connect(self.cancel_mission_clicked)
        self.pre2.ui.btn_start.clicked.connect(self.start_mission_clicked)

        self.pre3.ui.btn_return_back.clicked.connect(self.return_back_clicked)
        self.pre3.ui.btn_take_off.clicked.connect(self.take_off_clicked)

        self.mid.ui.btn_main.clicked.connect(self.go_to_main_clicked)
        self.post.ui.btn_main.clicked.connect(self.go_to_main_clicked)

    # PRE1 to PRE2
    def create_mission_clicked(self):
        mission_id = 0
        self.pre2.load_mission(mission_id)
        self.ui.stackedWidget.setCurrentIndex(1)

    # PRE1 to PRE2/MID/POST
    def edit_mission_clicked(self):
        selected_item = self.pre1.ui.listWidget.selectedItems()[0]
        mission_id = int(selected_item.text().split(":")[1].split(",")[0].strip())
        mission_status = selected_item.text().split(":")[2].strip().lower()

        # PRE1 to PRE2
        if mission_status == "draft":
            self.pre2.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(1)
        # PRE1 to MID
        elif mission_status == "mid flight":
            self.mid.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(3)
        # PRE1 to POST
        elif mission_status == "post flight":
            mission = session.query(Mission).filter_by(mission_id=mission_id).first()
            self.post.load_mission(mission.project_folder, mission_id)
            self.ui.stackedWidget.setCurrentIndex(4)

    def delete_mission_clicked(self, mission_id):
        threads = self.mid.threads
        if mission_id in threads:
            mission_thread = threads.pop(mission_id)
            mission_thread.terminate()
            print(f"Mission {mission_id} Thread Aborted!")

        if len(threads.keys()) == 1:
            server_thread = threads.pop(0)
            server_thread.terminate()
            print(f"Server Thread Aborted!")
        print(self.mid.threads)

    # PRE2 to PRE1
    def cancel_mission_clicked(self):
        self.pre1.refresh_mission_list()
        self.pre1.refresh_general_map()
        self.ui.stackedWidget.setCurrentIndex(0)

    # PRE3 to PRE2
    def return_back_clicked(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    # PRE2 to PRE3
    def start_mission_clicked(self):
        self.pre2.save_mission()
        self.pre3.ui.batt_provided_value.setText(self.pre2.ui.batt_provided_value.text())
        self.pre3.load_mission(self.pre2.mission.mission_id)
        self.ui.stackedWidget.setCurrentIndex(2)

    # PRE3 to MID
    def take_off_clicked(self):

        coords = json.loads(self.pre3.mission.coordinates)
        coords_lon_lat = invert_coordinates(coords)

        mission_id = self.pre3.mission.mission_id
        vertices = coords_lon_lat
        altitude = self.pre3.mission.altitude
        gimbal_angle = self.pre3.mission.gimbal_angle
        route_angle = self.pre3.mission.route_angle
        rotated_route_angle = self.pre3.mission.rotated_route_angle

        self.mid.load_mission(mission_id)

        mission_thread = self.mid.take_off(
            vertices=vertices,
            flight_altitude=altitude,
            mission_id=mission_id,
            gimbal_angle=gimbal_angle,
            route_angle=route_angle,
            rotated_route_angle=rotated_route_angle
        )
        mission_thread.finished.connect(self.scan_finished)

        if 0 not in self.mid.threads:
            self.start_server()
        self.simulate_drone_flight()

        self.ui.stackedWidget.setCurrentIndex(3)

    # MID to POST Automatically
    def scan_finished(self, msg, project_folder, mission_id):
        mission = session.query(Mission).filter_by(mission_id=mission_id).first()
        mission.mission_status = "Post Flight"
        mission.project_folder = project_folder
        session.commit()

        self.post.load_mission(project_folder, mission_id)
        self.mid.threads.clear()
        self.mid.has_taken_off = False
        self.ui.stackedWidget.setCurrentIndex(4)

    # MID/POST to PRE1
    def go_to_main_clicked(self):
        self.pre1.refresh_mission_list()
        self.pre1.refresh_general_map()
        self.ui.stackedWidget.setCurrentIndex(0)

    def simulate_drone_flight(self):
        print("Drone started flying")
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulate_position_update)
        self.timer.start(1000)  # Update position every 1000 milliseconds (1 second)

    def simulate_position_update(self):

        coords = json.loads(self.mid.mission.coordinates)
        sw_point, ne_point = calculate_sw_ne_points(coords)

        # Simulate random latitude and longitude
        latitude = random.uniform(sw_point[0], ne_point[0])
        longitude = random.uniform(sw_point[1], ne_point[1])
        print("GPS coordinate: ", latitude, longitude)
        # Emit signal with simulated position update
        self.mid.drone_position_updated.emit(latitude, longitude)

    # Method to start the HTTP server
    def start_server(self):
        server_thread = ServerThread()
        self.mid.threads[0] = server_thread
        server_thread.start()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()