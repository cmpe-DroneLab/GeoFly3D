import json
import sys

from PyQt6.QtGui import QIcon

import UI.main_design

from PyQt6.QtCore import QDateTime
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel
from UI.Preflight1.pre1 import Pre1
from UI.Preflight2.pre2 import Pre2
from UI.Preflight3.pre3 import Pre3
from UI.Midflight.mid import Mid
from UI.Postflight.post import Post
from UI.database import Mission, session, get_mission_by_drone_id, get_mission_by_id, get_drone_by_id
from UI.helpers import ServerThread, invert_coordinates, update_drone_status, update_drone_battery

from drone_controller import DroneController


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

        self.pre1.ui.btn_create_mission.clicked.connect(
            self.create_mission_clicked)
        self.pre1.ui.btn_edit_mission.clicked.connect(
            self.edit_mission_clicked)
        self.pre1.ui.listWidget.itemDoubleClicked.connect(
            self.edit_mission_clicked)
        self.pre1.mission_deleted.connect(self.delete_mission_clicked)

        self.pre2.ui.btn_cancel.clicked.connect(self.cancel_mission_clicked)
        self.pre2.ui.btn_start.clicked.connect(self.start_mission_clicked)

        self.pre3.drone_connect_click_signal.connect(self.connect_drone)
        self.pre3.ui.btn_return_back.clicked.connect(self.go_to_main_clicked)
        self.pre3.ui.btn_take_off.clicked.connect(self.take_off_clicked)

        self.mid.ui.btn_main.clicked.connect(self.go_to_main_clicked)
        self.mid.ui.btn_pause_resume.clicked.connect(self.pause_resume_clicked)
        self.mid.ui.btn_return_to_home.clicked.connect(self.rth_clicked)
        self.mid.ui.btn_land.clicked.connect(self.land_clicked)
        self.mid.emergency_rth_clicked.connect(self.emergency_rth_clicked)
        self.mid.emergency_land_clicked.connect(self.emergency_land_clicked)

        self.post.ui.btn_main.clicked.connect(self.go_to_main_clicked)
        self.post.ui.btn_download.clicked.connect(self.download_clicked)

        self.mission_threads = {}
        self.started = False

    # PRE1 to PRE2
    def create_mission_clicked(self):
        mission_id = 0
        self.pre2.load_mission(mission_id)
        self.ui.stackedWidget.setCurrentIndex(1)

    # PRE1 to PRE2/MID/POST
    def edit_mission_clicked(self):
        item = self.pre1.ui.listWidget.selectedItems()[0]
        mission_id = int(item.listWidget().itemWidget(item).findChild(QLabel, "id_text").text())
        mission_status = item.listWidget().itemWidget(item).findChild(QLabel, "status_text").text().lower()

        # PRE1 to PRE2
        if mission_status == "draft":
            self.pre2.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(1)
        # PRE1 to PRE3
        elif mission_status == "paused":
            self.pre3.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(2)
        # PRE1 to MID
        elif mission_status == "mid flight":
            self.mid.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(3)
        # PRE1 to POST
        elif mission_status == "post flight":
            self.post.load_mission(mission_id)
            self.ui.stackedWidget.setCurrentIndex(4)

    def delete_mission_clicked(self, mission_id):
        threads = self.mission_threads
        if mission_id in threads:
            while len(threads[mission_id]) > 0:
                controller_thread = threads.pop(mission_id)[mission_id]
                controller_thread.terminate()
            print(f"Mission {mission_id} Thread Aborted!")

        if len(threads.keys()) == 1:
            server_thread = threads.pop(0)
            server_thread.terminate()
            print(f"Server Thread Aborted!")
        print(self.mission_threads)

    # PRE2 to PRE1
    def cancel_mission_clicked(self):
        self.pre1.refresh_mission_list()
        self.pre1.refresh_general_map()
        self.ui.stackedWidget.setCurrentIndex(0)

    # PRE3 to PRE2
    def return_back_clicked(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    # TODO: PRE1 TO P3
    # MISSION THREAD

    # PRE2 to PRE3
    def start_mission_clicked(self):
        self.pre2.save_mission()

        if self.pre2.mission.mission_id not in self.mission_threads:
            self.mission_threads[self.pre2.mission.mission_id] = {}

        self.pre3.load_mission(self.pre2.mission.mission_id)
        self.ui.stackedWidget.setCurrentIndex(2)

    # PRE3 to MID
    def take_off_clicked(self):

        # coords = json.loads(self.pre3.mission.coordinates)
        # coords_lon_lat = invert_coordinates(coords)

        mission_id = self.pre3.mission.mission_id
        # vertices = coords_lon_lat
        # altitude = self.pre3.mission.altitude
        # gimbal_angle = self.pre3.mission.gimbal_angle
        # route_angle = self.pre3.mission.route_angle
        # rotated_route_angle = self.pre3.mission.rotated_route_angle

        self.mid.load_mission(mission_id)

        # mission_thread = self.mid.take_off(
        #     vertices=vertices,
        #     flight_altitude=altitude,
        #     mission_id=mission_id,
        #     gimbal_angle=gimbal_angle,
        #     route_angle=route_angle,
        #     rotated_route_angle=rotated_route_angle
        # )

        # mission_thread.finished.connect(self.scan_finished)

        self.mid.has_taken_off = True

        mission = get_mission_by_id(mission_id)
        mission.set_status("Mid Flight")
        mission.set_flight_start_time(QDateTime.currentDateTime().toString())

        for drone_id, drone_controller_thread in self.mission_threads[mission_id].items():
            drone = get_drone_by_id(drone_id)
            drone_controller_thread.started.connect(print)
            drone_controller_thread.progress_text.connect(print)
            drone_controller_thread.progress.connect(self.mid.last_visited_node_changed)
            drone_controller_thread.update_coord.connect(self.mid.drone_position_changed)
            drone_controller_thread.update_coord.connect(self.mid.update_live_data)
            drone_controller_thread.update_status.connect(update_drone_status)
            drone_controller_thread.update_status.connect(self.mid.update_live_data)
            drone_controller_thread.update_battery.connect(update_drone_battery)
            drone_controller_thread.update_battery.connect(self.mid.update_live_data)
            drone_controller_thread.start_mission((drone.path.last_visited_node.latitude, drone.path.last_visited_node.longitude))

        if 0 not in self.mission_threads:
            self.start_server()

        self.ui.stackedWidget.setCurrentIndex(3)

    # MID to POST Automatically
    def scan_finished(self, msg, mission_id):
        if "finished" not in self.mission_threads[mission_id]:
            self.mission_threads[mission_id]["finished"] = 1
        else:
            self.mission_threads[mission_id]["finished"] += 1
        if self.mission_threads[mission_id]["finished"] == len(self.mission_threads[mission_id]) - 1:

            mission = session.query(Mission).filter_by(mission_id=mission_id).first()
            mission.mission_status = "Post Flight"
            mission.flight_finish_time = QDateTime.currentDateTime().toString()
            mission.actual_mission_time = self.mid.ui.elapsed_time_value.text()
            session.commit()

            self.post.load_mission(mission_id)

            self.mid.has_taken_off = False
            if self.mid.timer.isActive():
                self.mid.timer.stop()
                self.mid.timer.disconnect()
            self.ui.stackedWidget.setCurrentIndex(4)

    # MID/POST to PRE1
    def go_to_main_clicked(self):
        # MID to PRE1
        if (self.ui.stackedWidget.currentIndex() == 3) and (self.mid.timer.isActive()):
            self.mid.timer.stop()
            self.mid.timer.disconnect()
        self.pre1.refresh_mission_list()
        self.pre1.refresh_general_map()
        self.ui.stackedWidget.setCurrentIndex(0)

    def connect_drone(self, drone_id):
        drone = get_drone_by_id(drone_id)
        mission = get_mission_by_drone_id(drone_id)
        coords = json.loads(drone.path.path_boundary)
        coords = invert_coordinates(coords)
        drone_controller_thread = DroneController(json.loads(drone.path.opt_route), json.loads(drone.path.rot_route), mission_id=mission.mission_id, drone_id=drone_id, drone_ip_address=drone.ip_address,
                                                  flight_altitude=mission.altitude,
                                                  gimbal_angle=mission.gimbal_angle, route_angle=mission.route_angle,
                                                  rotated_route_angle=mission.rotated_route_angle)

        drone_controller_thread.services_started.connect(
            self.on_services_started)
        drone_controller_thread.finished.connect(self.scan_finished)

        self.mission_threads[mission.mission_id][drone_id] = drone_controller_thread
        print(self.mission_threads)

        drone_controller_thread.start()

    def on_services_started(self, thread: DroneController):
        thread.connect_drone()

    def pause_resume_clicked(self):
        # TODO: CONTINUE
        if self.mid.ui.btn_land.isVisible():
            self.mid.ui.btn_land.setVisible(False)
            self.mid.ui.btn_return_to_home.setVisible(False)
            self.mid.ui.btn_pause_resume.setText("Pause")
            self.mid.ui.btn_pause_resume.setIcon(QIcon('./UI/Images/pause.png'))
            self.mid.mission.set_status("Mid Flight")

            for drone_id, drone_controller in self.mission_threads[self.mid.mission.mission_id].items():
                drone = get_drone_by_id(drone_id)
                drone_controller.start_mission((drone.path.last_visited_node.latitude, drone.path.last_visited_node.longitude))

        else:
            self.mid.ui.btn_land.setVisible(True)
            self.mid.ui.btn_return_to_home.setVisible(True)
            self.mid.ui.btn_pause_resume.setText("Resume")
            self.mid.ui.btn_pause_resume.setIcon(QIcon('./UI/Images/play.png'))
            self.mid.mission.set_status("Paused")

            for drone_controller in self.mission_threads[self.mid.mission.mission_id].values():
                drone_controller.pause_mission()

    def rth_clicked(self):
        for drone_controller in self.mission_threads[self.mid.mission.mission_id].values():
            drone_controller.rth_drone()

    def land_clicked(self):
        for drone_controller in self.mission_threads[self.mid.mission.mission_id].values():
            drone_controller.land_drone()

    def download_clicked(self):
        for drone_controller in self.mission_threads[self.mid.mission.mission_id].values():
            drone_controller.download_photos()
            self.mission_threads.pop(self.mid.mission.mission_id)[self.mid.mission.mission_id]

    def emergency_land_clicked(self, drone_id):
        for d_id, drone_controller in self.mission_threads[self.mid.mission.mission_id].items():
            if d_id == drone_id:
                drone_controller.pause_mission()
                drone_controller.land_drone()
                break

    def emergency_rth_clicked(self, drone_id):
        for d_id, drone_controller in self.mission_threads[self.mid.mission.mission_id].items():
            if d_id == drone_id:
                drone_controller.pause_mission()
                drone_controller.rth_drone()
                break

    # Method to start the HTTP server
    def start_server(self):
        server_thread = ServerThread()
        self.mission_threads[0] = server_thread
        server_thread.start()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
