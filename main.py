import sys

from PyQt6.QtWidgets import QApplication, QMainWindow

import UI.main_design
from UI.Preflight1.pre1 import Pre1
from UI.Preflight2.pre2 import Pre2, invert_coordinates
from UI.Preflight3.pre3 import Pre3
from UI.Midflight.mid import Mid
from UI.Postflight.post import Post

from UI.database import session, Mission


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = UI.main_design.Ui_MainWindow()
        self.ui.setupUi(self)

        # Drop and Recreate all tables
        # Base.metadata.drop_all(engine)
        # Base.metadata.create_all(engine)

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
            self.go_to_post("", project_folder=mission.project_folder,mission_id=mission_id)
            self.ui.stackedWidget.setCurrentIndex(4)

    # PRE2 to PRE1
    def cancel_mission_clicked(self):
        self.pre1.refresh_mission_list()
        self.ui.stackedWidget.setCurrentIndex(0)

    # PRE2 to PRE1
    def save_mission_clicked(self):
        self.pre1.refresh_mission_list()
        self.ui.stackedWidget.setCurrentIndex(0)

    # PRE3 to PRE2
    def return_back_clicked(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    # PRE2 to PRE3
    def start_mission_clicked(self):
        self.pre2.save_mission()
        self.pre3.ui.batt_provided_value.setText(self.pre2.ui.batt_provided_value.text())
        self.pre3.load_mission(self.pre2.mission_id)
        self.ui.stackedWidget.setCurrentIndex(2)

    # PRE3 to MID
    def take_off_clicked(self):
        self.mid.load_mission(self.pre3.mission_id)
        vertices = self.pre3.coords_lon_lat
        altitude = self.pre3.mission.altitude

        mission_thread = self.mid.take_off(vertices=vertices, flight_altitude=altitude, mission_id=self.pre3.mission_id)
        mission_thread.finished.connect(self.scan_finished)

        self.ui.stackedWidget.setCurrentIndex(3)

    def scan_finished(self, msg, project_folder, mission_id):
        print(msg)
        self.mid.threads.clear()
        self.mid.has_taken_off = False
        self.post.load_mission(project_folder, mission_id)

        # DB UPDATE
        # mission_id post_flight
        mission = session.query(Mission).filter_by(mission_id=mission_id).first()
        mission.mission_status = "Post Flight"
        mission.project_folder = project_folder
        session.commit()

        self.ui.stackedWidget.setCurrentIndex(4)

    # MID/POST to PRE1
    def go_to_main_clicked(self):
        self.pre1.refresh_mission_list()
        self.ui.stackedWidget.setCurrentIndex(0)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
