import sys

from PyQt6.QtWidgets import QApplication, QMainWindow

import UI.main_design
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

    # PRE1 to PRE2
    def create_mission_clicked(self):
        mission_id = 0
        self.pre2.load_mission(mission_id)
        self.ui.stackedWidget.setCurrentIndex(1)

    # PRE1 to PRE2
    def edit_mission_clicked(self):
        selected_item = self.pre1.ui.listWidget.selectedItems()[0]
        mission_id = int(selected_item.text().split(":")[1].split(",")[0].strip())
        self.pre2.load_mission(mission_id)
        self.ui.stackedWidget.setCurrentIndex(1)

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
        vertices = self.pre3.coords
        altitude = self.pre3.mission.altitude

        mission_thread = self.mid.take_off(vertices=vertices, flight_altitude=altitude)
        mission_thread.finished.connect(self.go_to_post)

        self.ui.stackedWidget.setCurrentIndex(3)

    # MID to POST
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
