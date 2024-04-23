# Form implementation generated from reading ui file 'preflight2.ui'
#
# Created by: PyQt6 UI code generator 6.6.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.

import os
from PyQt6 import QtCore, QtGui, QtWidgets
# Define function to import external files when using PyInstaller.
def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath("./UI")

    return os.path.join(base_path, relative_path)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1100, 700)
        MainWindow.setStyleSheet(open(resource_path('style.qss'), "r").read())

        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.v_lay_left = QtWidgets.QVBoxLayout()
        self.v_lay_left.setSpacing(10)
        self.v_lay_left.setObjectName("v_lay_left")
        self.labe_header = QtWidgets.QLabel(parent=self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.labe_header.setFont(font)
        self.labe_header.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.labe_header.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.labe_header.setObjectName("labe_header")
        self.v_lay_left.addWidget(self.labe_header)
        self.gb_drones = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.gb_drones.setTitle("")
        self.gb_drones.setObjectName("gb_drones")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.gb_drones)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_drones = QtWidgets.QLabel(parent=self.gb_drones)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.label_drones.setFont(font)
        self.label_drones.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_drones.setObjectName("label_drones")
        self.verticalLayout_2.addWidget(self.label_drones)
        self.line_3 = QtWidgets.QFrame(parent=self.gb_drones)
        self.line_3.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout_2.addWidget(self.line_3)
        self.listWidget = QtWidgets.QListWidget(parent=self.gb_drones)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout_2.addWidget(self.listWidget)
        self.h_lay_drone_buttons = QtWidgets.QHBoxLayout()
        self.h_lay_drone_buttons.setObjectName("h_lay_drone_buttons")
        self.btn_delete_drone = QtWidgets.QPushButton(parent=self.gb_drones)
        self.btn_delete_drone.setObjectName("btn_delete_drone")
        self.h_lay_drone_buttons.addWidget(self.btn_delete_drone)
        self.btn_add_drone = QtWidgets.QPushButton(parent=self.gb_drones)
        self.btn_add_drone.setObjectName("btn_add_drone")
        self.h_lay_drone_buttons.addWidget(self.btn_add_drone)
        self.verticalLayout_2.addLayout(self.h_lay_drone_buttons)
        self.v_lay_left.addWidget(self.gb_drones)
        self.gb_resolution = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.gb_resolution.setObjectName("gb_resolution")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gb_resolution)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.slider_altitude = QtWidgets.QSlider(parent=self.gb_resolution)
        self.slider_altitude.setMinimum(10)
        self.slider_altitude.setMaximum(500)
        self.slider_altitude.setProperty("value", 100)
        self.slider_altitude.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.slider_altitude.setObjectName("slider_altitude")
        self.gridLayout_2.addWidget(self.slider_altitude, 2, 0, 1, 1)
        self.label_altitude_value = QtWidgets.QLabel(parent=self.gb_resolution)
        self.label_altitude_value.setObjectName("label_altitude_value")
        self.gridLayout_2.addWidget(self.label_altitude_value, 2, 1, 1, 1)
        self.label_altitude = QtWidgets.QLabel(parent=self.gb_resolution)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.label_altitude.setFont(font)
        self.label_altitude.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_altitude.setObjectName("label_altitude")
        self.gridLayout_2.addWidget(self.label_altitude, 0, 0, 1, 2)
        self.line = QtWidgets.QFrame(parent=self.gb_resolution)
        self.line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line.setObjectName("line")
        self.gridLayout_2.addWidget(self.line, 1, 0, 1, 2)
        self.v_lay_left.addWidget(self.gb_resolution)
        self.gb_mission = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.gb_mission.setTitle("")
        self.gb_mission.setObjectName("gb_mission")
        self.gridLayout = QtWidgets.QGridLayout(self.gb_mission)
        self.gridLayout.setObjectName("gridLayout")
        self.selected_area_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_value.setText("")
        self.selected_area_value.setObjectName("selected_area_value")
        self.gridLayout.addWidget(self.selected_area_value, 4, 1, 1, 1)
        self.batt_required_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_value.setText("")
        self.batt_required_value.setObjectName("batt_required_value")
        self.gridLayout.addWidget(self.batt_required_value, 5, 1, 1, 1)
        self.batt_provided_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_value.setText("")
        self.batt_provided_value.setObjectName("batt_provided_value")
        self.gridLayout.addWidget(self.batt_provided_value, 6, 1, 1, 1)
        self.batt_required_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_label.setObjectName("batt_required_label")
        self.gridLayout.addWidget(self.batt_required_label, 5, 0, 1, 1)
        self.selected_area_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_label.setObjectName("selected_area_label")
        self.gridLayout.addWidget(self.selected_area_label, 4, 0, 1, 1)
        self.mission_time_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_value.setText("")
        self.mission_time_value.setObjectName("mission_time_value")
        self.gridLayout.addWidget(self.mission_time_value, 2, 1, 1, 1)
        self.batt_provided_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_label.setObjectName("batt_provided_label")
        self.gridLayout.addWidget(self.batt_provided_label, 6, 0, 1, 1)
        self.mission_time_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_label.setObjectName("mission_time_label")
        self.gridLayout.addWidget(self.mission_time_label, 2, 0, 1, 1)
        self.mission_time_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_unit.setObjectName("mission_time_unit")
        self.gridLayout.addWidget(self.mission_time_unit, 2, 2, 1, 1)
        self.label_mission_information = QtWidgets.QLabel(parent=self.gb_mission)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.label_mission_information.setFont(font)
        self.label_mission_information.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_mission_information.setObjectName("label_mission_information")
        self.gridLayout.addWidget(self.label_mission_information, 0, 0, 1, 3)
        self.line_2 = QtWidgets.QFrame(parent=self.gb_mission)
        self.line_2.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line_2.setObjectName("line_2")
        self.gridLayout.addWidget(self.line_2, 1, 0, 1, 3)
        self.selected_area_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_unit.setObjectName("selected_area_unit")
        self.gridLayout.addWidget(self.selected_area_unit, 4, 2, 1, 1)
        self.batt_required_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_unit.setObjectName("batt_required_unit")
        self.gridLayout.addWidget(self.batt_required_unit, 5, 2, 1, 1)
        self.batt_provided_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_unit.setObjectName("batt_provided_unit")
        self.gridLayout.addWidget(self.batt_provided_unit, 6, 2, 1, 1)
        self.v_lay_left.addWidget(self.gb_mission)
        self.h_lay_mission_buttons = QtWidgets.QHBoxLayout()
        self.h_lay_mission_buttons.setObjectName("h_lay_mission_buttons")
        self.btn_delete = QtWidgets.QPushButton(parent=self.centralwidget)
        self.btn_delete.setObjectName("btn_delete")
        self.h_lay_mission_buttons.addWidget(self.btn_delete)
        self.btn_cancel = QtWidgets.QPushButton(parent=self.centralwidget)
        self.btn_cancel.setObjectName("btn_cancel")
        self.h_lay_mission_buttons.addWidget(self.btn_cancel)
        self.btn_save = QtWidgets.QPushButton(parent=self.centralwidget)
        self.btn_save.setObjectName("btn_save")
        self.h_lay_mission_buttons.addWidget(self.btn_save)
        self.v_lay_left.addLayout(self.h_lay_mission_buttons)
        self.btn_start = QtWidgets.QPushButton(parent=self.centralwidget)
        self.btn_start.setObjectName("btn_start")
        self.v_lay_left.addWidget(self.btn_start)
        self.horizontalLayout_2.addLayout(self.v_lay_left)
        self.v_lay_right = QtWidgets.QVBoxLayout()
        self.v_lay_right.setObjectName("v_lay_right")
        self.horizontalLayout_2.addLayout(self.v_lay_right)
        self.horizontalLayout_2.setStretch(1, 4)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(parent=MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 588, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.labe_header.setText(_translate("MainWindow", "MISSION PLANNING"))
        self.label_drones.setText(_translate("MainWindow", "Drone List"))
        self.btn_delete_drone.setText(_translate("MainWindow", "Delete Drone"))
        self.btn_add_drone.setText(_translate("MainWindow", "Add Drone"))
        self.label_altitude_value.setText(_translate("MainWindow", "100"))
        self.label_altitude.setText(_translate("MainWindow", "Select Flight Altitude"))
        self.batt_required_label.setText(_translate("MainWindow", "Required Battery Capacity:"))
        self.selected_area_label.setText(_translate("MainWindow", "Selected Area:"))
        self.batt_provided_label.setText(_translate("MainWindow", "Provided Battery Capacity:"))
        self.mission_time_label.setText(_translate("MainWindow", "Estimated Mission Time:"))
        self.mission_time_unit.setText(_translate("MainWindow", "min"))
        self.label_mission_information.setText(_translate("MainWindow", "Mission Information"))
        self.selected_area_unit.setText(_translate("MainWindow", "m2"))
        self.batt_required_unit.setText(_translate("MainWindow", "min"))
        self.batt_provided_unit.setText(_translate("MainWindow", "min"))
        self.btn_delete.setText(_translate("MainWindow", "Delete"))
        self.btn_cancel.setText(_translate("MainWindow", "Cancel"))
        self.btn_save.setText(_translate("MainWindow", "Save"))
        self.btn_start.setText(_translate("MainWindow", "START THE MISSION"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())