# Form implementation generated from reading ui file 'mid_design.ui'
#
# Created by: PyQt6 UI code generator 6.6.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.
import os

from PyQt6 import QtCore, QtGui, QtWidgets


def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath("./UI")

    return os.path.join(base_path, relative_path)


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1024, 768)
        Form.setStyleSheet(open(resource_path('./style.qss'), "r").read())

        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.v_lay_left = QtWidgets.QVBoxLayout()
        self.v_lay_left.setSpacing(10)
        self.v_lay_left.setObjectName("v_lay_left")
        self.g_lay_mission_header = QtWidgets.QGridLayout()
        self.g_lay_mission_header.setObjectName("g_lay_mission_header")
        self.id_label = QtWidgets.QLabel(parent=Form)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.id_label.setFont(font)
        self.id_label.setObjectName("id_label")
        self.g_lay_mission_header.addWidget(self.id_label, 0, 2, 1, 1)
        self.label_header = QtWidgets.QLabel(parent=Form)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.label_header.setFont(font)
        self.label_header.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.label_header.setObjectName("label_header")
        self.g_lay_mission_header.addWidget(self.label_header, 0, 0, 1, 2)
        self.v_lay_left.addLayout(self.g_lay_mission_header)
        self.label_header1 = QtWidgets.QLabel(parent=Form)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.label_header1.setFont(font)
        self.label_header1.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.label_header1.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_header1.setObjectName("label_header1")
        self.v_lay_left.addWidget(self.label_header1)
        self.gb_drones = QtWidgets.QGroupBox(parent=Form)
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
        self.line = QtWidgets.QFrame(parent=self.gb_drones)
        self.line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_2.addWidget(self.line)
        self.listWidget = QtWidgets.QListWidget(parent=self.gb_drones)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout_2.addWidget(self.listWidget)
        self.v_lay_left.addWidget(self.gb_drones)
        self.gb_mission = QtWidgets.QGroupBox(parent=Form)
        self.gb_mission.setTitle("")
        self.gb_mission.setObjectName("gb_mission")
        self.gridLayout = QtWidgets.QGridLayout(self.gb_mission)
        self.gridLayout.setObjectName("gridLayout")
        self.selected_area_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_label.setObjectName("selected_area_label")
        self.gridLayout.addWidget(self.selected_area_label, 2, 0, 1, 1)
        self.scanned_area_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.scanned_area_label.setObjectName("scanned_area_label")
        self.gridLayout.addWidget(self.scanned_area_label, 4, 0, 1, 1)
        self.scanned_area_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.scanned_area_unit.setObjectName("scanned_area_unit")
        self.gridLayout.addWidget(self.scanned_area_unit, 4, 2, 1, 1)
        self.selected_area_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_unit.setObjectName("selected_area_unit")
        self.gridLayout.addWidget(self.selected_area_unit, 2, 2, 1, 1)
        self.mission_time_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_unit.setObjectName("mission_time_unit")
        self.gridLayout.addWidget(self.mission_time_unit, 5, 2, 1, 1)
        self.line_2 = QtWidgets.QFrame(parent=self.gb_mission)
        self.line_2.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line_2.setObjectName("line_2")
        self.gridLayout.addWidget(self.line_2, 1, 0, 1, 3)
        self.mission_time_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_label.setObjectName("mission_time_label")
        self.gridLayout.addWidget(self.mission_time_label, 5, 0, 1, 1)
        self.progress_bar = QtWidgets.QProgressBar(parent=self.gb_mission)
        self.progress_bar.setProperty("value", 24)
        self.progress_bar.setObjectName("progress_bar")
        self.gridLayout.addWidget(self.progress_bar, 7, 0, 1, 3)
        self.label_mission_information = QtWidgets.QLabel(parent=self.gb_mission)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.label_mission_information.setFont(font)
        self.label_mission_information.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_mission_information.setObjectName("label_mission_information")
        self.gridLayout.addWidget(self.label_mission_information, 0, 0, 1, 3)
        self.elapsed_time_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.elapsed_time_label.setObjectName("elapsed_time_label")
        self.gridLayout.addWidget(self.elapsed_time_label, 6, 0, 1, 1)
        self.elapsed_time_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.elapsed_time_value.setText("")
        self.elapsed_time_value.setObjectName("elapsed_time_value")
        self.gridLayout.addWidget(self.elapsed_time_value, 6, 1, 1, 1)
        self.elapsed_time_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.elapsed_time_unit.setObjectName("elapsed_time_unit")
        self.gridLayout.addWidget(self.elapsed_time_unit, 6, 2, 1, 1)
        self.mission_time_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_value.setText("")
        self.mission_time_value.setObjectName("mission_time_value")
        self.gridLayout.addWidget(self.mission_time_value, 5, 1, 1, 1)
        self.selected_area_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_value.setText("")
        self.selected_area_value.setObjectName("selected_area_value")
        self.gridLayout.addWidget(self.selected_area_value, 2, 1, 1, 1)
        self.scanned_area_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.scanned_area_value.setText("")
        self.scanned_area_value.setObjectName("scanned_area_value")
        self.gridLayout.addWidget(self.scanned_area_value, 4, 1, 1, 1)
        self.v_lay_left.addWidget(self.gb_mission)
        self.g_lay_buttons = QtWidgets.QGridLayout()
        self.g_lay_buttons.setObjectName("g_lay_buttons")
        self.btn_main = QtWidgets.QPushButton(parent=Form)
        self.btn_main.setObjectName("btn_main")
        self.g_lay_buttons.addWidget(self.btn_main, 0, 0, 1, 1)
        self.btn_return_to_home = QtWidgets.QPushButton(parent=Form)
        self.btn_return_to_home.setObjectName("btn_return_to_home")
        self.g_lay_buttons.addWidget(self.btn_return_to_home, 0, 1, 1, 2)
        self.v_lay_left.addLayout(self.g_lay_buttons)
        self.horizontalLayout_2.addLayout(self.v_lay_left)
        self.v_lay_right = QtWidgets.QVBoxLayout()
        self.v_lay_right.setObjectName("v_lay_right")
        self.horizontalLayout_2.addLayout(self.v_lay_right)
        self.horizontalLayout_2.setStretch(1, 4)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.id_label.setText(_translate("Form", "0"))
        self.label_header.setText(_translate("Form", "MISSION  #"))
        self.label_header1.setText(_translate("Form", "FLIGHT MONITORING"))
        self.label_drones.setText(_translate("Form", "Drone List"))
        self.selected_area_label.setText(_translate("Form", "Selected Area:"))
        self.scanned_area_label.setText(_translate("Form", "Scanned Area:"))
        self.scanned_area_unit.setText(_translate("Form", "m2"))
        self.selected_area_unit.setText(_translate("Form", "m2"))
        self.mission_time_unit.setText(_translate("Form", "min"))
        self.mission_time_label.setText(_translate("Form", "Estimated Mission Time:"))
        self.label_mission_information.setText(_translate("Form", "Mission Information"))
        self.elapsed_time_label.setText(_translate("Form", "Elapsed Mission Time:"))
        self.elapsed_time_unit.setText(_translate("Form", "min"))
        self.btn_main.setText(_translate("Form", "ᐊ Go Main"))
        self.btn_return_to_home.setText(_translate("Form", "Return to Home ᐁ"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
