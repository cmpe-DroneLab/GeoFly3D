# Form implementation generated from reading ui file 'pre3_design.ui'
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
        self.label_header = QtWidgets.QLabel(parent=Form)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.label_header.setFont(font)
        self.label_header.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.label_header.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_header.setObjectName("label_header")
        self.v_lay_left.addWidget(self.label_header)
        self.gb_mission = QtWidgets.QGroupBox(parent=Form)
        self.gb_mission.setTitle("")
        self.gb_mission.setObjectName("gb_mission")
        self.gridLayout = QtWidgets.QGridLayout(self.gb_mission)
        self.gridLayout.setObjectName("gridLayout")
        self.batt_provided_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_unit.setObjectName("batt_provided_unit")
        self.gridLayout.addWidget(self.batt_provided_unit, 7, 3, 1, 1)
        self.batt_required_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_value.setText("")
        self.batt_required_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.batt_required_value.setObjectName("batt_required_value")
        self.gridLayout.addWidget(self.batt_required_value, 6, 2, 1, 1)
        self.label_mission_information = QtWidgets.QLabel(parent=self.gb_mission)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setItalic(True)
        self.label_mission_information.setFont(font)
        self.label_mission_information.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.label_mission_information.setObjectName("label_mission_information")
        self.gridLayout.addWidget(self.label_mission_information, 0, 0, 1, 1)
        self.batt_provided_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_value.setText("")
        self.batt_provided_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.batt_provided_value.setObjectName("batt_provided_value")
        self.gridLayout.addWidget(self.batt_provided_value, 7, 2, 1, 1)
        self.id_label = QtWidgets.QLabel(parent=self.gb_mission)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setItalic(True)
        self.id_label.setFont(font)
        self.id_label.setObjectName("id_label")
        self.gridLayout.addWidget(self.id_label, 0, 2, 1, 1)
        self.line_2 = QtWidgets.QFrame(parent=self.gb_mission)
        self.line_2.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line_2.setObjectName("line_2")
        self.gridLayout.addWidget(self.line_2, 1, 0, 1, 4)
        self.selected_area_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_value.setText("")
        self.selected_area_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.selected_area_value.setObjectName("selected_area_value")
        self.gridLayout.addWidget(self.selected_area_value, 2, 2, 1, 1)
        self.selected_area_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_label.setObjectName("selected_area_label")
        self.gridLayout.addWidget(self.selected_area_label, 2, 0, 1, 1)
        self.mission_time_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_value.setText("")
        self.mission_time_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.mission_time_value.setObjectName("mission_time_value")
        self.gridLayout.addWidget(self.mission_time_value, 3, 2, 1, 1)
        self.mission_time_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_unit.setObjectName("mission_time_unit")
        self.gridLayout.addWidget(self.mission_time_unit, 3, 3, 1, 1)
        self.batt_provided_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_label.setObjectName("batt_provided_label")
        self.gridLayout.addWidget(self.batt_provided_label, 7, 0, 1, 1)
        self.mission_time_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_label.setObjectName("mission_time_label")
        self.gridLayout.addWidget(self.mission_time_label, 3, 0, 1, 1)
        self.selected_area_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_unit.setObjectName("selected_area_unit")
        self.gridLayout.addWidget(self.selected_area_unit, 2, 3, 1, 1)
        self.batt_required_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_label.setObjectName("batt_required_label")
        self.gridLayout.addWidget(self.batt_required_label, 6, 0, 1, 1)
        self.batt_required_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_unit.setObjectName("batt_required_unit")
        self.gridLayout.addWidget(self.batt_required_unit, 6, 3, 1, 1)
        self.colon_1 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_1.setObjectName("colon_1")
        self.gridLayout.addWidget(self.colon_1, 2, 1, 1, 1)
        self.colon_2 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_2.setObjectName("colon_2")
        self.gridLayout.addWidget(self.colon_2, 3, 1, 1, 1)
        self.colon_3 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_3.setObjectName("colon_3")
        self.gridLayout.addWidget(self.colon_3, 6, 1, 1, 1)
        self.colon_4 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_4.setObjectName("colon_4")
        self.gridLayout.addWidget(self.colon_4, 7, 1, 1, 1)
        self.gridLayout.setColumnStretch(2, 1)
        self.v_lay_left.addWidget(self.gb_mission)
        self.gb_drones = QtWidgets.QGroupBox(parent=Form)
        self.gb_drones.setTitle("")
        self.gb_drones.setObjectName("gb_drones")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.gb_drones)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_drones = QtWidgets.QLabel(parent=self.gb_drones)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setItalic(True)
        self.label_drones.setFont(font)
        self.label_drones.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_drones.setObjectName("label_drones")
        self.verticalLayout_2.addWidget(self.label_drones)
        self.listWidget = QtWidgets.QListWidget(parent=self.gb_drones)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout_2.addWidget(self.listWidget)
        self.v_lay_left.addWidget(self.gb_drones)
        self.g_lay_buttons = QtWidgets.QGridLayout()
        self.g_lay_buttons.setObjectName("g_lay_buttons")
        self.btn_take_off = QtWidgets.QPushButton(parent=Form)
        self.btn_take_off.setObjectName("btn_take_off")
        self.g_lay_buttons.addWidget(self.btn_take_off, 0, 1, 1, 2)
        self.btn_return_back = QtWidgets.QPushButton(parent=Form)
        self.btn_return_back.setObjectName("btn_return_back")
        self.g_lay_buttons.addWidget(self.btn_return_back, 0, 0, 1, 1)
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
        self.label_header.setText(_translate("Form", "Pre-Flight Checking"))
        self.batt_provided_unit.setText(_translate("Form", "min"))
        self.label_mission_information.setText(_translate("Form", "Mission #"))
        self.id_label.setText(_translate("Form", "0"))
        self.selected_area_label.setText(_translate("Form", "Selected Area"))
        self.mission_time_unit.setText(_translate("Form", "min"))
        self.batt_provided_label.setText(_translate("Form", "Provided Battery Capacity"))
        self.mission_time_label.setText(_translate("Form", "Estimated Mission Time"))
        self.selected_area_unit.setText(_translate("Form", "m²"))
        self.batt_required_label.setText(_translate("Form", "Required Battery Capacity"))
        self.batt_required_unit.setText(_translate("Form", "min"))
        self.colon_1.setText(_translate("Form", ":"))
        self.colon_2.setText(_translate("Form", ":"))
        self.colon_3.setText(_translate("Form", ":"))
        self.colon_4.setText(_translate("Form", ":"))
        self.label_drones.setText(_translate("Form", "Drone List"))
        self.btn_take_off.setText(_translate("Form", "Take Off ᐅ"))
        self.btn_return_back.setText(_translate("Form", "ᐊ Go Back"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
