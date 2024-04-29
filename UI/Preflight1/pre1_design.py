# Form implementation generated from reading ui file 'pre1_design.ui'
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
        self.labe_header = QtWidgets.QLabel(parent=Form)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.labe_header.setFont(font)
        self.labe_header.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.labe_header.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.labe_header.setObjectName("labe_header")
        self.v_lay_left.addWidget(self.labe_header)
        self.gb_missions = QtWidgets.QGroupBox(parent=Form)
        self.gb_missions.setTitle("")
        self.gb_missions.setObjectName("gb_missions")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.gb_missions)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_missions = QtWidgets.QLabel(parent=self.gb_missions)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        self.label_missions.setFont(font)
        self.label_missions.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_missions.setObjectName("label_missions")
        self.verticalLayout_2.addWidget(self.label_missions)
        self.line = QtWidgets.QFrame(parent=self.gb_missions)
        self.line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_2.addWidget(self.line)
        self.listWidget = QtWidgets.QListWidget(parent=self.gb_missions)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout_2.addWidget(self.listWidget)
        self.btn_test_missions = QtWidgets.QPushButton(parent=self.gb_missions)
        self.btn_test_missions.setObjectName("btn_test_missions")
        self.verticalLayout_2.addWidget(self.btn_test_missions)
        self.h_lay_drone_buttons = QtWidgets.QHBoxLayout()
        self.h_lay_drone_buttons.setObjectName("h_lay_drone_buttons")
        self.btn_create_mission = QtWidgets.QPushButton(parent=self.gb_missions)
        self.btn_create_mission.setObjectName("btn_create_mission")
        self.h_lay_drone_buttons.addWidget(self.btn_create_mission)
        self.btn_duplicate_mission = QtWidgets.QPushButton(parent=self.gb_missions)
        self.btn_duplicate_mission.setEnabled(False)
        self.btn_duplicate_mission.setObjectName("btn_duplicate_mission")
        self.h_lay_drone_buttons.addWidget(self.btn_duplicate_mission)
        self.btn_delete_mission = QtWidgets.QPushButton(parent=self.gb_missions)
        self.btn_delete_mission.setEnabled(False)
        self.btn_delete_mission.setObjectName("btn_delete_mission")
        self.h_lay_drone_buttons.addWidget(self.btn_delete_mission)
        self.verticalLayout_2.addLayout(self.h_lay_drone_buttons)
        self.v_lay_left.addWidget(self.gb_missions)
        self.g_lay_buttons = QtWidgets.QGridLayout()
        self.g_lay_buttons.setObjectName("g_lay_buttons")
        self.btn_edit_mission = QtWidgets.QPushButton(parent=Form)
        self.btn_edit_mission.setObjectName("btn_edit_mission")
        self.g_lay_buttons.addWidget(self.btn_edit_mission, 1, 0, 1, 1)
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
        self.labe_header.setText(_translate("Form", "MAIN PAGE"))
        self.label_missions.setText(_translate("Form", "Mission List"))
        self.btn_test_missions.setText(_translate("Form", "Create Test Missions"))
        self.btn_create_mission.setText(_translate("Form", "Create"))
        self.btn_duplicate_mission.setText(_translate("Form", "Duplicate"))
        self.btn_delete_mission.setText(_translate("Form", "Delete"))
        self.btn_edit_mission.setText(_translate("Form", "Continue with Selected Mission ᐅ"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
