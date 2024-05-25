# Form implementation generated from reading ui file 'pre1_design.ui'
#
# Created by: PyQt6 UI code generator 6.6.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1024, 768)
        Form.setStyleSheet("QPushButton {padding:10px}")
        self.horizontalLayout = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.splitter = QtWidgets.QSplitter(parent=Form)
        self.splitter.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.splitter.setObjectName("splitter")
        self.layoutWidget = QtWidgets.QWidget(parent=self.splitter)
        self.layoutWidget.setObjectName("layoutWidget")
        self.v_lay_left = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.v_lay_left.setContentsMargins(0, 0, 15, 0)
        self.v_lay_left.setSpacing(10)
        self.v_lay_left.setObjectName("v_lay_left")
        self.label_header = QtWidgets.QLabel(parent=self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        self.label_header.setFont(font)
        self.label_header.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.label_header.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_header.setObjectName("label_header")
        self.v_lay_left.addWidget(self.label_header)
        self.label_description = QtWidgets.QLabel(parent=self.layoutWidget)
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_description.setFont(font)
        self.label_description.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_description.setObjectName("label_description")
        self.v_lay_left.addWidget(self.label_description)
        spacerItem = QtWidgets.QSpacerItem(20, 15, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.v_lay_left.addItem(spacerItem)
        self.gb_missions = QtWidgets.QGroupBox(parent=self.layoutWidget)
        self.gb_missions.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.gb_missions.setObjectName("gb_missions")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.gb_missions)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.listWidget = QtWidgets.QListWidget(parent=self.gb_missions)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout_2.addWidget(self.listWidget)
        self.btn_test_missions = QtWidgets.QPushButton(parent=self.gb_missions)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("./UI/Images/add.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_test_missions.setIcon(icon)
        self.btn_test_missions.setObjectName("btn_test_missions")
        self.verticalLayout_2.addWidget(self.btn_test_missions)
        self.h_lay_drone_buttons = QtWidgets.QHBoxLayout()
        self.h_lay_drone_buttons.setObjectName("h_lay_drone_buttons")
        self.btn_create_mission = QtWidgets.QPushButton(parent=self.gb_missions)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("./UI/Images/add.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_create_mission.setIcon(icon)
        self.btn_create_mission.setObjectName("btn_create_mission")
        self.h_lay_drone_buttons.addWidget(self.btn_create_mission)
        self.btn_duplicate_mission = QtWidgets.QPushButton(parent=self.gb_missions)
        self.btn_duplicate_mission.setEnabled(False)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("./UI/Images/copy.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_duplicate_mission.setIcon(icon1)
        self.btn_duplicate_mission.setObjectName("btn_duplicate_mission")
        self.h_lay_drone_buttons.addWidget(self.btn_duplicate_mission)
        self.btn_delete_mission = QtWidgets.QPushButton(parent=self.gb_missions)
        self.btn_delete_mission.setEnabled(False)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("./UI/Images/delete.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_delete_mission.setIcon(icon2)
        self.btn_delete_mission.setObjectName("btn_delete_mission")
        self.h_lay_drone_buttons.addWidget(self.btn_delete_mission)
        self.verticalLayout_2.addLayout(self.h_lay_drone_buttons)
        self.v_lay_left.addWidget(self.gb_missions)
        self.g_lay_buttons = QtWidgets.QGridLayout()
        self.g_lay_buttons.setObjectName("g_lay_buttons")
        self.btn_edit_mission = QtWidgets.QPushButton(parent=self.layoutWidget)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("./UI/Images/arrow-right.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_edit_mission.setIcon(icon3)
        self.btn_edit_mission.setObjectName("btn_edit_mission")
        self.g_lay_buttons.addWidget(self.btn_edit_mission, 1, 0, 1, 1)
        self.v_lay_left.addLayout(self.g_lay_buttons)
        self.layoutWidget1 = QtWidgets.QWidget(parent=self.splitter)
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.v_lay_right = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.v_lay_right.setContentsMargins(0, 0, 0, 0)
        self.v_lay_right.setObjectName("v_lay_right")
        self.horizontalLayout.addWidget(self.splitter)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label_header.setText(_translate("Form", "GeoFly3D Main Page"))
        self.label_description.setText(_translate("Form", "Create a new mission or select from the list."))
        self.gb_missions.setTitle(_translate("Form", "Mission List"))
        self.btn_test_missions.setText(_translate("Form", "Create Test Missions"))
        self.btn_create_mission.setText(_translate("Form", "Create"))
        self.btn_duplicate_mission.setText(_translate("Form", "Duplicate"))
        self.btn_delete_mission.setText(_translate("Form", "Delete"))
        self.btn_edit_mission.setText(_translate("Form", "Continue with Selected Mission"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
