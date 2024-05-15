# Form implementation generated from reading ui file 'pre3_design.ui'
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
        Form.setStyleSheet("QPushButton { padding:10px;}")
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
        self.gb_mission = QtWidgets.QGroupBox(parent=self.layoutWidget)
        self.gb_mission.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.gb_mission.setObjectName("gb_mission")
        self.gridLayout = QtWidgets.QGridLayout(self.gb_mission)
        self.gridLayout.setObjectName("gridLayout")
        self.selected_area_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_value.setText("")
        self.selected_area_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.selected_area_value.setObjectName("selected_area_value")
        self.gridLayout.addWidget(self.selected_area_value, 0, 2, 1, 1)
        self.batt_required_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_value.setText("")
        self.batt_required_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.batt_required_value.setObjectName("batt_required_value")
        self.gridLayout.addWidget(self.batt_required_value, 4, 2, 1, 1)
        self.mission_time_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_label.setObjectName("mission_time_label")
        self.gridLayout.addWidget(self.mission_time_label, 1, 0, 1, 1)
        self.colon_4 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_4.setObjectName("colon_4")
        self.gridLayout.addWidget(self.colon_4, 5, 1, 1, 1)
        self.batt_provided_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_label.setObjectName("batt_provided_label")
        self.gridLayout.addWidget(self.batt_provided_label, 5, 0, 1, 1)
        self.selected_area_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_unit.setObjectName("selected_area_unit")
        self.gridLayout.addWidget(self.selected_area_unit, 0, 3, 1, 1)
        self.batt_required_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_label.setObjectName("batt_required_label")
        self.gridLayout.addWidget(self.batt_required_label, 4, 0, 1, 1)
        self.batt_provided_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_value.setText("")
        self.batt_provided_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.batt_provided_value.setObjectName("batt_provided_value")
        self.gridLayout.addWidget(self.batt_provided_value, 5, 2, 1, 1)
        self.mission_time_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_unit.setObjectName("mission_time_unit")
        self.gridLayout.addWidget(self.mission_time_unit, 1, 3, 1, 1)
        self.batt_provided_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_provided_unit.setObjectName("batt_provided_unit")
        self.gridLayout.addWidget(self.batt_provided_unit, 5, 3, 1, 1)
        self.colon_2 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_2.setObjectName("colon_2")
        self.gridLayout.addWidget(self.colon_2, 1, 1, 1, 1)
        self.colon_1 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_1.setObjectName("colon_1")
        self.gridLayout.addWidget(self.colon_1, 0, 1, 1, 1)
        self.mission_time_value = QtWidgets.QLabel(parent=self.gb_mission)
        self.mission_time_value.setText("")
        self.mission_time_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.mission_time_value.setObjectName("mission_time_value")
        self.gridLayout.addWidget(self.mission_time_value, 1, 2, 1, 1)
        self.colon_3 = QtWidgets.QLabel(parent=self.gb_mission)
        self.colon_3.setObjectName("colon_3")
        self.gridLayout.addWidget(self.colon_3, 4, 1, 1, 1)
        self.selected_area_label = QtWidgets.QLabel(parent=self.gb_mission)
        self.selected_area_label.setObjectName("selected_area_label")
        self.gridLayout.addWidget(self.selected_area_label, 0, 0, 1, 1)
        self.batt_required_unit = QtWidgets.QLabel(parent=self.gb_mission)
        self.batt_required_unit.setObjectName("batt_required_unit")
        self.gridLayout.addWidget(self.batt_required_unit, 4, 3, 1, 1)
        self.gridLayout.setColumnStretch(2, 1)
        self.v_lay_left.addWidget(self.gb_mission)
        self.gb_drones = QtWidgets.QGroupBox(parent=self.layoutWidget)
        self.gb_drones.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.gb_drones.setObjectName("gb_drones")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.gb_drones)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.listWidget = QtWidgets.QListWidget(parent=self.gb_drones)
        self.listWidget.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout_2.addWidget(self.listWidget)
        self.v_lay_left.addWidget(self.gb_drones)
        self.g_lay_buttons = QtWidgets.QGridLayout()
        self.g_lay_buttons.setObjectName("g_lay_buttons")
        self.btn_return_back = QtWidgets.QPushButton(parent=self.layoutWidget)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("./UI/Images/home.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_return_back.setIcon(icon)
        self.btn_return_back.setObjectName("btn_return_back")
        self.g_lay_buttons.addWidget(self.btn_return_back, 0, 0, 1, 1)
        self.btn_take_off = QtWidgets.QPushButton(parent=self.layoutWidget)
        self.btn_take_off.setEnabled(False)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("./UI/Images/arrow-up.png"), QtGui.QIcon.Mode.Normal, QtGui.QIcon.State.Off)
        self.btn_take_off.setIcon(icon1)
        self.btn_take_off.setObjectName("btn_take_off")
        self.g_lay_buttons.addWidget(self.btn_take_off, 0, 1, 1, 1)
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
        self.label_header.setText(_translate("Form", "Pre-Flight Checking"))
        self.label_description.setText(_translate("Form", "Add GCS point to the map and connect drone(s)."))
        self.gb_mission.setTitle(_translate("Form", "Mission #0"))
        self.mission_time_label.setText(_translate("Form", "Estimated Mission Time"))
        self.colon_4.setText(_translate("Form", ":"))
        self.batt_provided_label.setText(_translate("Form", "Provided Battery Capacity"))
        self.selected_area_unit.setText(_translate("Form", "m²"))
        self.batt_required_label.setText(_translate("Form", "Required Battery Capacity"))
        self.mission_time_unit.setText(_translate("Form", "min"))
        self.batt_provided_unit.setText(_translate("Form", "min"))
        self.colon_2.setText(_translate("Form", ":"))
        self.colon_1.setText(_translate("Form", ":"))
        self.colon_3.setText(_translate("Form", ":"))
        self.selected_area_label.setText(_translate("Form", "Selected Area"))
        self.batt_required_unit.setText(_translate("Form", "min"))
        self.gb_drones.setTitle(_translate("Form", "Drone List"))
        self.btn_return_back.setText(_translate("Form", "Home"))
        self.btn_take_off.setText(_translate("Form", "Take Off"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
