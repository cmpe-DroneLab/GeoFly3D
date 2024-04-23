from PyQt6 import QtCore, QtWidgets
from PyQt6.QtGui import QIcon
import os
import sys


class Ui_drone_dialog(object):

    def setupUi(self, drone_dialog):
        drone_dialog.setStyleSheet(open('style.qss', "r").read())
        drone_dialog.setObjectName("drone_dialog")
        drone_dialog.setWindowTitle("Drone Details")
        drone_dialog.resize(300, 200)
        drone_dialog.setMinimumSize(QtCore.QSize(300, 200))
        drone_dialog.setMaximumSize(QtCore.QSize(300, 200))

        #### hl_buttons >> SAVE BUTTON | CANCEL BUTTON
        self.hl_buttons = QtWidgets.QHBoxLayout()
        self.hl_buttons.setObjectName("hl_buttons")

        ## spacerItem1
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding,
                                            QtWidgets.QSizePolicy.Policy.Minimum)
        ## btn_save
        self.btn_save = QtWidgets.QPushButton(parent=drone_dialog)
        self.btn_save.setObjectName("btn_save")
        self.btn_save.setText("Save")
        ## btn_cancel
        self.btn_cancel = QtWidgets.QPushButton(parent=drone_dialog)
        self.btn_cancel.setObjectName("btn_cancel")
        self.btn_cancel.setText("Cancel")
        ## layout order
        self.hl_buttons.addItem(spacerItem1)
        self.hl_buttons.addWidget(self.btn_save)
        self.hl_buttons.addWidget(self.btn_cancel)

        #### verticalLayout >> MODEL | SPARE | HL_BUTTONS
        self.verticalLayout = QtWidgets.QVBoxLayout(drone_dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        ## label_model
        self.label_model = QtWidgets.QLabel(parent=drone_dialog)
        self.label_model.setObjectName("label_model")
        self.label_model.setText("Select Your Drone :")
        ## combo_model
        self.combo_model = QtWidgets.QComboBox(parent=drone_dialog)
        self.combo_model.setObjectName("combo_model")
        self.combo_model.addItem("Parrot Anafi 4k")
        self.combo_model.addItem("Parrot Anafi USA")
        ## label_spare_batt
        self.label_spare_batt = QtWidgets.QLabel(parent=drone_dialog)
        self.label_spare_batt.setObjectName("label_spare_batt")
        self.label_spare_batt.setText("Enter Number of Spare Battery :")
        ## text_spare_batt
        self.spin_spare_batt = QtWidgets.QSpinBox(parent=drone_dialog)
        self.spin_spare_batt.setObjectName("spin_spare_batt")

        ## spacerItem
        spacerItem = QtWidgets.QSpacerItem(20, 24, QtWidgets.QSizePolicy.Policy.Minimum,
                                           QtWidgets.QSizePolicy.Policy.Expanding)
        ## layout order
        self.verticalLayout.addWidget(self.label_model)
        self.verticalLayout.addWidget(self.combo_model)
        self.verticalLayout.addWidget(self.label_spare_batt)
        self.verticalLayout.addWidget(self.spin_spare_batt)

        self.verticalLayout.addItem(spacerItem)
        self.verticalLayout.addLayout(self.hl_buttons)

        QtCore.QMetaObject.connectSlotsByName(drone_dialog)
        self.btn_save.clicked.connect(drone_dialog.accept)
        self.btn_cancel.clicked.connect(drone_dialog.reject)