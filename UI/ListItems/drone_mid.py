# Form implementation generated from reading ui file 'drone_mid.ui'
#
# Created by: PyQt6 UI code generator 6.6.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setVerticalSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.battery_label = QtWidgets.QLabel(parent=Form)
        self.battery_label.setObjectName("battery_label")
        self.gridLayout.addWidget(self.battery_label, 2, 1, 1, 1)
        self.model_label = QtWidgets.QLabel(parent=Form)
        self.model_label.setObjectName("model_label")
        self.gridLayout.addWidget(self.model_label, 1, 1, 1, 1)
        self.battery_text = QtWidgets.QLabel(parent=Form)
        self.battery_text.setText("")
        self.battery_text.setObjectName("battery_text")
        self.gridLayout.addWidget(self.battery_text, 2, 3, 1, 2)
        self.status_text = QtWidgets.QLabel(parent=Form)
        self.status_text.setText("")
        self.status_text.setObjectName("status_text")
        self.gridLayout.addWidget(self.status_text, 3, 3, 1, 1)
        self.colon_3 = QtWidgets.QLabel(parent=Form)
        self.colon_3.setObjectName("colon_3")
        self.gridLayout.addWidget(self.colon_3, 3, 2, 1, 1)
        self.colon_1 = QtWidgets.QLabel(parent=Form)
        self.colon_1.setObjectName("colon_1")
        self.gridLayout.addWidget(self.colon_1, 1, 2, 1, 1)
        self.model_text = QtWidgets.QLabel(parent=Form)
        self.model_text.setText("")
        self.model_text.setObjectName("model_text")
        self.gridLayout.addWidget(self.model_text, 1, 3, 1, 2)
        self.status_label = QtWidgets.QLabel(parent=Form)
        self.status_label.setObjectName("status_label")
        self.gridLayout.addWidget(self.status_label, 3, 1, 1, 1)
        self.colon_2 = QtWidgets.QLabel(parent=Form)
        self.colon_2.setObjectName("colon_2")
        self.gridLayout.addWidget(self.colon_2, 2, 2, 1, 1)
        self.id_text = QtWidgets.QLabel(parent=Form)
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        self.id_text.setFont(font)
        self.id_text.setObjectName("id_text")
        self.gridLayout.addWidget(self.id_text, 1, 0, 3, 1)
        self.gridLayout.setColumnStretch(4, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.battery_label.setText(_translate("Form", "Battery"))
        self.model_label.setText(_translate("Form", "Model"))
        self.colon_3.setText(_translate("Form", ":"))
        self.colon_1.setText(_translate("Form", ":"))
        self.status_label.setText(_translate("Form", "Status"))
        self.colon_2.setText(_translate("Form", ":"))
        self.id_text.setText(_translate("Form", "1"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
