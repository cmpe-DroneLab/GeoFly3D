from PyQt6.QtWidgets import QWidget

import UI.Midflight.mid_design


class Mid(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Midflight.mid_design.Ui_Form()
        self.ui.setupUi(self)

