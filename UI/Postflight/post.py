from PyQt6.QtWidgets import QWidget

import UI.Postflight.post_design


class Post(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Postflight.post_design.Ui_Form()
        self.ui.setupUi(self)

