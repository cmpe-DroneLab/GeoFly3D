from PyQt6.QtWidgets import QWidget

import UI.Postflight.post_design

from orthophoto_generator import OrthophotoGenerator

class Post(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.ui = UI.Postflight.post_design.Ui_Form()
        self.ui.setupUi(self)

        self.ui.btn_process.clicked.connect(self.generate_orthophoto)

        self.is_clicked = False
        self.threads = {}

    def setup(self, project_folder):
        self.project_folder = project_folder

    def generate_orthophoto(self):
        if self.is_clicked:
            return
        
        ortho_gen_thread = OrthophotoGenerator(project_folder=self.project_folder, orthophoto_resolution=1)

        ortho_gen_thread.started.connect(print)
        ortho_gen_thread.progress_text.connect(print)
        ortho_gen_thread.finished.connect(self.finish_process)

        self.threads[1] = ortho_gen_thread
        ortho_gen_thread.start()
        self.is_clicked = True
 
    def finish_process(self, msg):
        print(msg)
        self.threads.clear()
        self.is_clicked = False
