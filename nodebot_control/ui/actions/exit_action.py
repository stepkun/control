
import os

from PySide2.QtGui import  QIcon
from PySide2.QtWidgets import QAction

class ExitAction(QAction):
    def __init__(self, parent, controls, publisher):
        super().__init__(QIcon(os.path.dirname(__file__)+"/exit.png"),"Exit", parent)

        self.worker1 = controls
        self.worker2 = publisher

        self.setToolTip("Exit Application")
        self.triggered.connect(self.trigger)


    def trigger(self):
        try:
            self.worker1.stop()
            self.worker2.stop()
        except Exception:
            pass
        exit()
