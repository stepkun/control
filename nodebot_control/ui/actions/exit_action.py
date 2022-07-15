
import os

from PySide2.QtGui import  QIcon
from PySide2.QtWidgets import QAction

class ExitAction(QAction):
    def __init__(self, parent, controls, ros_executor):
        super().__init__(QIcon(os.path.dirname(__file__)+"/exit.png"),"Exit", parent)

        self.worker1 = controls
        self.worker2 = ros_executor

        self.setToolTip("Exit Application")
        self.triggered.connect(self.trigger)


    def trigger(self):
        try:
            self.worker2.shutdown()
            self.worker1.stop()
            self.worker2.stop()
        except Exception:
            pass
        exit()
