
import os

from PySide2.QtGui import  QIcon
from PySide2.QtWidgets import QAction

class ConfigureAction(QAction):
    def __init__(self, parent, controls, ros_executor):
        super().__init__(QIcon(os.path.dirname(__file__)+"/configure.png"),"Configure", parent)

        self.ros_executor = ros_executor
        self.isConfigured = False
        self.setToolTip("Configure")
        self.triggered.connect(self.trigger)

    def trigger(self):
        if self.isConfigured:
            self.ros_executor.cleanup()
            self.isConfigured = False
        else:
            self.ros_executor.configure()
            self.isConfigured = True
