
import os

from PySide2.QtGui import  QIcon
from PySide2.QtWidgets import QAction

class ActivateAction(QAction):
    def __init__(self, parent, controls, ros_executor):
        super().__init__(QIcon(os.path.dirname(__file__)+"/activate.png"),"Activate", parent)

        self.ros_executor = ros_executor
        self.isActive = False
        self.setToolTip("Activate/Deactivate")
        self.triggered.connect(self.trigger)


    def trigger(self):
        if self.isActive:
            self.ros_executor.deactivate()
            self.isActive = False
        else:
            self.ros_executor.activate()
            self.isActive = True
