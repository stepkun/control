
import os

from PySide2.QtGui import  QIcon
from PySide2.QtWidgets import QAction

class DummyAction(QAction):
    def __init__(self, parent):
        super().__init__(QIcon(os.path.dirname(__file__)+"/dummy.png"),"Dummy", parent)

        self.setToolTip("No action")
        self.triggered.connect(self.trigger)


    def trigger(self):
        pass
