
from PySide2.QtCore import QSize, Qt
from PySide2.QtGui import QPalette, QColor
from PySide2.QtWidgets import QMainWindow, QWidget

from .toolbar import ToolBar
from .actions.configure_action import ConfigureAction
from .actions.activate_action import ActivateAction
from .actions.exit_action import ExitAction
from .actions.dummy_action import DummyAction

class MainWindow(QMainWindow):
    def __init__(self, controls, ros_executor):
        super().__init__()

        self.worker1 = controls
        self.worker2 = ros_executor

        self.setWindowTitle("NodebotControl")
        self.setFixedSize(QSize(800, 480))
        # hide titlebar
        self.setWindowFlag(Qt.FramelessWindowHint)

        # left toolbar
        toolbar = ToolBar("LeftActions", self)
        self.addToolBar(Qt.LeftToolBarArea, toolbar)
        # add actions
        for i in range(0, 3):
            toolbar.addAction(DummyAction(self))  # dummy!!!
        toolbar.addAction(ConfigureAction(self, controls, ros_executor))
        toolbar.addAction(ActivateAction(self, controls, ros_executor))
        toolbar.addAction(ExitAction(self, controls, ros_executor))

        # right toolbar
        toolbar = ToolBar("RightActions", self)
        self.addToolBar(Qt.RightToolBarArea, toolbar)
        # add actions
        for i in range(0, 6):
            toolbar.addAction(DummyAction(self))  # dummy!!!

        # main area
        area = QWidget()
        area.setFixedSize(QSize(640, 480))
        # background color
        area.setAutoFillBackground(True)
        palette = area.palette()
        palette.setColor(QPalette.Window, QColor("Black"))
        area.setPalette(palette)
        self.setCentralWidget(area)

