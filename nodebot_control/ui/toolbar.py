
from PySide2.QtCore import QSize
from PySide2.QtWidgets import QToolBar


class ToolBar(QToolBar):
    def __init__(self, name, parent):
        super().__init__(name, parent)

        icon_size = QSize(64,64)

        self.setIconSize(icon_size)
        self.setMovable(False)
        self.setFloatable(False)

        self.num_actions = 0

    def addAction(self, action):
        if self.num_actions > 0:
            self.addSeparator()
        super().addAction(action)
        self.num_actions += 1