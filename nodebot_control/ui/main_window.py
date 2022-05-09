
from PySide2.QtCore import QSize, Qt
from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton

class MainWindow(QMainWindow):
    def __init__(self, controls, publisher):
        super().__init__()

        self.worker1 = controls
        self.worker2 = publisher

        self.setWindowTitle("NodebotControl")
        self.setFixedSize(QSize(800, 480))
        # hide titlebar
        self.setWindowFlag(Qt.FramelessWindowHint)

        button = QPushButton('Exit')
        button.clicked.connect(self.exit_clicked)

        self.setCentralWidget(button)


    def exit_clicked(self):
        try:
            self.worker1.stop()
            self.worker2.stop()
        except Exception:
            pass
        exit()
