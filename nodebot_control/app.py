
from PySide2.QtWidgets import QApplication

from .ui.main_window import MainWindow
from .models.data_store import DataStore
from .devices.control_set import ControlSet
from .devices.joystick import Joystick
from .ros.publisher import RosPublisher

def main(args=None):
    # setup application environment
    app = QApplication([])

    # central data store
    data = DataStore()

    # setup application components
    controls = ControlSet(data)
    controls.add(Joystick('leftStick', 0x48))
    controls.add(Joystick('rightStick', 0x49))

    publisher = RosPublisher(data)

    #setup main window
    window = MainWindow(controls, publisher)
    window.show()

    # run application event loop
    app.exec_()

if __name__ == '__main__':
    main()
