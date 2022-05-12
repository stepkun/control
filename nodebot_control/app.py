
from PySide2.QtWidgets import QApplication

from .ui.main_window import MainWindow
from .models.data_store import DataStore
from .devices.control_set import ControlSet
from .devices.joystick import Joystick
from .ros.publisher import RosPublisher

def main(args=None):
    try:
        # setup application environment
        app = QApplication([])

        # central data store
        data = DataStore()

        # setup application components, running in a separate thread
        controls = ControlSet(data)
        controls.add(Joystick('leftStick', 0x48))
        controls.add(Joystick('rightStick', 0x49))

        # setup ros publisher running in a separate thread
        publisher = RosPublisher(data)

        #setup main window
        window = MainWindow(controls, publisher)
        window.show()

        # run application event loop
        app.exec_()

    # final exeption handler to ensure proper shutdown of threads
    except(Exception) as e:
        print(e)
        try:
            controls.stop()
        except:
            pass
        try:
            publisher.stop()
        except:
            pass
        exit()


if __name__ == '__main__':
    main()
