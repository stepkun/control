
from PySide2.QtCore import QThreadPool, QRunnable, Slot

class ControlsWorker(QRunnable):
    '''
    Worker thread handling querying of physical control devices.
    Inherits from QRunnable to handle worker thread setup, signals and wrap-up.

    :param controls: The set of control devices this worker shall regularly query
    :type controls: ControlSet

    '''

    def __init__(self, controls):
        super(ControlsWorker, self).__init__()

        self.controls = controls
        self.do_run = True



    @Slot()  # QtCore.Slot
    def run(self):
        #print("control set started")
        while self.do_run:
            # query all devices in that list
            for v in self.controls.values():
                v.read()
            
        #print("control set ended")


    def stop(self):
        self.do_run = False



class ControlSet:
    def __init__(self, data_store):
        self.data = data_store
        # dictionary for the controls
        self.controls = {}
        self.threadpool = QThreadPool()
        self.devices = ControlsWorker(self.controls)
        self.threadpool.start(self.devices)


    def add(self, control):
        self.controls[control.id] = control
        self.data.dict[control.id] = control.get_value_set()
        control.signals.data.connect(self.data.receive_data)


    def stop(self):
        self.devices.stop()