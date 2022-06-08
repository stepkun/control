
from cmath import pi
import math

import rclpy

from PySide2.QtCore import QThreadPool, QRunnable, Slot

from .data_publisher import DataPublisher


class RosExecutor(QRunnable):
    '''
    Worker thread handling ros executor
    Inherits from QRunnable to handle worker thread setup, signals and wrap-up.

    '''
    def __init__(self, data_store):
        super(RosExecutor, self).__init__()
        self.data = data_store
        self.do_run = True

        self.threadpool = QThreadPool()
        self.threadpool.start(self)

    @Slot()  # QtCore.Slot
    def run(self):
        #print("publisher started")
        rclpy.init(args=None)

        publisher = DataPublisher(self.data)
 
        while self.do_run:
            rclpy.spin_once(publisher)

        #print("publisher ended")

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        publisher.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self.do_run = False
