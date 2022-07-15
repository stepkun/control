
from cmath import pi
import math
from typing import Optional
#from xmlrpc.client import boolean

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
        self.lc_publisher: Optional[DataPublisher] = None

        self.threadpool = QThreadPool()
        self.threadpool.start(self)

    @Slot()  # QtCore.Slot
    def run(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        self.lc_publisher = DataPublisher(self.data)
        executor.add_node(self.lc_publisher)


        while self.do_run:
            executor.spin_once(timeout_sec=0.1)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.lc_publisher.destroy_node()
        self.lc_publisher = None
        rclpy.shutdown()

    def stop(self):
        print("tell publisher to end")
        self.do_run = False

    def configure(self):
        self.lc_publisher.trigger_configure()

    def activate(self):
        self.lc_publisher.trigger_activate()

    def deactivate(self):
        self.lc_publisher.trigger_deactivate()

    def cleanup(self):
        self.lc_publisher.trigger_cleanup()

    def shutdown(self):
        self.lc_publisher.trigger_shutdown()