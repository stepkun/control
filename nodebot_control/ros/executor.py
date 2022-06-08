
from cmath import pi
import math

import rclpy

from PySide2.QtCore import QThreadPool, QRunnable, Slot

from .data_publisher import DataPublisher


class RosPublisherWorker(QRunnable):
    '''
    Worker thread handling ros publishing
    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    '''
    def __init__(self, data_store):
        super(RosPublisherWorker, self).__init__()
        self.data = data_store

        self.do_run = True


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



class RosExecutor:
    def __init__(self, data_store):
        self.threadpool = QThreadPool()
        self.publisher = RosPublisherWorker(data_store)
        self.threadpool.start(self.publisher)


    def stop(self):
        self.publisher.stop()
