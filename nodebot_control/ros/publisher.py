
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from PySide2.QtCore import QThreadPool, QRunnable, Slot

class DataPublisher(Node):
    '''
    ROS2 publisher node
    Inherits from ROS2's class Node.

    '''
    def __init__(self, data_store):
        super().__init__('data_publisher')
        self.data = data_store
        self.publisher_ = self.create_publisher(Twist, '/nodebot1/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #print(self.data)
        msg = Twist()
        dict = self.data.dict.copy()
        msg.linear.x = 1.0 * dict['leftStick']['x']
        msg.linear.y = 1.0 * dict['leftStick']['y']
        msg.linear.z = 1.0 * dict['leftStick']['z']
        msg.angular.x = 1.0 * dict['rightStick']['x']
        msg.angular.y = 1.0 * dict['rightStick']['y']
        msg.angular.z = 1.0 * dict['rightStick']['z']
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

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



class RosPublisher:
    def __init__(self, data_store):
        self.threadpool = QThreadPool()
        self.publisher = RosPublisherWorker(data_store)
        self.threadpool.start(self.publisher)


    def stop(self):
        self.publisher.stop()
