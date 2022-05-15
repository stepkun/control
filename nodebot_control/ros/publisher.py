
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/nodebot1/cmd_vel', 10)
        self.cmd_cam_publisher = self.create_publisher(Twist, '/nodebot1/cmd_cam', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #print(self.data)
        msg = Twist()
        #dict = self.data.dict.copy()
        # publish cmd_vel from right stick
        # do transformation into robot x,y,z - axis
        msg.linear.x =   0.05 * self.data.dict['rightStick']['y']
        msg.angular.z = -0.25 * self.data.dict['rightStick']['z']
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.cmd_vel_publisher.publish(msg)

        # publish cmd_cam from left stick
        # do transformation into robot x,y,z - axis
        msg.angular.y = 90 + 40 * self.data.dict['leftStick']['y']
        msg.angular.z = 90 - 70 * self.data.dict['leftStick']['z']
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.cmd_vel_publisher.publish(msg)

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
