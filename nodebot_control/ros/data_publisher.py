from cmath import pi
import math

from typing import Optional

import rclpy

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion


class DataPublisher(Node):
    '''
    ROS2 publisher node
    Inherits from ROS2's class Node.

    '''
    def __init__(self, data_store):
        self.scale = [math.pi/2] * 3
        self.data = data_store
        self.cmd_vel_publisher: Optional[Publisher] = None
        self.cmd_cam_publisher: Optional[Publisher] = None
        self.timer: Optional[Timer] = None
        super().__init__('data_publisher')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        # command for robot speed
        self.topic_vel = '/nodebot1/cmd_vel'
        self.cmd_vel_publisher = self.create_publisher(Twist, self.topic_vel, 10)
        # command for camera orientation
        self.topic_cam = '/nodebot1/cmd_cam'
        self.cmd_cam_publisher = self.create_publisher(Quaternion, self.topic_cam, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:

        self.get_logger().info("on_activate() is called.")

        # the timer interval is 0.05 seconds to have no recognizable latency
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publishing_callback)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")

        # stop publishing
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')

        # remove publisher and timer
        self.cleanup()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')

        # remove publisher and timer
        self.cleanup()

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_error() is called.")
        return TransitionCallbackReturn.SUCCESS

    def cleanup(self):
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.cmd_cam_publisher is not None:
            self.destroy_publisher(self.cmd_cam_publisher)
            self.cmd_cam_publisher = None
        if self.cmd_vel_publisher is not None:
            self.destroy_publisher(self.cmd_vel_publisher)
            self.cmd_vel_publisher = None

    def publishing_callback(self):
        # publish cmd_vel from right stick
        msg_vel = Twist()
        # do transformation into robot x,y,z - axis and scale with maximum possible speeed
        msg_vel.linear.x =   0.05 * self.data.dict['rightStick']['y']
        msg_vel.angular.z = -0.25 * self.data.dict['rightStick']['z']
        if self.cmd_vel_publisher is not None:
            self.get_logger().debug('Publishing ' + self.topic_vel + ': "%s"' % msg_vel)
            self.cmd_vel_publisher.publish(msg_vel)

        # publish cmd_cam from left stick
        msg_cam = Quaternion()
        # data with respect to robot x,y,z - axis
        half_roll  = self.data.dict['leftStick']['x'] * self.scale[0] / 2
        half_pitch = self.data.dict['leftStick']['y'] * self.scale[1] / 2
        half_yaw   = self.data.dict['leftStick']['z'] * self.scale[2] / 2

        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cr = math.cos(half_roll)
        sr = math.sin(half_roll)

        msg_cam.w = cr * cp * cy + sr * sp * sy
        msg_cam.x = sr * cp * cy - cr * sp * sy
        msg_cam.y = cr * sp * cy + sr * cp * sy
        msg_cam.z = cr * cp * sy - sr * sp * cy
        if self.cmd_cam_publisher is not None:
            self.get_logger().debug('Publishing ' + self.topic_cam + ': "%s"' % msg_cam)
            self.cmd_cam_publisher.publish(msg_cam)
