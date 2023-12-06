#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'poses3d', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        x_list = [2.0, 2.0, 4.0, 4.0, 6.0]
        y_list = [0.0, 2.0, 2.0, -2.0, 0.0]
        # 
        for i in range(5):
            x, y, z, qx, qy, qz, qw = x_list[i], y_list[i], 0.0, 0.0, 0.0, 0.0, 1.0 # set Pose values
            pose = Pose() # create a new Pose message
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            msg.poses.append(pose) # add the new Pose object to the PoseArray list
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing path : ')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


    # minimal_publisher = MinimalPublisher()

    # rclpy.spin(minimal_publisher)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()