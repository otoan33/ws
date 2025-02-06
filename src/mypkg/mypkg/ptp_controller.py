#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class PTPController(Node):
    def __init__(self):
        super().__init__('ptp_controller')
        
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
        
        # 3秒間隔で実行（動作を確認しやすくするため）
        self.timer = self.create_timer(3.0, self.timer_callback)
        
        # ホームポジション（0度）と目標位置（45度）の往復
        self.is_home = True
        self.joint_names = ['joint1', 'joint2']
        self.home_position = [0.0, 0.0]
        self.target_position = [math.pi/4, math.pi/4]

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.home_position if self.is_home else self.target_position
        point.time_from_start = Duration(sec=2)
        msg.points = [point]
        
        self.trajectory_publisher.publish(msg)
        self.get_logger().info(f'Moving to {"home" if self.is_home else "target"}')
        self.is_home = not self.is_home

def main(args=None):
    rclpy.init(args=args)
    node = PTPController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 