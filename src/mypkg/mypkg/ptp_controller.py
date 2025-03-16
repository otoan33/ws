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
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        # ホームポジション（0度）と目標位置（45度）の往復
        self.joint_names = ['joint1', 'joint2']
        self.target_num = 0
        self.target_points = [[0.0,0.0],[math.pi/4, math.pi/4],[math.pi/4, math.pi/2],[0.0, math.pi/4]]

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.target_points[self.target_num]
        point.time_from_start = Duration(sec=2)
        msg.points = [point]
        
        self.trajectory_publisher.publish(msg)
        self.get_logger().info(f'Moving to P'+str(self.target_num))
        self.target_num = (self.target_num + 1) % len(self.target_points)

def main(args=None):
    rclpy.init(args=args)
    node = PTPController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 