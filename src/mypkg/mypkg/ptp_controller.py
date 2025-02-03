#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class PTPController(Node):
    def __init__(self):
        super().__init__('ptp_controller')
        
        # トラジェクトリパブリッシャーの作成
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
        
        # タイマーの作成（2秒間隔で実行）
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # 往復動作のフラグ
        self.is_first_point = True
        
        # ジョイントの名前
        self.joint_names = ['joint1', 'joint2']
        
        # 2つの目標位置（ラジアン）
        self.point1 = [0.0, 0.0]  # ホームポジション
        self.point2 = [math.pi/4, math.pi/4]  # 45度の位置

    def timer_callback(self):
        # トラジェクトリメッセージの作成
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        # 現在の目標位置を選択
        target_position = self.point1 if self.is_first_point else self.point2
        
        # トラジェクトリポイントの作成
        point = JointTrajectoryPoint()
        point.positions = target_position
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        
        # 移動時間の設定（2秒）
        point.time_from_start = Duration(sec=2)
        
        trajectory_msg.points = [point]
        
        # メッセージのパブリッシュ
        self.trajectory_publisher.publish(trajectory_msg)
        
        # 次の目標位置のために状態を切り替え
        self.is_first_point = not self.is_first_point
        
        # ログの出力
        self.get_logger().info(f'Moving to {"point1" if self.is_first_point else "point2"}')

def main(args=None):
    rclpy.init(args=args)
    ptp_controller = PTPController()
    
    try:
        rclpy.spin(ptp_controller)
    except KeyboardInterrupt:
        pass
    
    ptp_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 