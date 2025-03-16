#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # publisherの作成（ユーザ指定のトピックに合わせています）
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        # 1msごとにtimer_callbackを呼ぶタイマーを作成（周期：0.001秒）
        self.timer = self.create_timer(0.001, self.timer_callback)
        # ノード起動時刻を記録
        self.start_time = self.get_clock().now()
        
        self.span = 10

    def timer_callback(self):
        now = self.get_clock().now()
        # 経過時間（秒）
        elapsed = (now - self.start_time).nanoseconds / 1e9

        # 例としてサイン波で角度指令を計算
        angle_command = math.sin(2*math.pi * elapsed/self.span) * math.pi / 4

        # JointTrajectoryメッセージの作成
        traj_msg = JointTrajectory()
        # traj_msg.header.stamp = now.to_msg()
        # 使用するジョイント名（必要に応じて変更）
        traj_msg.joint_names = ['joint1', 'joint2']

        # TrajectoryPointの作成
        point = JointTrajectoryPoint()
        point.positions = [angle_command,math.pi/4 + angle_command]
        # 指令到達時間：ここでは10ms後を指定（0秒と10000000ナノ秒 = 0.01秒）
        point.time_from_start = Duration(sec=0, nanosec=1000000)

        traj_msg.points.append(point)

        # Gazeboへ指令を送信
        self.trajectory_publisher.publish(traj_msg)
        self.get_logger().info(f'Published angle command: {angle_command:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 