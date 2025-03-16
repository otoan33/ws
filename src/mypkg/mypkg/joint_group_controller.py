#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointGroupController(Node):
    def __init__(self):
        super().__init__('joint_group_controller')
        
        # publisherの作成
        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        # 1msごとにtimer_callbackを呼ぶタイマーを作成（周期：0.001秒）
        self.timer = self.create_timer(0.01, self.timer_callback)
        # ノード起動時刻を記録
        self.start_time = self.get_clock().now()
        
        self.span = 10

    def timer_callback(self):
        now = self.get_clock().now()
        # 経過時間（秒）
        elapsed = (now - self.start_time).nanoseconds / 1e9

        # 例としてサイン波で角度指令を計算
        angle_command = math.sin(2*math.pi * elapsed/self.span) * math.pi / 4

        # Float64MultiArrayメッセージの作成
        msg = Float64MultiArray()
        msg.data = [angle_command, math.pi/4 + angle_command]

        # コマンドを送信
        self.position_publisher.publish(msg)
        self.get_logger().info(f'Published angle command: {angle_command:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = JointGroupController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 