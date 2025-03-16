#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # ポジションコマンドのパブリッシャー
        self.position_publisher = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )
        
        # 1msごとにタイマーコールバックを呼び出し
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        # 周期運動の周期（秒）
        self.period = 10.0
        
    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # サイン波で関節角度を計算
        angle1 = math.sin(2*math.pi * elapsed/self.period) * math.pi / 4  # ±45度
        angle2 = math.cos(2*math.pi * elapsed/self.period) * math.pi / 4  # ±45度
        
        # Float64MultiArrayメッセージの作成
        msg = Float64MultiArray()
        msg.data = [angle1, angle2]
        
        # メッセージのパブリッシュ
        self.position_publisher.publish(msg)
        self.get_logger().info(f'Published angles: [{angle1:.2f}, {angle2:.2f}]')

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 