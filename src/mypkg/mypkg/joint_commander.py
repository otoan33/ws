import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            10)
        
        # タイマーで定期的に関節角度を更新
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0
        
    def timer_callback(self):
        msg = Float64MultiArray()
        # サイン波で動かす例（±π/2 radianの範囲で動く）
        msg.data = [
            math.pi/2 * math.sin(self.t),  # ±1.57 radian (±90度)
            math.pi/2 * math.cos(self.t)   # ±1.57 radian (±90度)
        ]
        self.publisher.publish(msg)
        self.t += 0.1
        
def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 