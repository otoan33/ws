import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TestNode(Node):
    def __init__(self):
        super().__init__("TestNode")
        print("Hello")
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_transform()

    def broadcast_static_transform(self):
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'world'
        static_transform_stamped.child_frame_id = 'test_frame'

        static_transform_stamped.transform.translation.x = 1.0
        static_transform_stamped.transform.translation.y = 2.0
        static_transform_stamped.transform.translation.z = 3.0

        self.static_broadcaster.sendTransform(static_transform_stamped)

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
