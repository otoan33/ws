import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from robot_state_publisher.robot_state_publisher import RobotStatePublisher
import os

class URDFPublisher(Node):
    def __init__(self):
        super().__init__('urdf_publisher')
        urdf_file = os.path.join(get_package_share_directory('mypkg'), 'urdf', 'sample.urdf')
        with open(urdf_file, 'r') as infp:
            robot_description = infp.read()
        self.publisher = self.create_publisher(String, 'robot_description', 10)
        self.publisher.publish(String(data=robot_description))
        self.get_logger().info('URDF Published')

def main(args=None):
    rclpy.init(args=args)
    node = URDFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()