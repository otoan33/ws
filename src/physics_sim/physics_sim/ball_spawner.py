import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import time
from ament_index_python.packages import get_package_share_directory

class BallSpawner(Node):
    def __init__(self):
        super().__init__('ball_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Gazeboの起動を待つ
        time.sleep(2.0)
        
        # ボールのSDFを読み込む
        sdf_path = os.path.join(
            get_package_share_directory('physics_sim'),
            'models',
            'ball.sdf'
        )
        with open(sdf_path, 'r') as f:
            self.ball_sdf = f.read()
            
        self.spawn_ball()
        
    def spawn_ball(self):
        request = SpawnEntity.Request()
        request.name = 'bouncing_ball'
        request.xml = self.ball_sdf
        
        # 初期位置の設定
        initial_pose = Pose()
        initial_pose.position = Point(x=0.0, y=0.0, z=2.0)  # z=2.0で2メートルの高さ
        initial_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # デフォルトの向き
        request.initial_pose = initial_pose
        request.reference_frame = "world"  # 明示的にworld座標系を指定
        
        self.get_logger().info(f'Spawning ball at height: {initial_pose.position.z} meters')
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)
        
    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Ball spawned: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 