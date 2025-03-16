import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class BallStateMonitor(Node):
    def __init__(self):
        super().__init__('ball_state_monitor')
        
        # QoSプロファイルをBEST_EFFORTとVOLATILEに設定
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # クロックの設定
        self.get_clock().now()  # シミュレーション時間の初期化
        
        self.get_logger().info('Starting ball state monitor...')
        
        # トピックの存在確認用のタイマー
        self.create_timer(1.0, self.check_topics)
        
        # Gazeboのモデル状態をサブスクライブ
        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_callback,
            qos
        )
        
        self.message_received = False
        self.get_logger().info('Subscribed to /model_states')
    
    def check_topics(self):
        if not self.message_received:
            self.get_logger().warn('No messages received from /model_states')
            # 利用可能なトピックを表示
            topics = self.get_topic_names_and_types()
            self.get_logger().info('Available topics:')
            for topic_name, topic_types in topics:
                self.get_logger().info(f'  {topic_name}: {topic_types}')
    
    def model_callback(self, msg):
        self.message_received = True
        try:
            # ボールのインデックスを探す
            if 'bouncing_ball' not in msg.name:
                self.get_logger().debug('Waiting for ball to spawn...')
                return
                
            ball_index = msg.name.index('bouncing_ball')
            
            # ボールの位置と速度を取得
            position = msg.pose[ball_index].position
            velocity = msg.twist[ball_index].linear
            
            # 情報を表示
            self.get_logger().info(
                f'Ball state - Position: (x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f})m, '
                f'Velocity: (x={velocity.x:.3f}, y={velocity.y:.3f}, z={velocity.z:.3f})m/s'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = BallStateMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()  # shutdownをtry_shutdownに変更
        except Exception as e:
            pass

if __name__ == '__main__':
    main() 