import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import threading
# import pdb  # または import ipdb

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')
        
        # データ更新用のロックを追加
        self.data_lock = threading.Lock()
        
        # JointStateのサブスクライバー
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # データ保存用のdeque（最大1000点）
        self.time_points = deque(maxlen=1000)
        self.joint1_pos = deque(maxlen=1000)
        self.joint2_pos = deque(maxlen=1000)
        self.joint1_vel = deque(maxlen=1000)
        self.joint2_vel = deque(maxlen=1000)
        
        self.start_time = self.get_clock().now()
        
        # プロット設定
        plt.ion()  # インタラクティブモードを有効化
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # 位置のプロット
        self.pos_lines = []
        self.pos_lines.extend(self.ax1.plot([], [], 'r-', label='joint1'))
        self.pos_lines.extend(self.ax1.plot([], [], 'b-', label='joint2'))
        self.ax1.set_title('Joint Positions')
        self.ax1.set_xlabel('Time [s]')
        self.ax1.set_ylabel('Position [rad]')
        self.ax1.grid(True)
        self.ax1.legend()
        
        # 速度のプロット
        self.vel_lines = []
        self.vel_lines.extend(self.ax2.plot([], [], 'r-', label='joint1'))
        self.vel_lines.extend(self.ax2.plot([], [], 'b-', label='joint2'))
        self.ax2.set_title('Joint Velocities')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Velocity [rad/s]')
        self.ax2.grid(True)
        self.ax2.legend()
        
        plt.tight_layout()
        
        # アニメーション更新用のタイマー
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=100)
        
    def joint_state_callback(self, msg):
        with self.data_lock:
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            # メッセージの形式チェック
            if len(msg.position) < 2 or len(msg.velocity) < 2:
                self.get_logger().warn('受信したJointStateメッセージのデータが不足しています')
                return
            # データを追加
            self.time_points.append(current_time)
            self.joint1_pos.append(msg.position[0])
            self.joint2_pos.append(msg.position[1])
            self.joint1_vel.append(msg.velocity[0])
            self.joint2_vel.append(msg.velocity[1])
        
        # データ数を制限（例：直近の100点のみ表示）
        # max_points = 100
        # if len(self.time_points) > max_points:
        #     self.time_points = self.time_points[-max_points:]
        #     self.joint1_pos = self.joint1_pos[-max_points:]
        #     self.joint2_pos = self.joint2_pos[-max_points:]
        #     self.joint1_vel = self.joint1_vel[-max_points:]
        #     self.joint2_vel = self.joint2_vel[-max_points:]
    
    def update_plot(self, frame):
        with self.data_lock:
            # データが空の場合は更新しない
            if not self.time_points:
                return self.pos_lines + self.vel_lines
            
            # 軸の範囲を更新
            time_array = np.array(self.time_points)
            self.ax1.set_xlim(max(0, time_array[-1] - 10), time_array[-1] + 0.5)
            self.ax2.set_xlim(max(0, time_array[-1] - 10), time_array[-1] + 0.5)
            
            # 位置のプロット更新
            self.pos_lines[0].set_data(time_array, self.joint1_pos)
            self.pos_lines[1].set_data(time_array, self.joint2_pos)
            self.ax1.autoscale_view()
            self.ax1.relim()
            
            # 速度のプロット更新
            self.vel_lines[0].set_data(time_array, self.joint1_vel)
            self.vel_lines[1].set_data(time_array, self.joint2_vel)
            self.ax2.autoscale_view()
            self.ax2.relim()
            
            return self.pos_lines + self.vel_lines

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()
    
    # 別スレッドでROSのコールバックを実行
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # メインスレッドでプロットを表示
    plt.show(block=True)
    
    # クリーンアップ
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 