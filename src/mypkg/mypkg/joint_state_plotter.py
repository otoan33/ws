import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
# import pdb  # または import ipdb

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')
        
        # データ更新用のロックを追加
        self.data_lock = threading.Lock()
        
        # JointStateのサブスクライバー
        self.js_sub = self.create_subscription(
            JointState,'joint_states',
            self.joint_state_callback, 10)
        
        # 指令値のサブスクライバー
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            self.joint_command_callback, 10)
        
        # データ保存用のdeque（最大1000点）
        self.time_points = deque(maxlen=1000)
        self.joint_pos = [deque(maxlen=1000) for _ in range(2)]
        self.joint_vel = [deque(maxlen=1000) for _ in range(2)]
        self.joint_ref = [deque(maxlen=1000) for _ in range(2)]
        
        self.start_time = self.get_clock().now()
        self.joint_ref_now = np.zeros(2)
        
        # プロット設定
        plt.ion()  # インタラクティブモードを有効化
        self.fig, self.axs = plt.subplots(2, 1, figsize=(10, 8))
        
        # 位置,速度のプロット
        self.pos_lines = []
        self.ref_lines = []
        self.vel_lines = []
        for ji in range(2):
            self.pos_lines.extend(self.axs[0].plot([], [], ['r-','b-'][ji], label='joint'+str(ji+1)))
            self.ref_lines.extend(self.axs[0].plot([], [], ['r--','b--'][ji], label='joint'+str(ji+1)+" ref"))
            self.vel_lines.extend(self.axs[1].plot([], [], ['r-','b-'][ji], label='joint'+str(ji+1)))
        
        self.axs[0].set_title('Joint Positions')
        self.axs[1].set_title('Joint Velocities')
        self.axs[0].set_ylabel('Position [deg]')
        self.axs[1].set_ylabel('Velocity [deg/s]')
        for axi in range(2):
            self.axs[axi].set_xlabel('Time [s]')
            self.axs[axi].grid(True)
            self.axs[axi].legend()
            # self.axs[axi].set_ylim(-45,45)
        
        plt.tight_layout()
        
        # アニメーション更新用のタイマー
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50)
        
    def joint_state_callback(self, msg):
        with self.data_lock:
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            # メッセージの形式チェック
            if len(msg.position) < 2 or len(msg.velocity) < 2:
                self.get_logger().warn('受信したJointStateメッセージのデータが不足しています')
                return
            
            # データを追加（ラジアンから度に変換）
            RAD_TO_DEG = 180.0 / np.pi
            self.time_points.append(current_time)
            for ji in range(2):
                self.joint_pos[ji].append(msg.position[ji] * RAD_TO_DEG)
                self.joint_vel[ji].append(msg.velocity[ji] * RAD_TO_DEG)
                self.joint_ref[ji].append(self.joint_ref_now[ji])

    def joint_command_callback(self, msg):
        with self.data_lock:
            if not msg.points or len(msg.points[0].positions) < 2:
                self.get_logger().warn('受信した指令値メッセージのデータが不足しています')
                return
            
            # 受信したメッセージの内容をログ出力
            self.get_logger().info(f'受信した指令値: joint1={msg.points[0].positions[0]:.3f}rad, ' 
                                 f'joint2={msg.points[0].positions[1]:.3f}rad')
            
            RAD_TO_DEG = 180.0 / np.pi
            for ji in range(2):
                self.joint_ref_now[ji] = msg.points[0].positions[ji] * RAD_TO_DEG
    
    def update_plot(self, frame):
        with self.data_lock:
            # データが空の場合は更新しない
            if not self.time_points:
                return self.pos_lines + self.vel_lines
            
            # 軸の範囲を更新
            time_array = np.array(self.time_points)
            for axi in range(2):
                self.axs[axi].set_xlim(max(0, time_array[-1] - 10), time_array[-1] + 0.5)
            
            # プロット更新
            for ji in range(2):
                self.pos_lines[ji].set_data(time_array, self.joint_pos[ji])
                self.ref_lines[ji].set_data(time_array, self.joint_ref[ji])
                self.vel_lines[ji].set_data(time_array, self.joint_vel[ji])
            for axi in range(2):
                self.axs[axi].autoscale_view()
                self.axs[axi].relim()
            
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