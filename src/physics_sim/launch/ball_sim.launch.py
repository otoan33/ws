from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('physics_sim').find('physics_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # GZ Harmonicの起動設定
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': '-v4 -r ' + world_file
        }.items()
    )
    
    # ボールスポナーの起動（2秒待ってから）
    ball_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='physics_sim',
                executable='ball_spawner',
                name='ball_spawner',
                output='screen'
            )
        ]
    )
    
    # ボール状態モニターの起動（3秒待ってから）
    ball_monitor = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='physics_sim',
                executable='ball_state_monitor',
                name='ball_state_monitor',
                output='screen',
                parameters=[{'use_sim_time': True}]  # シミュレーション時間を使用
            )
        ]
    )
    
    return LaunchDescription([
        gz_sim,
        ball_spawner,
        ball_monitor,
    ]) 