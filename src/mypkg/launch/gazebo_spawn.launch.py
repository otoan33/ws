from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
import os

def generate_launch_description():
    pkg_share = FindPackageShare('mypkg').find('mypkg')
    
    # Gazeboの起動設定
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros').find('gazebo_ros'), 
            '/launch/gazebo.launch.py'
        ]),
    )
    
    # Gazeboにロボットモデルをスポーン
    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_entity',
    #     arguments=[
    #         '-entity', 'two_link_manipulator',
    #         '-topic', 'robot_description'
    #     ],
    #     output='screen'
    # )
    
    return LaunchDescription([
        gazebo,
        # spawn_entity,
    ]) 