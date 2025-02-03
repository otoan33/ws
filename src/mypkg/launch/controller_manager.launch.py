from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
import os

def generate_launch_description():
    
    # Joint State Broadcasterの起動
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                  '--controller-manager', '/controller_manager'],
    )
    
    # Forward Position Controllerの起動
    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller',
                  '--controller-manager', '/controller_manager'],
    )
    
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ]) 