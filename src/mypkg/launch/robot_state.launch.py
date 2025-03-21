from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
import os

def generate_launch_description():
    pkg_share = FindPackageShare('mypkg').find('mypkg')
    
    # Robot Stateの設定
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', 
         os.path.join(pkg_share, 'urdf', 'two_link_manipulator.urdf.xacro')]
    )
    
    # Robot State Publisherの設定
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
    ]) 