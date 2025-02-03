from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Gazeboの起動設定
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros').find('gazebo_ros'), 
            '/launch/gazebo.launch.py'
        ]),
    )
    
    # Gazeboにロボットモデルをスポーン
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'two_link_manipulator',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
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
        robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
    ]) 