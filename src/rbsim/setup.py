from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rbsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'controller_manager',
        'robot_state_publisher',
        'joint_state_broadcaster',
        'joint_trajectory_controller',
        'gazebo_ros',
        'xacro',
    ],
    zip_safe=True,
    maintainer='nfukuda',
    maintainer_email='otoan33@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_commander = rbsim.joint_commander:main',
            'cmd_relay = rbsim.cmd_relay:main',
        ],
    },
)
