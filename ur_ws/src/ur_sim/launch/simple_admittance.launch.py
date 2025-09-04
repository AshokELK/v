from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ur_sim = FindPackageShare('ur_sim').find('ur_sim')
    
    # Include the base robot spawn launch file
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ur_sim, 'launch', 'spawn_ur3e_gripper.launch.py')
        ]),
        launch_arguments={
            'robot_ip': '',
            'ur_type': 'ur3e',
            'safety_limits': 'true',
            'safety_pos_margin': '0.15',
            'safety_k_position': '20',
            'name': 'ur',
            'use_fake_hardware': 'true',
            'robot_controller': 'joint_trajectory_controller',
            'sim_gazebo': 'true',
            'simulation_controllers': os.path.join(pkg_ur_sim, 'config', 'ur3e_controllers_gripper.yaml'),
            'prefix': ''
        }.items()
    )
    
    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_ur_sim, 'rviz', 'ur3e.rviz')],
        output='screen'
    )
    
    # Launch the simple admittance controller
    admittance_controller = Node(
        package='ur_sim',
        executable='simple_admittance_controller.py',
        name='simple_admittance_controller',
        output='screen',
        parameters=[{
            'admittance_gain': 0.001,
            'max_velocity': 0.5,
            'force_dead_zone': 2.0,
            'control_rate': 50.0
        }]
    )
    
    return LaunchDescription([
        spawn_robot,
        rviz,
        admittance_controller
    ])
