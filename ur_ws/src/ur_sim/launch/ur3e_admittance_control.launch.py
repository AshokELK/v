from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_ur_sim = FindPackageShare('ur_sim').find('ur_sim')
    
    # Include the UR3e with gripper spawn launch file
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
            'description_package': 'ur_sim',
            'description_file': 'ur3e_robot_with_ft.urdf.xacro',
            'prefix': ''
        }.items()
    )
    
    # Launch RViz with custom configuration
    rviz_config = PathJoinSubstitution([pkg_ur_sim, 'rviz', 'ur3e_admittance.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Launch admittance controller
    admittance_controller = Node(
        package='ur_sim',
        executable='admittance_controller.py',
        name='admittance_controller',
        output='screen',
        parameters=[{
            'mass_matrix': [1.0, 1.0, 1.0, 0.5, 0.5, 0.5],
            'damping_matrix': [50.0, 50.0, 50.0, 20.0, 20.0, 20.0],
            'stiffness_matrix': [0.0, 0.0, 100.0, 10.0, 10.0, 10.0],
            'force_deadband': 2.0,
            'torque_deadband': 0.1
        }]
    )
    
    return LaunchDescription([
        spawn_robot,
        rviz,
        admittance_controller
    ])
