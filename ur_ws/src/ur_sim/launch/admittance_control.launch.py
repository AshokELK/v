from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_ur_sim = get_package_share_directory('ur_sim')
    
    # Process the URDF file
    urdf_file = os.path.join(pkg_ur_sim, 'urdf', 'ur3e_robot_with_ft.urdf.xacro')
    robot_description_content = xacro.process_file(urdf_file).toxml()
    
    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'pause': 'false'}.items()
    )
    
    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'ur3e',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1',
                  '-topic', '/robot_description']
    )

    # Load controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            os.path.join(pkg_ur_sim, "config", "ur3e_controllers.yaml"),
        ],
        output="screen",
    )

    # Load and start controllers
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # Start RViz
    rviz_config_file = os.path.join(pkg_ur_sim, 'rviz', 'ur3e_admittance.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Launch admittance controller
    admittance_controller = Node(
        package='ur_sim',
        executable='admittance_controller.py',
        name='admittance_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        rviz,
        admittance_controller
    ])
