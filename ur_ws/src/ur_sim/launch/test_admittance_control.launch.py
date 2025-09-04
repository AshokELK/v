from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="admittance_controller.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    controllers_file = LaunchConfiguration("controllers_file")

    # Load controllers
    load_controllers = []
    for controller in [
        "admittance_controller",
        "force_torque_sensor_broadcaster",
        "joint_state_broadcaster",
    ]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # Make sure controllers are loaded after robot is started
    delayed_controllers = []
    for controller in load_controllers:
        delayed_controllers.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_controllers[0],
                    on_exit=[controller],
                )
            )
        )

    return LaunchDescription(declared_arguments + delayed_controllers)
