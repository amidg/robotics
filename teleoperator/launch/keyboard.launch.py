from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare launch arguments
    cmd_vel = LaunchConfiguration("cmd_vel")
    remap_cmd_vel = DeclareLaunchArgument(
        "cmd_vel",
        default_value="cmd_vel",
        description="Declare remapping for the `cmd_vel` topic"
    )

    # search for the params file
    teleop_parameters = PathJoinSubstitution(
        [FindPackageShare("teleoperator"), "config", "teleop_params.yaml"]
    )

    # start teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix = 'xterm -e',
        parameters=[teleop_parameters],
        remappings=[
            ('cmd_vel', cmd_vel)
        ]
    )

    # add arguments and nodes
    declared_arguments = [
        remap_cmd_vel
    ]

    nodes = [
        teleop_node
    ]

    return LaunchDescription(declared_arguments + nodes)
