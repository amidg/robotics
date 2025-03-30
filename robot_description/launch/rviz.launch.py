from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

launch_args = [
    ("robot", "", "Robot package name"),
    ("config", "view_robot.rviz", "RViz configuration file"),
    ("rsp", "true", "Bring up Robot State Publisher"),
    ("jsp_gui", "false", "Joint State Publisher gui automatically")
]

"""
Generic RViz launcher
param: args -> command line arguments
param: dict -> not used
"""
def launch_setup(context, *args, **kwargs):
    # joint state publisher gui
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("jsp_gui")),
    )

    # RViz bringup
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("robot")),
            "rviz",
            LaunchConfiguration("config")
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # return nodes
    return [
        joint_state_publisher_gui,
        rviz_node
    ]


def generate_launch_description():
    # Declare arguments
    launch_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description
        )
        for name, default_value, description in launch_args
    ]

    return LaunchDescription([
        *launch_arguments,
        OpaqueFunction(function=launch_setup)
    ])
