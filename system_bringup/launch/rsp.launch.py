from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    description_package = DeclareLaunchArgument(
        "description_package",
        description="ROS2 package that contains all the necessary URDFs/xacros"
    )

    description_file = DeclareLaunchArgument(
        "description_file",
        description="URDF/XACRO description file with the robot."
    )

    # get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(LaunchConfiguration("description_package")),
                 "urdf", LaunchConfiguration("description_file")]
            ),
            #" ",
            #"use_mock_hardware:="
            #"true"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # RSP node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Launch Description
    declared_arguments = [
        description_package,
        description_file
    ]

    nodes = [robot_state_publisher_node]

    return LaunchDescription(declared_arguments + nodes)
