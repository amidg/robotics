from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    robot_package = DeclareLaunchArgument(
        "robot_package",
        description="Robot package with rviz and robot specific launch"
    )

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value="view_robot.rviz",
        description="Robot package with rviz and robot specific launch"
    )

    rsp = DeclareLaunchArgument(
        "rsp",
        default_value="true",
        description="Bring up Robot State Publisher"
    )

    jsp_gui = DeclareLaunchArgument(
        "jsp_gui",
        default_value="true",
        description="Joint State Publisher gui automatically"
    )

    # bring up the rsp.launch.py
    rsp_launch_path = PathJoinSubstitution(
        [FindPackageShare("system_bringup"), "launch", "rsp.launch.py"]
    )

    description_file = [LaunchConfiguration("robot_package"), ".urdf.xacro"]
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={
            "description_package": "robot_description",
            "description_file": description_file
        }.items(),
        condition=IfCondition(LaunchConfiguration("rsp"))
    )

    # joint state publisher gui
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("jsp_gui")),
    )
    
    # Rviz node
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("robot_package")),
            "rviz",
            LaunchConfiguration("rviz_config")
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Launch Description
    declared_arguments = [
        robot_package,
        rviz_config,
        rsp,
        jsp_gui
    ]

    nodes = [
        rsp_launch,
        joint_state_publisher_gui,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
