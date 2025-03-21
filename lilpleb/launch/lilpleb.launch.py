from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare launch parameters
    set_teleop_type = DeclareLaunchArgument(
        "teleop_type",
        default_value="none",
        choices=["none", "keyboard", "joy", "vr"],
        description="Choose teleoperation type"
    )

    set_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Use mock hardware mirroring command to its states.",
    )

    # get lilpleb controls launch
    lilpleb_controls_launch_path = PathJoinSubstitution(
        [FindPackageShare("lilpleb"), "launch", "controls.launch.py"]
    )

    lilpleb_controls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lilpleb_controls_launch_path),
        launch_arguments={
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware")
        }.items()
    )

    # launch teleoperator
    teleoperator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("teleoperator"),
                    "launch",
                    f"keyboard.launch.py"
                ]
            )
        ),
        launch_arguments={
            "cmd_vel": "lilpleb_diff_controller/cmd_vel"
        }.items(),
        condition=IfCondition(NotEqualsSubstitution("teleop_type", "none"))
    )

    # add arguments and nodes
    declared_arguments = [
        set_teleop_type,
        set_mock_hardware
    ]

    nodes = [
        lilpleb_controls_launch,
        teleoperator_launch
    ]

    return LaunchDescription(declared_arguments + nodes)
