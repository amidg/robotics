from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
This is launcher that starts roomba's hardware:
    - robot state publisher
    - ros2 controller manager
"""
def launch_setup(context, *args, **kwargs):
    # robot state publisher
    rsp_launch_path = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "launch", "rsp.launch.py"]
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={
            "description_file": "lilpleb.urdf.xacro",
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware")
        }.items()
    )

    # controls
    controls_launch_path = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "launch", "controls.launch.py"]
    )

    controls_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controls_launch_path),
        launch_arguments={
            "robot": "lilpleb"
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_mock_hardware"))
    )

    return [
        rsp_launch,
        controls_launch,
    ]


def generate_launch_description():
    # declare launch parameters
    set_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware mirroring command to its states.",
    )

    return LaunchDescription([
        set_mock_hardware,
        OpaqueFunction(function=launch_setup)
    ])
