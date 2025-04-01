from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

launch_args = [
    ("teleop_type", "none", "Choose teleoperation type: none, keyboard, joy, vr"),
    ("rviz", "true", "Launch RViz? true/false"),
    #("use_mock_hardware", "false", "Use mock hardware mirroring command to its states.")
]

"""
This launcher starts lilpleb and various services
- Hardware
- Gazebo simulation
- Teleoperation stack
"""
def launch_setup(context, *args, **kwargs):
    # launch teleoperator
    teleoperator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("teleoperator"),
                    "launch",
                    f"{LaunchConfiguration('teleop_type').perform(context)}.launch.py"
                ]
            )
        ),
        launch_arguments={
            "cmd_vel": "lilpleb_diff_controller/cmd_vel"
        }.items(),
        condition=IfCondition(NotEqualsSubstitution("teleop_type", "none"))
    )

    # rviz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "launch",
                    "rviz.launch.py" 
                ]
            )
        ),
        launch_arguments={
            "robot": "lilpleb",
            #"config": "lilpleb.rviz",
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return [
        teleoperator_launch,
        rviz_launch
    ]


def generate_launch_description():
    # declare arguments
    launch_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description
        )
        for name, default_value, description in launch_args
    ]

    # done with Launch Description
    return LaunchDescription([
        # must unpack the list of launch args
        *launch_arguments,
        OpaqueFunction(function=launch_setup)
    ])
