from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from utils_python.files import load_yaml

launch_args = [
    ("robot", "", "robot model to retrieve ros2 control config"),
    ("config", "controls.yaml", "yaml config for ros2 control"),
]

"""
Programmatically retrieve list of controllers
Returns list of strings
"""
def get_controllers(package_name, config):
    controllers = []
    # get controllers from yaml file
    controls_yaml = load_yaml(package_name, f"config/{config}")
    for key in controls_yaml["controller_manager"]["ros__parameters"]:
        if key != "update_rate":
            controllers.append(key)
    return controllers

"""
Generic RViz launcher
param: args -> command line arguments
param: dict -> TODO: add support for ros control args
"""
def launch_setup(context, *args, **kwargs):
    # get controllers
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration("robot")),
        "config",
        LaunchConfiguration("config"),
    ])

    # ros2 controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    # spawn controllers per each controller that is brought up
    # NOTE (Dmitrii): joint state broacaster will always be the first one
    names = get_controllers(
        LaunchConfiguration("robot").perform(context),
        LaunchConfiguration("config").perform(context),
    )
    controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                # TODO: implement controller arguments
                #"--param-file",
                #robot_controllers
                #"--controller-ros-args",
                #"-r /lilpleb_diff_controller/cmd_vel:=/cmd_vel",
            ]
        )
        for controller in names
    ]

    # delay joint state broadcaster after the last controller
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controllers[-1], # get last controller
            on_exit=[controllers[0]], # joint state broadcaster is always first
        )
    )

    # return nodes
    return [
        control_node,
        *controllers,
        delay_joint_state_broadcaster
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
