from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

lilpleb_package = "lilpleb"
robot_description_package = "robot_description"

# args
rqt_joint_gui_arg = "rqt_joint_gui"
mock_hardware_arg = "use_mock_hardware"

def generate_launch_description():
    # create LaunchConfigurations
    gui = LaunchConfiguration(rqt_joint_gui_arg)
    use_mock_hardware = LaunchConfiguration(mock_hardware_arg)

    # Declare arguments
    enable_rqt_joint_pub_gui = DeclareLaunchArgument(
        rqt_joint_gui_arg,
        default_value="false",
        description="Start RViz2 automatically with this launch file.",
    )

    enable_mock_hardware = DeclareLaunchArgument(
        mock_hardware_arg,
        default_value="true",
        description="Use mock hardware mirroring command to its states.",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package),
                 "urdf", "lilpleb.urdf.xacro"]
            ),
            #" ",
            #"use_mock_hardware:="
            #"true"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ros2 control
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(lilpleb_package),
            "config",
            "controls.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(lilpleb_package), "rviz", "lilpleb.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lilpleb_diff_controller",
            #"--param-file",
            #robot_controllers
            #"--controller-ros-args",
            #"-r /lilpleb_diff_controller/cmd_vel:=/cmd_vel",
        ]
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )


    # add launch arguments to the array
    declared_arguments = [
        enable_rqt_joint_pub_gui,
        enable_mock_hardware
    ]

    # add nodes to the array
    nodes = [
        control_node,
        robot_state_pub_node,
        diff_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
