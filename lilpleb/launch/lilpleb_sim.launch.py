from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import os
import sys
from utils_python.files import load_file, load_yaml
from ament_index_python.packages import get_package_share_directory

package = "lilpleb"
robot_description_package = "robot_description"
simulation_package = "simulation"
packages = {
    package: FindPackageShare(package),
    robot_description_package: FindPackageShare("robot_description"),
    simulation_package: FindPackageShare("simulation")
}


def generate_launch_description():
    ld = LaunchDescription()

    # robot package
    lilpleb_launch_py = PathJoinSubstitution(
        [packages[package], 'launch', 'lilpleb.launch.py'])
    ld.add_action(
        IncludeLaunchDescription(
            lilpleb_launch_py,
            launch_arguments={
                'use_sim_time': 'true',
                'ros2_control': 'true'
            }.items()
        )
    )

    # joystick controls
    #joystick = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package),'launch','joystick.launch.py'
    #            )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)
    #twist_mux_params = PathJoinSubstitution(
    #    [packages[robot_description_package], 'config', 'twist_mux.yaml'])
    #twist_mux_params = os.path.join(get_package_share_directory("robot_description"),'config','twist_mux.yaml')
    #ld.add_action(
    #    Node(
    #        package="twist_mux",
    #        executable="twist_mux",
    #        parameters=[twist_mux_params, {'use_sim_time': True}],
    #        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #    )
    #)

    # gazebo
    #gazebo_params_file = PathJoinSubstitution(
    #    [packages[simulation_package], 'config', 'gazebo_params.yaml'])
    #gazebo_launch = PathJoinSubstitution(["gazebo_ros", "launch", "gazebo.launch.py"])

    #gazebo_launch = PythonLaunchDescriptionSource(
    #    [os.path.join(get_package_share_directory('gazebo_ros'), 
    #     'launch', 'gazebo.launch.py')])
    ## Include the Gazebo launch file, provided by the gazebo_ros package
    #ld.add_action(
    #    IncludeLaunchDescription(
    #        gazebo_launch,
    #        launch_arguments={
    #            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
    #        }.items()
    #    )
    #)
    # Run the spawner node from the gazebo_ros package
    # The entity name doesn't really matter if you only have a single robot.
    #ld.add_action(
    #    Node(
    #        package='gazebo_ros',
    #        executable='spawn_entity.py',
    #        arguments=['-topic', 'robot_description',
    #                   '-entity', 'my_bot'],
    #        output='screen'
    #    )
    #)

    # controller manager
    robot_controllers = PathJoinSubstitution([
        packages[robot_description_package],
        "config",
        "lilpleb.controllers.yaml",
    ])
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diffbot_base_controller",
                "--param-file",
                robot_controllers,
                "--controller-ros-args",
                "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
            ],
        )
    )

    # controller manager
    # diff_drive_spawner
    #ld.add_action(
    #    Node(
    #        package="controller_manager",
    #        executable="spawner.py",
    #        arguments=["diff_cont"]
    #    )
    #)

    ## joint_broad_spawner
    #ld.add_action(
    #    Node(
    #        package="controller_manager",
    #        executable="spawner.py",
    #        arguments=["joint_broad"]
    #    )
    #)


    # rviz
    default_rviz_config_path = PathJoinSubstitution(
        [packages[package], 'rviz', 'lilpleb.rviz'])
    ld.add_action(
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        )
    )
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        )
    )
    return ld
