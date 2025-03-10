from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from utils_python.files import load_file, load_yaml

def generate_launch_description():
    ld = LaunchDescription()

    # launch packages
    launch_package = FindPackageShare('lilpleb')
    robot_package = FindPackageShare('robot_description')

    # robot urdf
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([robot_package, 'robots', 'lilpleb.urdf.xacro'])]),
        value_type=str
    )

    # add arguments
    ld.add_action(
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use sim time if true'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name='ros2_control',
            default_value='true',
            choices=['true', 'false'],
            description='Use ROS2 Control if true'
        )
    )

    # rviz config
    #ld.add_action(
    #    DeclareLaunchArgument(
    #        name='rviz_config',
    #        default_value=default_rviz_config_path,
    #        description='Absolute path to rviz config file'
    #    )
    #)

    # robot state publisher node
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
            }]
        )
    )
    return ld
