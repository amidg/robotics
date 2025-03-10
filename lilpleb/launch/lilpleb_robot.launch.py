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
    launch_package = FindPackageShare('robot_description')
    robot_package = launch_package
    #robot_package = FindPackageShare('create_description')

    # rviz file
    default_rviz_config_path = PathJoinSubstitution([launch_package, 'rviz', 'lilpleb.rviz'])

    # robot urdf
    robot_description = ParameterValue(
        #Command(['xacro ', PathJoinSubstitution([robot_package, 'urdf', 'create_2.urdf.xacro'])]),
        Command(['xacro ', PathJoinSubstitution([robot_package, 'robots', 'lilpleb.urdf.xacro'])]),
        value_type=str
    )

    # add arguments
    # joint state publisher gui
    ld.add_action(
        DeclareLaunchArgument(
            name='jsp_gui',
            default_value='false',
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui'
        )
    )

    # rviz config
    ld.add_action(
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        )
    )

    # urdf
    ld.add_action(
        DeclareLaunchArgument(
            name='model',
            default_value=str(robot_description),
            description='Path to robot urdf file relative to urdf_tutorial package'
        )
    )

    # nodes
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
        )
    )

    ld.add_action(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('jsp_gui'))
        )
    )

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

    # rviz2 node
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        )
    )
    return ld
