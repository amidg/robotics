from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from utils_python.files import load_file, load_yaml

package_name = "simulation_gazebo"

def generate_launch_description():
    ld = LaunchDescription()

    # gazebo
    gazebo_params_file = PathJoinSubstitution(
        [packages[simulation_package], 'config', 'gazebo_params.yaml'])
    # Include the Gazebo launch file, provided by the gazebo_ros package
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(["gazebo_ros", "launch", "gazebo.launch.py"]),
            launch_arguments={
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
            }.items()
        )
    )
    # Run the spawner node from the gazebo_ros package
    # The entity name doesn't really matter if you only have a single robot.
    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'my_bot'],
            output='screen'
        )
    )
    return ld
