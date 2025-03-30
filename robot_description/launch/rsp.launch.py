from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

"""
RSP launch arguments
"""
launch_args = [
    ("description_file", "", "URDF/XACRO description file with the robot."),
    ("prefix", "", "xacro prefix"),
    ("use_sim", "false", "use gazebo simulation"),
    ("use_mock_hardware", "false", "Use mock hardware mirroring inputs to outputs")
]

"""
This is generic rsp launcher that takes arguments required
for the robot state publisher
param: args -> command line arguments
param: dict -> not used
"""
def launch_setup(context, *args, **kwargs):
    # build xacro command
    xacro_command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("robot_description"),
             "urdf", LaunchConfiguration("description_file")]
        ),
    ]

    for arg in args:
        xacro_command.append(" ")
        xacro_command.append(f"{arg}:=")
        xacro_command.append(LaunchConfiguration(arg).perform(context))

    # run xacro command
    robot_description_content = Command(xacro_command)
    robot_description = {"robot_description": robot_description_content}

    # run robot state publisher node
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return [rsp]


"""
generate_launch_description() will be called by other launch files
as well as ros2 itself
"""
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

    # done with Launch Description
    return LaunchDescription([
        # must unpack the list of launch args
        *launch_arguments,
        OpaqueFunction(
            function=launch_setup,
            args=[arg[0] for arg in launch_args[1:]]
        )
    ])
