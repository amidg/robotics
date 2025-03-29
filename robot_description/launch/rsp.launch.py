from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    description_file = DeclareLaunchArgument(
        "description_file",
        description="URDF/XACRO description file with the robot."
    )

    prefix = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="xacro prefix"
    )

    use_gazebo_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="use gazebo simulation"
    )

    use_mock_hardware = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware mirroring inputs to outputs"
    )

    # done with Launch Description
    return LaunchDescription([
        description_file,
        prefix,
        use_gazebo_sim,
        use_mock_hardware,
        OpaqueFunction(
            function=launch_setup,
            args=[
                "prefix",
                "use_sim",
                "use_mock_hardware"
            ]
        )
    ])
