from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_simulation = FindPackageShare("robot_simulation")
    pkg_description = FindPackageShare("robot_description")

    world = LaunchConfiguration("world")

    set_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([pkg_description, ".."]),
            ":",
            PathJoinSubstitution([pkg_simulation, "models"]),
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 1 ", PathJoinSubstitution([pkg_simulation, "worlds", world])]
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/pusher/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        DeclareLaunchArgument(
            "world",
            default_value="planar_push.sdf",
            description="Planar pushing world file in robot_simulation/worlds/.",
        ),
        set_resource_path,
        gz_sim,
        bridge,
    ])
