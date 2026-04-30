from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("robot_simulation"),
                    "launch",
                    "simulation.launch.py",
                ])
            ]),
            launch_arguments={
                "world": "arm_planar_push.sdf",
                "initial_positions_file": "initial_positions_planar_push.yaml",
                "moveit": "true",
                "servo": "true",
                "commander": "false",
                "rviz_sim": "true",
                "rviz_cmd": "false",
                "rqt": "false",
            }.items(),
        )
    ])
