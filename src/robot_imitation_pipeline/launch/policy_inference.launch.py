from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_imitation_pipeline"), "config", "policy_inference.yaml"]
        ),
        description="Policy inference YAML config.",
    )
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value="data/imitation_runs/bc_state/best.pt",
        description="Path to a train_bc checkpoint.",
    )
    publish_arg = DeclareLaunchArgument(
        "publish_commands",
        default_value="false",
        description="Set true only when the robot is ready to accept policy commands.",
    )

    inference = Node(
        package="robot_imitation_pipeline",
        executable="policy_inference_node",
        name="policy_inference",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "checkpoint_path": LaunchConfiguration("checkpoint_path"),
                "publish_commands": ParameterValue(
                    LaunchConfiguration("publish_commands"),
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription([config_arg, checkpoint_arg, publish_arg, inference])
