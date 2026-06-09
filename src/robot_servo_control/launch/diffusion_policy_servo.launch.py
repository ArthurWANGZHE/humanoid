from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    checkpoint_path_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value="data/checkpoints/diffusion_policy/latest.pt",
        description="Path to the diffusion policy checkpoint.",
    )
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="cuda",
        description="Inference device: cuda or cpu.",
    )
    control_rate_arg = DeclareLaunchArgument(
        "control_rate",
        default_value="10.0",
        description="Policy control loop rate in Hz.",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        description="When true, print actions but do not publish JointJog commands.",
    )
    action_scale_arg = DeclareLaunchArgument(
        "action_scale",
        default_value="1.0",
        description="Safety scale applied to policy actions.",
    )
    obs_horizon_arg = DeclareLaunchArgument(
        "obs_horizon",
        default_value="0",
        description="Observation history length. 0 means use checkpoint config.",
    )
    action_horizon_arg = DeclareLaunchArgument(
        "action_horizon",
        default_value="1",
        description="Number of predicted actions to execute before replanning.",
    )
    max_joint_delta_arg = DeclareLaunchArgument(
        "max_joint_delta",
        default_value="0.05",
        description="Maximum per-cycle joint target delta in radians.",
    )
    max_velocity_arg = DeclareLaunchArgument(
        "max_velocity",
        default_value="0.5",
        description="Maximum JointJog velocity in rad/s.",
    )
    use_normalizer_arg = DeclareLaunchArgument(
        "use_normalizer",
        default_value="true",
        description="Use checkpoint normalization statistics when present.",
    )

    node = Node(
        package="robot_servo_control",
        executable="diffusion_policy_servo_node",
        name="diffusion_policy_servo_node",
        output="screen",
        parameters=[
            {
                "checkpoint_path": LaunchConfiguration("checkpoint_path"),
                "device": LaunchConfiguration("device"),
                "control_rate": ParameterValue(
                    LaunchConfiguration("control_rate"),
                    value_type=float,
                ),
                "dry_run": ParameterValue(
                    LaunchConfiguration("dry_run"),
                    value_type=bool,
                ),
                "action_scale": ParameterValue(
                    LaunchConfiguration("action_scale"),
                    value_type=float,
                ),
                "obs_horizon": ParameterValue(
                    LaunchConfiguration("obs_horizon"),
                    value_type=int,
                ),
                "action_horizon": ParameterValue(
                    LaunchConfiguration("action_horizon"),
                    value_type=int,
                ),
                "max_joint_delta": ParameterValue(
                    LaunchConfiguration("max_joint_delta"),
                    value_type=float,
                ),
                "max_velocity": ParameterValue(
                    LaunchConfiguration("max_velocity"),
                    value_type=float,
                ),
                "use_normalizer": ParameterValue(
                    LaunchConfiguration("use_normalizer"),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            checkpoint_path_arg,
            device_arg,
            control_rate_arg,
            dry_run_arg,
            action_scale_arg,
            obs_horizon_arg,
            action_horizon_arg,
            max_joint_delta_arg,
            max_velocity_arg,
            use_normalizer_arg,
            node,
        ]
    )
