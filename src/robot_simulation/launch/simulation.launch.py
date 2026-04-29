# robot_complete.launch.py
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    LaunchConfiguration,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context: LaunchContext):
    
    # ============ 1. Package Paths ============
    pkg_simulation = FindPackageShare("robot_simulation")
    pkg_description = FindPackageShare("robot_description")
    pkg_commander = FindPackageShare("robot_commander")
    pkg_moveit = FindPackageShare("robot_moveit_config")
    
    # ============ 2. Get Launch Arguments (with defaults) ============
    
    # World file (default: empty.sdf)
    world_file_arg = LaunchConfiguration('world').perform(context)
    if os.path.isabs(world_file_arg):
        world_file = world_file_arg
    else:
        world_name = world_file_arg
        if world_name.startswith('worlds/'):
            world_name = world_name[len('worlds/'):]
        world_file = PathJoinSubstitution([
            pkg_simulation, "worlds", world_name
        ])
    
    # Component enable/disable flags (all default to True)
    enable_moveit = LaunchConfiguration('moveit').perform(context).lower() == 'true'
    enable_rviz_sim = LaunchConfiguration('rviz_sim').perform(context).lower() == 'true'
    enable_rviz_cmd = LaunchConfiguration('rviz_cmd').perform(context).lower() == 'true'
    enable_rqt = LaunchConfiguration('rqt').perform(context).lower() == 'true'
    enable_topic_bridge = LaunchConfiguration('topic_bridge').perform(context).lower() == 'true'
    enable_servo = LaunchConfiguration('servo').perform(context).lower() == 'true'
    
    # Controller enable flags
    enable_neck = LaunchConfiguration('neck').perform(context).lower() == 'true'
    enable_right_arm = LaunchConfiguration('right_arm').perform(context).lower() == 'true'
    enable_left_arm = LaunchConfiguration('left_arm').perform(context).lower() == 'true'
    enable_right_gripper = LaunchConfiguration('right_gripper').perform(context).lower() == 'true'
    enable_left_gripper = LaunchConfiguration('left_gripper').perform(context).lower() == 'true'
    
    # ============ 3. Paths ============
    xacro_path = PathJoinSubstitution([
        pkg_description, "urdf", "humanoid.urdf.xacro"
    ])
    srdf_path = PathJoinSubstitution([
        pkg_moveit, "config", "humanoid.srdf"
    ])
    
    rviz_sim_config = PathJoinSubstitution([
        pkg_simulation, "rviz", "simulation.rviz"
    ])
    
    rviz_cmd_config = PathJoinSubstitution([
        pkg_commander, "launch", "setting.rviz"
    ])
    
    # ============ 4. Environment Setup ============
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_description, ".."]),
            ':',
            PathJoinSubstitution([pkg_simulation, "models"])
        ]
    )
    
    # ============ 5. Core Simulation Nodes ============
    
    # Gazebo with world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 1 ", world_file]
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                'xacro ', xacro_path, " use_gazebo:=true"]),
            "publish_frequency": 1000.0,
            "use_sim_time": True,
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "humanoid",
            "-topic", "/robot_description",
            "-z", "0.9",
            "-x", "0",
            "-y", "0",
            "-Y", "1.5708",
        ],
        output="screen"
    )

    # Keep RViz/MoveIt's fixed world frame aligned with the Gazebo spawn pose.
    static_tf_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.9",
            "--yaw", "1.5708",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
        output="screen"
    )
    
    # ============ 6. Topic Bridges (optional) ============
    topic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/right_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/right_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/left_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/left_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/head_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/head_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/head_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/head_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ============ 7. Controller Spawners ============
    # Gazebo loads gz_ros2_control from the robot xacro and provides /controller_manager.
    # Do not start a second ros2_control_node here; it races the Gazebo controller manager.
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "40",
            "--service-call-timeout", "40",
        ],
        output="screen"
    )
    
    # Individual controllers (will be added conditionally)
    controller_nodes = []
    
    if enable_neck:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "neck_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "40",
                    "--service-call-timeout", "40",
                ],
                output="screen"
            )
        )
    
    if enable_right_arm:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "right_arm_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "40",
                    "--service-call-timeout", "40",
                ],
                output="screen"
            )
        )
    
    if enable_left_arm:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "left_arm_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "40",
                    "--service-call-timeout", "40",
                ],
                output="screen"
            )
        )
    
    if enable_right_gripper:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "right_gripper_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "40",
                    "--service-call-timeout", "40",
                ],
                output="screen"
            )
        )
    
    if enable_left_gripper:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "left_gripper_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "40",
                    "--service-call-timeout", "40",
                ],
                output="screen"
            )
        )
    
    # ============ 8. MoveIt ============
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_moveit, '/launch/move_group.launch.py'
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    moveit_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('servo_control'),
            '/launch/servo.launch.py'
        ])
    )
    
    # ============ 9. Commander Node ============
    commander_node = Node(
        package="robot_commander",  # From your ros2 run command
        executable="commander",      # The executable name
        name="robot_commander",      # Optional: give it a specific name
        output="screen",
        parameters=[{'use_sim_time': True}],  # Match simulation time
        # Add any command line arguments if needed
        # arguments=["--some-arg", "value"]
    )
    # ============ 10. Visualization ============
    rviz_sim = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_simulation",
        output="screen",
        arguments=["-d", rviz_sim_config],
        parameters=[{
            "robot_description": Command([
                'xacro ', xacro_path, " use_gazebo:=true"]),
            "robot_description_semantic": Command([
                'cat ', srdf_path]),
            "use_sim_time": True,
        }]
    )
    
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_commander",
        output="screen",
        arguments=["-d", rviz_cmd_config],
        parameters=[{
            "robot_description": Command([
                'xacro ', xacro_path, " use_gazebo:=true"]),
            "robot_description_semantic": Command([
                'cat ', srdf_path]),
            "use_sim_time": True,
        }]
    )
    
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        output="screen",
        arguments=[
            "--perspective-file",
            PathJoinSubstitution([pkg_simulation, "config", "rqt.perspective"])
        ],
        parameters=[{"use_sim_time": True}],
        cwd="/tmp"  # Fix for rqt crash
    )
    
    # ============ 11. Event Handlers ============
    
    # After robot spawns, load joint state broadcaster
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster]
        )
    )
    
    # After JSB loads, load all other controllers
    if controller_nodes:
        load_controllers = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=controller_nodes
            )
        )
    else:
        load_controllers = None
    
    # ============ 12. Build Launch Description ============
    
    # Core nodes always run
    core_nodes = [
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        static_tf_world_to_base,
        spawn_entity,
        load_joint_state_broadcaster,
    ]
    
    if load_controllers:
        core_nodes.append(load_controllers)
    
    if enable_topic_bridge:
        core_nodes.append(topic_bridge)
    
    # Optional nodes
    optional_nodes = []
    
    if enable_moveit:
        optional_nodes.append(moveit)

    if enable_servo:
        optional_nodes.append(moveit_servo)

        # Add commander node (using the same 'commander' argument)
    if LaunchConfiguration('commander').perform(context).lower() == 'true':
        optional_nodes.append(commander_node)
    
    # Commander node (always enabled by default, but can be disabled)
    # if LaunchConfiguration('commander').perform(context).lower() == 'true':
    #    optional_nodes.append(commander_node)
    
    if enable_rviz_sim:
        optional_nodes.append(rviz_sim)
    
    if enable_rviz_cmd:
        optional_nodes.append(rviz_cmd)
    
    if enable_rqt:
        optional_nodes.append(rqt)
    
    # Add log message
    log_startup = LogInfo(msg="=== Robot Complete System Starting ===")
    
    return [log_startup] + core_nodes + optional_nodes


def generate_launch_description():
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        
        # World file argument (default: empty.sdf)
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='World file name (in robot_simulation/worlds/) or absolute path'
        ),
        
        # Component enable/disable flags (all default to True)
        DeclareLaunchArgument(
            'moveit',
            default_value='true',
            description='Enable MoveIt'
        ),

        DeclareLaunchArgument(
            'servo',
            default_value='false',
            description='Enable MoveIt Servo'
        ),
        
        DeclareLaunchArgument(
            'commander',
            default_value='true',
            description='Enable commander node'
        ),
        
        DeclareLaunchArgument(
            'rviz_sim',
            default_value='true',
            description='Enable simulation RViz'
        ),
        
        DeclareLaunchArgument(
            'rviz_cmd',
            default_value='false',  # Default to false to avoid duplicate RViz
            description='Enable commander RViz'
        ),
        
        DeclareLaunchArgument(
            'rqt',
            default_value='false',  # Default to false to avoid crash
            description='Enable RQT'
        ),
        
        DeclareLaunchArgument(
            'topic_bridge',
            default_value='true',
            description='Enable topic bridge for cameras'
        ),
        
        # Controller enable flags (all default to True)
        DeclareLaunchArgument(
            'neck',
            default_value='true',
            description='Enable neck controller'
        ),
        
        DeclareLaunchArgument(
            'right_arm',
            default_value='true',
            description='Enable right arm controller'
        ),
        
        DeclareLaunchArgument(
            'left_arm',
            default_value='true',
            description='Enable left arm controller'
        ),
        
        DeclareLaunchArgument(
            'right_gripper',
            default_value='true',
            description='Enable right gripper controller'
        ),
        
        DeclareLaunchArgument(
            'left_gripper',
            default_value='true',
            description='Enable left gripper controller'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])
