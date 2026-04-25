from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_available(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _build_launch(context, *args, **kwargs):
    del args
    del kwargs

    actions = []

    use_wrist_camera = _as_bool(LaunchConfiguration("use_wrist_camera").perform(context))
    use_head_camera = _as_bool(LaunchConfiguration("use_head_camera").perform(context))
    wrist_namespace = LaunchConfiguration("wrist_camera_namespace").perform(context).strip("/")
    head_camera_name = LaunchConfiguration("head_camera_namespace").perform(context).strip("/") or "head_camera"
    wrist_video_device = LaunchConfiguration("wrist_video_device").perform(context)
    wrist_width = int(LaunchConfiguration("wrist_image_width").perform(context))
    wrist_height = int(LaunchConfiguration("wrist_image_height").perform(context))
    wrist_fps = float(LaunchConfiguration("wrist_fps").perform(context))
    wrist_driver = LaunchConfiguration("wrist_driver").perform(context).strip().lower()

    if use_wrist_camera:
        selected_driver = wrist_driver
        if selected_driver == "auto":
            if _package_available("v4l2_camera"):
                selected_driver = "v4l2_camera"
            elif _package_available("usb_cam"):
                selected_driver = "usb_cam"
            else:
                selected_driver = "opencv"

        if selected_driver == "v4l2_camera" and not _package_available("v4l2_camera"):
            actions.append(LogInfo(msg="[WARN] v4l2_camera package is missing; falling back to usb_cam/OpenCV."))
            selected_driver = "usb_cam" if _package_available("usb_cam") else "opencv"

        if selected_driver == "usb_cam" and not _package_available("usb_cam"):
            actions.append(LogInfo(msg="[WARN] usb_cam package is missing; falling back to OpenCV publisher."))
            selected_driver = "opencv"

        if selected_driver == "v4l2_camera":
            wrist_node = Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                namespace=wrist_namespace,
                name="camera",
                output="screen",
                parameters=[
                    {
                        "video_device": wrist_video_device,
                        "image_size": [wrist_width, wrist_height],
                        "time_per_frame": [1, max(int(round(wrist_fps)), 1)],
                    }
                ],
                remappings=[
                    ("image_raw", "image_raw"),
                    ("camera_info", "camera_info"),
                ],
            )
            actions.append(LogInfo(msg=f"[INFO] Starting wrist camera with v4l2_camera on {wrist_video_device}."))
        elif selected_driver == "usb_cam":
            wrist_node = Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                namespace=wrist_namespace,
                name="camera",
                output="screen",
                parameters=[
                    {
                        "video_device": wrist_video_device,
                        "image_width": wrist_width,
                        "image_height": wrist_height,
                        "framerate": wrist_fps,
                        "camera_name": wrist_namespace or "right_wrist_camera",
                        "frame_id": "right_wrist_camera_optical_frame",
                        "pixel_format": "mjpeg2rgb",
                    }
                ],
                remappings=[
                    ("image_raw", "image_raw"),
                    ("camera_info", "camera_info"),
                ],
            )
            actions.append(LogInfo(msg=f"[INFO] Starting wrist camera with usb_cam on {wrist_video_device}."))
        else:
            wrist_node = Node(
                package="robot_imitation_pipeline",
                executable="usb_camera_publisher",
                namespace=wrist_namespace,
                name="camera",
                output="screen",
                parameters=[
                    {
                        "video_device": wrist_video_device,
                        "image_width": wrist_width,
                        "image_height": wrist_height,
                        "fps": wrist_fps,
                        "camera_name": wrist_namespace or "right_wrist_camera",
                    }
                ],
            )
            actions.append(
                LogInfo(
                    msg="[WARN] No ROS USB camera driver detected. Falling back to robot_imitation_pipeline OpenCV publisher."
                )
            )

        actions.append(wrist_node)
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wrist_node,
                    on_exit=[
                        LogInfo(
                            msg="[ERROR] Required wrist camera node exited. "
                            "Recording needs /right_wrist_camera/image_raw to be available."
                        )
                    ],
                )
            )
        )

    if use_head_camera:
        if _package_available("realsense2_camera"):
            head_node = Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name=head_camera_name,
                output="screen",
                parameters=[
                    {
                        "enable_color": True,
                        "enable_depth": False,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "pointcloud.enable": False,
                    }
                ],
            )
            actions.append(
                LogInfo(
                    msg="[INFO] Starting optional RealSense head camera. "
                    "If the device is absent, continue recording with the wrist camera only."
                )
            )
            actions.append(head_node)
            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=head_node,
                        on_exit=[
                            LogInfo(
                                msg="[WARN] Optional RealSense head camera exited. "
                                "Continuing with wrist camera only."
                            )
                        ],
                    )
                )
            )
        else:
            actions.append(
                LogInfo(
                    msg="[WARN] realsense2_camera package is not installed. "
                    "Skipping optional head camera and continuing with wrist camera only."
                )
            )

    if not use_wrist_camera:
        actions.append(
            LogInfo(
                msg="[WARN] use_wrist_camera:=false disables the required wrist camera. "
                "The recorder will not receive /right_wrist_camera/image_raw."
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_head_camera", default_value="true"),
            DeclareLaunchArgument("use_wrist_camera", default_value="true"),
            DeclareLaunchArgument("wrist_video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("wrist_image_width", default_value="640"),
            DeclareLaunchArgument("wrist_image_height", default_value="480"),
            DeclareLaunchArgument("wrist_fps", default_value="30"),
            DeclareLaunchArgument("wrist_camera_namespace", default_value="right_wrist_camera"),
            DeclareLaunchArgument("head_camera_namespace", default_value="head_camera"),
            DeclareLaunchArgument("wrist_driver", default_value="auto"),
            OpaqueFunction(function=_build_launch),
        ]
    )
