#!/usr/bin/env python3
from pathlib import Path
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from example_interfaces.msg import Bool, Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import SetBool, Trigger

from robot_imitation_pipeline.io_utils import (
    ACTION_COMPONENTS,
    next_episode_dir,
    now_to_float,
    stamp_to_float,
    write_json,
)

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:
    cv2 = None
    CvBridge = None


class DemoRecorder(Node):
    def __init__(self) -> None:
        super().__init__("demo_recorder")
        self._declare_parameters()
        self._load_parameters()

        self.bridge = CvBridge() if CvBridge is not None else None
        self.recording = False
        self.episode_dir: Optional[Path] = None
        self.obs_dir: Optional[Path] = None
        self.episode_start_wall: Optional[float] = None
        self.episode_start_ros: Optional[float] = None
        self.episode_start_iso = ""

        self.latest_joint_state: Optional[JointState] = None
        self.latest_joint_state_time = float("nan")
        self.latest_joint_state_wall = 0.0
        self.latest_action = np.full(16, np.nan, dtype=np.float64)
        self.latest_action_valid = np.zeros(16, dtype=np.bool_)
        self.latest_gripper_command = np.full(2, np.nan, dtype=np.float64)
        self.latest_images: Dict[str, np.ndarray] = {}
        self.latest_image_ros_times: Dict[str, float] = {}
        self.latest_image_wall_times: Dict[str, float] = {}
        self.required_warned: Dict[str, bool] = {}

        self.samples: List[Dict[str, np.ndarray]] = []
        self.saved_image_paths: Dict[str, List[str]] = {self.camera_name: []}
        self.sample_skip_counts: Dict[str, int] = {}
        self.last_missing_inputs: List[str] = []
        self.topic_wall_times: Dict[str, float] = {}
        self.topic_message_counts: Dict[str, int] = {}
        self.last_rate_check_wall = time.monotonic()
        self.last_rate_check_counts: Dict[str, int] = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        default_qos = QoSProfile(depth=10)

        self.joint_sub = self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, sensor_qos
        )
        self.left_cmd_sub = self.create_subscription(
            Float64MultiArray, self.left_joint_command_topic, self._left_cmd_cb, default_qos
        )
        self.right_cmd_sub = self.create_subscription(
            Float64MultiArray, self.right_joint_command_topic, self._right_cmd_cb, default_qos
        )
        self.neck_cmd_sub = self.create_subscription(
            Float64MultiArray, self.neck_joint_command_topic, self._neck_cmd_cb, default_qos
        )
        self.left_gripper_sub = self.create_subscription(
            Bool, self.left_gripper_topic, self._left_gripper_cb, default_qos
        )
        self.right_gripper_sub = self.create_subscription(
            Bool, self.right_gripper_topic, self._right_gripper_cb, default_qos
        )
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self._image_cb,
            sensor_qos,
        ) if self.required_cameras else None

        self.start_srv = self.create_service(Trigger, "~/start", self._start_cb)
        self.stop_srv = self.create_service(SetBool, "~/stop", self._stop_cb)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._sample_once)
        self.warn_timer = self.create_timer(1.0, self._warn_if_stale)

        self.get_logger().info("Demo recorder ready.")
        self.get_logger().info(f"Save root: {self.save_root}")
        if self.required_cameras:
            self.get_logger().info(f"Primary camera: {self.camera_name} -> {self.camera_topic}")
        else:
            self.get_logger().info("Camera recording disabled. Recording joint data only.")

    def _declare_parameters(self) -> None:
        string_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)

        self.declare_parameter("output_dir", "")
        self.declare_parameter("save_root", "data/imitation_raw")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("sample_rate_hz", 10.0)
        self.declare_parameter("min_joint_state_hz", 5.0)
        self.declare_parameter("min_camera_hz", 5.0)
        self.declare_parameter("stale_topic_warn_sec", 1.0)
        self.declare_parameter("image_format", "jpg")
        self.declare_parameter("jpeg_quality", 92)
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("image_size", [224, 224])
        self.declare_parameter("control_mode", "joint_target")
        self.declare_parameter("action_mode", "joint_target")
        self.declare_parameter("task_name", "toy_brick_grasp")
        self.declare_parameter("robot_name", "humanoid")
        self.declare_parameter("require_joint_state_before_start", True)
        self.declare_parameter("joint_states_topic", "")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("left_joint_command_topic", "/left_joint_command")
        self.declare_parameter("right_arm_command_topic", "")
        self.declare_parameter("right_joint_command_topic", "/right_joint_command")
        self.declare_parameter("neck_joint_command_topic", "/neck_joint_command")
        self.declare_parameter("left_gripper_topic", "/open_left_gripper")
        self.declare_parameter("right_gripper_command_topic", "")
        self.declare_parameter("right_gripper_topic", "/open_right_gripper")
        self.declare_parameter(
            "left_joint_names",
            [
                "left_base_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_pitch_joint",
                "left_wrist_pitch_joint",
                "left_wrist_yaw_joint",
            ],
            string_array_desc,
        )
        self.declare_parameter(
            "right_arm_joint_names",
            [
                "right_base_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_pitch_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ],
            string_array_desc,
        )
        self.declare_parameter(
            "right_joint_names",
            [
                "right_base_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_pitch_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ],
            string_array_desc,
        )
        self.declare_parameter(
            "neck_joint_names",
            ["neck_pitch_joint", "neck_yaw_joint"],
            string_array_desc,
        )
        self.declare_parameter(
            "gripper_joint_names",
            ["left_gripper1_joint", "right_gripper1_joint"],
            string_array_desc,
        )
        self.declare_parameter(
            "required_cameras",
            [],
            string_array_desc,
        )
        self.declare_parameter(
            "optional_cameras",
            [],
            string_array_desc,
        )
        self.declare_parameter(
            "action_names",
            [
                "delta_right_base_pitch_joint",
                "delta_right_shoulder_roll_joint",
                "delta_right_shoulder_yaw_joint",
                "delta_right_elbow_pitch_joint",
                "delta_right_wrist_pitch_joint",
                "delta_right_wrist_yaw_joint",
                "right_gripper_command",
            ],
            string_array_desc,
        )
        self.declare_parameter("right_wrist_camera_image_topic", "")
        self.declare_parameter("right_wrist_camera_info_topic", "")
        # self.declare_parameter("head_camera_image_topic", "")
        # self.declare_parameter("head_camera_info_topic", "")
        self.declare_parameter("camera_name", "right_wrist_camera")
        self.declare_parameter("camera_topic", "/right_wrist_camera/image_raw")

    def _load_parameters(self) -> None:
        output_dir = str(self.get_parameter("output_dir").value)
        if output_dir:
            self.save_root = Path(output_dir)
        else:
            self.save_root = Path(self.get_parameter("save_root").value)
        if not self.save_root.is_absolute():
            self.save_root = Path.cwd() / self.save_root
        self.control_rate_hz = float(
            self.get_parameter("control_rate_hz").value or self.get_parameter("sample_rate_hz").value
        )
        self.min_joint_state_hz = float(self.get_parameter("min_joint_state_hz").value)
        self.min_camera_hz = float(self.get_parameter("min_camera_hz").value)
        self.stale_topic_warn_sec = float(self.get_parameter("stale_topic_warn_sec").value)
        self.image_format = str(self.get_parameter("image_format").value).lower()
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        image_width = int(self.get_parameter("image_width").value)
        image_height = int(self.get_parameter("image_height").value)
        if image_width > 0 and image_height > 0:
            self.image_size = [image_height, image_width]
        else:
            self.image_size = [int(v) for v in self.get_parameter("image_size").value]
        self.control_mode = str(
            self.get_parameter("control_mode").value or self.get_parameter("action_mode").value
        )
        self.task_name = str(self.get_parameter("task_name").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.require_joint_state_before_start = bool(
            self.get_parameter("require_joint_state_before_start").value
        )
        self.joint_state_topic = str(
            self.get_parameter("joint_states_topic").value or self.get_parameter("joint_state_topic").value
        )
        self.left_joint_command_topic = str(self.get_parameter("left_joint_command_topic").value)
        self.right_joint_command_topic = str(
            self.get_parameter("right_arm_command_topic").value
            or self.get_parameter("right_joint_command_topic").value
        )
        self.neck_joint_command_topic = str(self.get_parameter("neck_joint_command_topic").value)
        self.left_gripper_topic = str(self.get_parameter("left_gripper_topic").value)
        self.right_gripper_topic = str(
            self.get_parameter("right_gripper_command_topic").value
            or self.get_parameter("right_gripper_topic").value
        )
        self.left_joint_names = list(self.get_parameter("left_joint_names").value)
        self.right_joint_names = list(
            self.get_parameter("right_arm_joint_names").value or self.get_parameter("right_joint_names").value
        )
        self.neck_joint_names = list(self.get_parameter("neck_joint_names").value)
        self.gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)
        self.required_cameras = list(self.get_parameter("required_cameras").value)
        self.optional_cameras = list(self.get_parameter("optional_cameras").value)
        right_wrist_camera_image_topic = str(self.get_parameter("right_wrist_camera_image_topic").value)
        right_wrist_camera_info_topic = str(self.get_parameter("right_wrist_camera_info_topic").value)
        # head_camera_image_topic = str(self.get_parameter("head_camera_image_topic").value)
        # head_camera_info_topic = str(self.get_parameter("head_camera_info_topic").value)
        self.camera_topics = {
            "right_wrist_camera": {
                "image": right_wrist_camera_image_topic or str(self.get_parameter("camera_topic").value),
                "camera_info": right_wrist_camera_info_topic or "/right_wrist_camera/camera_info",
                "required": "right_wrist_camera" in self.required_cameras,
            },
            # "head_camera": {
            #     "image": head_camera_image_topic or "/camera/camera/color/image_raw",
            #     "camera_info": head_camera_info_topic or "/camera/camera/color/camera_info",
            #     "required": "head_camera" in self.required_cameras,
            # },
        }
        self.camera_name = "right_wrist_camera"
        self.camera_topic = self.camera_topics[self.camera_name]["image"]
        self.robot_state_names = (
            [f"{name}:position" for name in self.right_joint_names]
            + [f"{name}:velocity" for name in self.right_joint_names]
            + [f"{self.gripper_joint_names[1]}:position"]
        )
        action_names_param = list(self.get_parameter("action_names").value)
        self.action_names = action_names_param or (self.right_joint_names + ["right_gripper_command"])

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        self.latest_joint_state_time = stamp_to_float(msg.header.stamp)
        self.latest_joint_state_wall = time.monotonic()
        self.topic_wall_times[self.joint_state_topic] = self.latest_joint_state_wall
        self.topic_message_counts[self.joint_state_topic] = self.topic_message_counts.get(self.joint_state_topic, 0) + 1

    def _left_cmd_cb(self, msg: Float64MultiArray) -> None:
        self._set_action_slice("left_joint_target", msg.data, self.left_joint_command_topic)

    def _right_cmd_cb(self, msg: Float64MultiArray) -> None:
        self._set_action_slice("right_joint_target", msg.data, self.right_joint_command_topic)

    def _neck_cmd_cb(self, msg: Float64MultiArray) -> None:
        self._set_action_slice("neck_joint_target", msg.data, self.neck_joint_command_topic)

    def _left_gripper_cb(self, msg: Bool) -> None:
        self.latest_action[14] = 1.0 if msg.data else 0.0
        self.latest_action_valid[14] = True
        self.latest_gripper_command[0] = self.latest_action[14]
        self.topic_wall_times[self.left_gripper_topic] = time.monotonic()

    def _right_gripper_cb(self, msg: Bool) -> None:
        self.latest_action[15] = 1.0 if msg.data else 0.0
        self.latest_action_valid[15] = True
        self.latest_gripper_command[1] = self.latest_action[15]
        self.topic_wall_times[self.right_gripper_topic] = time.monotonic()

    def _set_action_slice(self, component: str, values: List[float], topic: str) -> None:
        start, end = ACTION_COMPONENTS[component]
        expected = end - start
        if len(values) != expected:
            self.get_logger().warn(
                f"Ignoring {topic}: expected {expected} values, received {len(values)}"
            )
            return
        self.latest_action[start:end] = np.asarray(values, dtype=np.float64)
        self.latest_action_valid[start:end] = True
        self.topic_wall_times[topic] = time.monotonic()

    def _image_cb(self, msg: Image) -> None:
        self.topic_wall_times[self.camera_topic] = time.monotonic()
        self.topic_message_counts[self.camera_topic] = self.topic_message_counts.get(self.camera_topic, 0) + 1
        if self.bridge is None or cv2 is None:
            self.get_logger().warn("cv_bridge and OpenCV are required to save camera JPG frames.")
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            target_size = (self.image_size[1], self.image_size[0])
            if image.shape[0:2] != tuple(self.image_size):
                image = cv2.resize(image, target_size, interpolation=cv2.INTER_AREA)
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode image from {self.camera_topic}: {exc}")
            return
        self.latest_images[self.camera_name] = image
        self.latest_image_ros_times[self.camera_name] = stamp_to_float(msg.header.stamp)
        self.latest_image_wall_times[self.camera_name] = now_to_float(self.get_clock())

    def _start_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self.recording:
            response.success = False
            response.message = f"Already recording {self.episode_dir}"
            return response
        if self.require_joint_state_before_start and self.latest_joint_state is None:
            response.success = False
            response.message = "No /joint_states received yet; refusing to start."
            return response
        self.episode_dir = next_episode_dir(self.save_root)
        self.obs_dir = self.episode_dir / "obs" / self.camera_name
        self.obs_dir.mkdir(parents=True, exist_ok=True)
        self.samples = []
        self.saved_image_paths = {self.camera_name: []}
        self.sample_skip_counts = {}
        self.last_missing_inputs = []
        self.required_warned = {}
        self.episode_start_wall = time.time()
        self.episode_start_ros = now_to_float(self.get_clock())
        self.episode_start_iso = time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(self.episode_start_wall))
        self._initialize_action_from_state()
        self.recording = True
        response.success = True
        response.message = str(self.episode_dir)
        self.get_logger().info(f"Started recording: {self.episode_dir}")
        return response

    def _stop_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not self.recording:
            response.success = False
            response.message = "Recorder is not running."
            return response
        episode_dir = self.episode_dir
        success = bool(request.data)
        self.recording = False
        self._flush_episode(success)
        response.success = True
        response.message = str(episode_dir)
        self.get_logger().info(f"Stopped recording: {episode_dir}, success={success}")
        return response

    def _initialize_action_from_state(self) -> None:
        if self.latest_joint_state is None:
            return
        name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        self._initialize_slice_from_names("left_joint_target", self.left_joint_names, name_to_pos)
        self._initialize_slice_from_names("right_joint_target", self.right_joint_names, name_to_pos)
        self._initialize_slice_from_names("neck_joint_target", self.neck_joint_names, name_to_pos)
        for idx, name in enumerate(self.gripper_joint_names[:2]):
            if name in name_to_pos:
                self.latest_gripper_command[idx] = float(name_to_pos[name])
                self.latest_action[14 + idx] = self.latest_gripper_command[idx]
                self.latest_action_valid[14 + idx] = True
        if not np.isfinite(self.latest_action[15]):
            self.latest_action[15] = 0.0
            self.latest_action_valid[15] = False

    def _initialize_slice_from_names(self, component: str, names: List[str], name_to_pos: Dict[str, float]) -> None:
        start, end = ACTION_COMPONENTS[component]
        if len(names) != (end - start):
            return
        values = []
        for name in names:
            if name not in name_to_pos:
                return
            values.append(name_to_pos[name])
        self.latest_action[start:end] = np.asarray(values, dtype=np.float64)
        self.latest_action_valid[start:end] = True

    def _sample_once(self) -> None:
        if not self.recording:
            return
        missing = self._missing_inputs()
        if missing:
            self.last_missing_inputs = missing
            for field in missing:
                self.sample_skip_counts[field] = self.sample_skip_counts.get(field, 0) + 1
                if not self.required_warned.get(field):
                    self.get_logger().warn(f"Recorder waiting for required input: {field}")
                    self.required_warned[field] = True
            return
        joint_pos, joint_vel = self._ordered_joint_arrays(self.latest_joint_state)
        robot_state = np.concatenate(
            [joint_pos[6:12], joint_vel[6:12], joint_pos[15:16]],
            axis=0,
        )
        action = np.concatenate([self.latest_action[6:12], self.latest_action[15:16]], axis=0)
        action_valid = np.concatenate([self.latest_action_valid[6:12], self.latest_action_valid[15:16]], axis=0)
        frame_index = len(self.samples)

        # Save image only if camera data is available
        image_rel_path = ""
        has_image = self.camera_name in self.latest_images
        if has_image:
            image_rel_path = self._save_step_image(frame_index)

        image_ts = self.latest_image_wall_times.get(self.camera_name, 0.0)
        image_ros_ts = self.latest_image_ros_times.get(self.camera_name, 0.0)

        sample = {
            "timestamp": np.asarray(now_to_float(self.get_clock()), dtype=np.float64),
            "joint_state_timestamp": np.asarray(self.latest_joint_state_time, dtype=np.float64),
            "image_timestamp": np.asarray(image_ts, dtype=np.float64),
            "image_ros_timestamp": np.asarray(image_ros_ts, dtype=np.float64),
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "action_legacy": self.latest_action.copy(),
            "action_valid_legacy": self.latest_action_valid.copy(),
            "gripper": self.latest_gripper_command.copy(),
            "robot_state": robot_state.astype(np.float64),
            "action": action.astype(np.float64),
            "action_valid": action_valid.astype(np.bool_),
            "image_rel_path": image_rel_path,
        }
        self.samples.append(sample)
        self.last_missing_inputs = []
        self.required_warned = {}

    def _missing_inputs(self) -> List[str]:
        missing: List[str] = []
        if self.latest_joint_state is None:
            missing.append("joint state missing")
        if not self.latest_action_valid[6:12].all():
            missing.append("action command missing")
        # Camera image is no longer required for joint-only recording
        # if self.camera_name not in self.latest_images:
        #     missing.append("camera image missing")
        if not self.latest_action_valid[15]:
            missing.append("gripper command/status missing")
        return missing

    def _ordered_joint_arrays(self, msg: JointState) -> Tuple[np.ndarray, np.ndarray]:
        names = self.left_joint_names + self.right_joint_names + self.neck_joint_names + self.gripper_joint_names[:2]
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity))
        pos = np.asarray([name_to_pos.get(name, np.nan) for name in names], dtype=np.float64)
        vel = np.asarray([name_to_vel.get(name, np.nan) for name in names], dtype=np.float64)
        return pos, vel

    def _save_step_image(self, frame_index: int) -> str:
        if self.obs_dir is None or cv2 is None:
            raise RuntimeError("Recorder image output directory is not ready")
        filename = f"{frame_index:06d}.{self.image_format}"
        image_path = self.obs_dir / filename
        image = self.latest_images[self.camera_name]
        if self.image_format in ("jpg", "jpeg"):
            ok = cv2.imwrite(str(image_path), image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        else:
            ok = cv2.imwrite(str(image_path), image)
        if not ok:
            raise RuntimeError(f"cv2.imwrite returned false for {image_path}")
        self.saved_image_paths[self.camera_name].append(str(image_path.relative_to(self.episode_dir)))
        return str(Path("obs") / self.camera_name / filename)

    def _flush_episode(self, success: bool) -> None:
        if self.episode_dir is None:
            return
        episode_dir = self.episode_dir
        n = len(self.samples)
        legacy_width = len(self.left_joint_names + self.right_joint_names + self.neck_joint_names + self.gripper_joint_names[:2])
        if n == 0:
            timestamps = np.zeros((0,), dtype=np.float64)
            joint_state_timestamps = np.zeros((0,), dtype=np.float64)
            image_timestamps = np.zeros((0,), dtype=np.float64)
            image_ros_timestamps = np.zeros((0,), dtype=np.float64)
            joint_pos = np.zeros((0, legacy_width), dtype=np.float64)
            joint_vel = np.zeros((0, legacy_width), dtype=np.float64)
            actions_legacy = np.zeros((0, legacy_width), dtype=np.float64)
            action_valid_legacy = np.zeros((0, legacy_width), dtype=np.bool_)
            gripper = np.zeros((0, 2), dtype=np.float64)
            robot_state = np.zeros((0, len(self.robot_state_names)), dtype=np.float64)
            action = np.zeros((0, len(self.action_names)), dtype=np.float64)
            action_valid = np.zeros((0, len(self.action_names)), dtype=np.bool_)
        else:
            timestamps = np.asarray([s["timestamp"] for s in self.samples], dtype=np.float64)
            joint_state_timestamps = np.asarray([s["joint_state_timestamp"] for s in self.samples], dtype=np.float64)
            image_timestamps = np.asarray([s["image_timestamp"] for s in self.samples], dtype=np.float64)
            image_ros_timestamps = np.asarray([s["image_ros_timestamp"] for s in self.samples], dtype=np.float64)
            joint_pos = np.stack([s["joint_pos"] for s in self.samples])
            joint_vel = np.stack([s["joint_vel"] for s in self.samples])
            actions_legacy = np.stack([s["action_legacy"] for s in self.samples])
            action_valid_legacy = np.stack([s["action_valid_legacy"] for s in self.samples])
            gripper = np.stack([s["gripper"] for s in self.samples])
            robot_state = np.stack([s["robot_state"] for s in self.samples])
            action = np.stack([s["action"] for s in self.samples])
            action_valid = np.stack([s["action_valid"] for s in self.samples])

        np.save(episode_dir / "timestamps.npy", timestamps)
        np.save(episode_dir / "joint_state_timestamps.npy", joint_state_timestamps)
        np.save(episode_dir / "robot_state.npy", robot_state)
        np.save(episode_dir / "action.npy", action)
        np.save(episode_dir / "joint_pos.npy", joint_pos)
        np.save(episode_dir / "joint_vel.npy", joint_vel)
        np.save(episode_dir / "actions.npy", actions_legacy)
        np.save(episode_dir / "action_valid.npy", action_valid_legacy)
        np.save(episode_dir / "gripper.npy", gripper)
        np.save(episode_dir / f"{self.camera_name}_timestamps.npy", image_timestamps)
        np.save(episode_dir / f"{self.camera_name}_ros_timestamps.npy", image_ros_timestamps)

        duration = 0.0
        if n >= 2:
            duration = float(timestamps[-1] - timestamps[0])
        end_time = now_to_float(self.get_clock())
        meta = {
            "schema_version": "0.2.0",
            "format": "robot_imitation_pipeline_raw_episode_v2",
            "created_wall_time": self.episode_start_wall,
            "start_time": self.episode_start_iso,
            "start_ros_time": self.episode_start_ros,
            "end_time": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
            "end_ros_time": end_time,
            "duration_sec": duration,
            "num_samples": n,
            "control_rate_hz": self.control_rate_hz,
            "sample_rate_hz": self.control_rate_hz,
            "control_mode": self.control_mode,
            "action_mode": self.control_mode,
            "task_name": self.task_name,
            "robot_name": self.robot_name,
            "camera_name": self.camera_name,
            "image_size": self.image_size,
            "robot_state_dim": len(self.robot_state_names),
            "action_dim": len(self.action_names),
            "robot_state_names": self.robot_state_names,
            "action_names": self.action_names,
            "joint_names": self.left_joint_names + self.right_joint_names + self.neck_joint_names + self.gripper_joint_names[:2],
            "right_arm_joint_names": self.right_joint_names,
            "right_gripper_name": self.gripper_joint_names[1],
            "topic_names": {
                "joint_states": self.joint_state_topic,
                "right_arm_command": self.right_joint_command_topic,
                "right_gripper_command": self.right_gripper_topic,
                "camera": self.camera_topic,
            },
            "command_topics": {
                "left_joint": self.left_joint_command_topic,
                "right_joint": self.right_joint_command_topic,
                "neck_joint": self.neck_joint_command_topic,
                "left_gripper": self.left_gripper_topic,
                "right_gripper": self.right_gripper_topic,
            },
            "camera_topics": {self.camera_name: self.camera_topic},
            "camera_frame_counts": {self.camera_name: n},
            "diagnostics": {
                "sample_skip_counts": self.sample_skip_counts,
                "last_missing_inputs": self.last_missing_inputs,
                "topic_message_counts": self.topic_message_counts,
            },
            "alignment_rule": "obs[i], robot_state[i] -> action[i]",
            "obs": {
                self.camera_name: {
                    "path": str(Path("obs") / self.camera_name),
                    "count": n,
                    "timestamps_file": f"{self.camera_name}_timestamps.npy",
                    "ros_timestamps_file": f"{self.camera_name}_ros_timestamps.npy",
                }
            },
        }
        write_json(episode_dir / "meta.json", meta)
        write_json(
            episode_dir / "success.json",
            {
                "success": success,
                "valid_for_training": bool(success and n > 0),
            },
        )

        self.episode_dir = None
        self.obs_dir = None

    def _warn_if_stale(self) -> None:
        now = time.monotonic()
        topics = [
            self.joint_state_topic,
            self.right_joint_command_topic,
            self.right_gripper_topic,
        ]
        # Only check camera topic if camera is required
        if self.required_cameras:
            topics.append(self.camera_topic)
        for topic in topics:
            last = self.topic_wall_times.get(topic)
            if last is None:
                self.get_logger().warn(f"No messages received yet on {topic}")
                continue
            age = now - last
            if age > self.stale_topic_warn_sec:
                self.get_logger().warn(f"Topic {topic} is stale: {age:.2f}s since last message")
        elapsed = now - self.last_rate_check_wall
        if elapsed <= 0.0:
            return
        min_rates = {
            self.joint_state_topic: self.min_joint_state_hz,
        }
        if self.required_cameras:
            min_rates[self.camera_topic] = self.min_camera_hz
        for topic, min_rate in min_rates.items():
            count = self.topic_message_counts.get(topic, 0)
            last_count = self.last_rate_check_counts.get(topic, 0)
            rate = float(count - last_count) / elapsed
            if count > 0 and rate < min_rate:
                self.get_logger().warn(
                    f"Topic {topic} rate is low: {rate:.2f} Hz < expected {min_rate:.2f} Hz"
                )
            self.last_rate_check_counts[topic] = count
        self.last_rate_check_wall = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recording:
            node.get_logger().warn("Interrupted while recording; saving as failure.")
            node.recording = False
            node._flush_episode(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
