#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from collections import OrderedDict, deque
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from control_msgs.msg import JointJog
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

try:
    import torch
    from torch import nn
except Exception as exc:  # pragma: no cover - depends on deployment env
    torch = None
    nn = None
    TORCH_IMPORT_ERROR = exc
else:
    TORCH_IMPORT_ERROR = None


if nn is not None:

    class SinusoidalTimestepEmbedding(nn.Module):
        def __init__(self, dim: int):
            super().__init__()
            self.dim = dim

        def forward(self, timesteps: Any) -> Any:
            half_dim = self.dim // 2
            scale = math.log(10000) / max(half_dim - 1, 1)
            freqs = torch.exp(
                torch.arange(
                    half_dim,
                    device=timesteps.device,
                    dtype=torch.float32,
                )
                * -scale
            )
            args = timesteps.float().unsqueeze(1) * freqs.unsqueeze(0)
            emb = torch.cat([torch.sin(args), torch.cos(args)], dim=1)
            if self.dim % 2 == 1:
                emb = torch.cat([emb, torch.zeros_like(emb[:, :1])], dim=1)
            return emb


    class DiffusionPolicy(nn.Module):
        def __init__(
            self,
            state_dim: int,
            action_dim: int,
            obs_horizon: int = 2,
            pred_horizon: int = 16,
            hidden_dim: int = 256,
            timestep_embed_dim: int = 64,
        ):
            super().__init__()
            self.state_dim = state_dim
            self.action_dim = action_dim
            self.obs_horizon = obs_horizon
            self.pred_horizon = pred_horizon

            action_size = pred_horizon * action_dim
            cond_size = obs_horizon * state_dim
            self.time_embedding = SinusoidalTimestepEmbedding(timestep_embed_dim)
            self.net = nn.Sequential(
                nn.Linear(action_size + cond_size + timestep_embed_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, action_size),
            )

        def forward(self, noisy_action: Any, cond: Any, t: Any) -> Any:
            batch_size = noisy_action.shape[0]
            x = noisy_action.reshape(batch_size, -1)
            c = cond.reshape(batch_size, -1)
            te = self.time_embedding(t)
            pred = self.net(torch.cat([x, c, te], dim=1))
            return pred.reshape(batch_size, self.pred_horizon, self.action_dim)

else:

    class DiffusionPolicy:  # type: ignore[no-redef]
        pass


class DiffusionPolicyServoNode(Node):
    RIGHT_ARM_JOINTS = [
        "right_base_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

    def __init__(self) -> None:
        super().__init__("diffusion_policy_servo_node")

        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("action_horizon", 1)
        self.declare_parameter("obs_horizon", 0)
        self.declare_parameter("pred_horizon", 0)
        self.declare_parameter("state_dim", 0)
        self.declare_parameter("action_dim", 0)
        self.declare_parameter("diffusion_timesteps", 0)
        self.declare_parameter("beta_start", 1e-4)
        self.declare_parameter("beta_end", 0.02)
        self.declare_parameter("hidden_dim", 0)
        self.declare_parameter("timestep_embed_dim", 0)
        self.declare_parameter("action_scale", 1.0)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("max_joint_delta", 0.05)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("use_normalizer", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "right_joint_topic",
            "/right_servo/moveit_servo/delta_joint_cmds",
        )
        self.declare_parameter(
            "right_start_service",
            "/right_servo/moveit_servo/start_servo",
        )
        self.declare_parameter("start_servo", True)
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("command_timeout", 0.2)
        self.declare_parameter("joint_state_timeout", 0.5)
        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("action_type", "target_position")
        self.declare_parameter("emergency_stop", False)
        self.declare_parameter(
            "emergency_stop_topic",
            "/diffusion_policy_servo/emergency_stop",
        )
        self.declare_parameter("state_joint_names", self.RIGHT_ARM_JOINTS)
        self.declare_parameter("action_joint_names", self.RIGHT_ARM_JOINTS)

        checkpoint_path_value = str(self.get_parameter("checkpoint_path").value)
        if not checkpoint_path_value:
            raise ValueError("checkpoint_path parameter is required")
        self.checkpoint_path = Path(checkpoint_path_value)
        self.requested_device = str(self.get_parameter("device").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.action_horizon = int(self.get_parameter("action_horizon").value)
        self.action_scale = float(self.get_parameter("action_scale").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.max_joint_delta = float(self.get_parameter("max_joint_delta").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.use_normalizer = bool(self.get_parameter("use_normalizer").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.right_joint_topic = str(self.get_parameter("right_joint_topic").value)
        self.right_start_service = str(
            self.get_parameter("right_start_service").value
        )
        self.start_servo_on_active = bool(self.get_parameter("start_servo").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.joint_state_timeout = float(
            self.get_parameter("joint_state_timeout").value
        )
        self.startup_delay_sec = float(
            self.get_parameter("startup_delay_sec").value
        )
        self.action_type = str(self.get_parameter("action_type").value)
        self.emergency_stop = bool(self.get_parameter("emergency_stop").value)
        self.state_joint_names = self._string_array_param(
            "state_joint_names",
            self.RIGHT_ARM_JOINTS,
        )
        self.action_joint_names = self._string_array_param(
            "action_joint_names",
            self.RIGHT_ARM_JOINTS,
        )

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be positive")
        if self.action_horizon <= 0:
            raise ValueError("action_horizon must be positive")
        if self.action_type not in {"target_position", "joint_delta", "joint_velocity"}:
            raise ValueError(
                "action_type must be one of: target_position, joint_delta, "
                "joint_velocity"
            )

        self.model: Any = None
        self.model_kind = "diffusion"
        self.config: Dict[str, Any] = {}
        self.stats: Dict[str, Any] = {}
        self.normalizer: Dict[str, Optional[np.ndarray]] = {}
        self.device: Any = None
        self.obs_horizon = 0
        self.pred_horizon = 0
        self.state_dim = 0
        self.action_dim = 0
        self.diffusion_timesteps = 0
        self.beta_start = 1e-4
        self.beta_end = 0.02

        self._load_checkpoint()
        self._validate_training_contract()

        self.obs_buffer: Deque[np.ndarray] = deque(maxlen=self.obs_horizon)
        self.pending_actions: Deque[np.ndarray] = deque()
        self.latest_joint_positions: Dict[str, float] = {}
        self.latest_joint_velocities: Dict[str, float] = {}
        self.last_joint_state_time = 0.0
        self.last_summary_time = 0.0
        self.last_action_raw: Optional[np.ndarray] = None
        self.last_action_clipped: Optional[np.ndarray] = None
        self.last_publish_state = "not_started"
        self.zero_sent = True
        self.start_time = time.monotonic()

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            20,
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.emergency_stop_callback,
            10,
        )
        self.right_joint_pub = self.create_publisher(
            JointJog,
            self.right_joint_topic,
            10,
        )

        self._print_startup_summary()

        if not self.dry_run and self.start_servo_on_active:
            self.start_servo()

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

    def _string_array_param(self, name: str, fallback: List[str]) -> List[str]:
        value = self.get_parameter(name).value
        if value is None:
            return list(fallback)
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        if isinstance(value, str) and value:
            return [part.strip() for part in value.split(",") if part.strip()]
        return list(fallback)

    def _load_checkpoint(self) -> None:
        if TORCH_IMPORT_ERROR is not None or torch is None or nn is None:
            raise RuntimeError(
                "Failed to import torch in this ROS Python environment. "
                "Check with: python3 -c \"import torch; print(torch.__version__)\". "
                f"Original error: {TORCH_IMPORT_ERROR}"
            )
        if not self.checkpoint_path.is_file():
            raise FileNotFoundError(f"checkpoint_path not found: {self.checkpoint_path}")

        requested = self.requested_device.lower()
        if requested.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn("CUDA requested but unavailable; falling back to CPU.")
            requested = "cpu"
        self.device = torch.device(requested)

        self.get_logger().info(f"Loading checkpoint: {self.checkpoint_path}")
        try:
            checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        except Exception as exc:
            self.get_logger().warn(
                "torch.load default mode failed; retrying with weights_only=False. "
                f"Original error: {exc}"
            )
            checkpoint = torch.load(
                self.checkpoint_path,
                map_location=self.device,
                weights_only=False,
            )

        self.config = self._extract_config(checkpoint)
        self.stats = self._extract_stats(checkpoint)
        self.normalizer = self._extract_normalizer(checkpoint, self.stats)
        self.model, self.model_kind = self._build_model(checkpoint)
        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info("Model loaded successfully.")

    def _extract_config(self, checkpoint: Any) -> Dict[str, Any]:
        if isinstance(checkpoint, Mapping) and isinstance(
            checkpoint.get("config"),
            Mapping,
        ):
            return dict(checkpoint["config"])
        return {}

    def _extract_stats(self, checkpoint: Any) -> Dict[str, Any]:
        if not isinstance(checkpoint, Mapping):
            return {}
        for key in ("stats", "normalizer"):
            value = checkpoint.get(key)
            if isinstance(value, Mapping):
                return dict(value)
        return {}

    def _extract_normalizer(
        self,
        checkpoint: Any,
        stats: Mapping[str, Any],
    ) -> Dict[str, Optional[np.ndarray]]:
        sources: List[Mapping[str, Any]] = []
        if isinstance(stats, Mapping):
            sources.append(stats)
        if isinstance(checkpoint, Mapping):
            sources.append(checkpoint)
            for key in ("normalizer", "stats"):
                if isinstance(checkpoint.get(key), Mapping):
                    sources.append(checkpoint[key])

        return {
            "obs_mean": self._find_array(
                sources,
                ("robot_state_mean", "state_mean", "obs_mean", "observation_mean"),
            ),
            "obs_std": self._find_array(
                sources,
                ("robot_state_std", "state_std", "obs_std", "observation_std"),
            ),
            "action_mean": self._find_array(
                sources,
                ("action_mean", "act_mean"),
            ),
            "action_std": self._find_array(
                sources,
                ("action_std", "act_std"),
            ),
        }

    def _find_array(
        self,
        sources: List[Mapping[str, Any]],
        keys: Tuple[str, ...],
    ) -> Optional[np.ndarray]:
        for source in sources:
            for key in keys:
                if key in source:
                    return self._to_numpy(source[key])
            for nested_key in ("obs", "state", "robot_state", "action"):
                nested = source.get(nested_key)
                if isinstance(nested, Mapping):
                    for key in keys:
                        if key in nested:
                            return self._to_numpy(nested[key])
        return None

    def _to_numpy(self, value: Any) -> np.ndarray:
        if torch is not None and isinstance(value, torch.Tensor):
            return value.detach().cpu().numpy().astype(np.float32)
        return np.asarray(value, dtype=np.float32)

    def _build_model(self, checkpoint: Any) -> Tuple[Any, str]:
        direct_model = self._extract_direct_model(checkpoint)
        if direct_model is not None:
            self._resolve_direct_model_metadata(direct_model)
            model_kind = (
                "diffusion"
                if hasattr(direct_model, "pred_horizon")
                and hasattr(direct_model, "action_dim")
                else "direct"
            )
            return direct_model, model_kind

        state_dict = self._extract_state_dict(checkpoint)
        if state_dict is None:
            raise ValueError(
                "Unsupported checkpoint format: expected model_state_dict, model, "
                "or a direct state_dict."
            )
        state_dict = self._strip_module_prefix(state_dict)

        state_dim, action_dim = self._resolve_dims(state_dict)
        obs_horizon = self._resolve_int(
            "obs_horizon",
            self.config,
            self.stats,
            int(self.get_parameter("obs_horizon").value),
            default=2,
        )
        pred_horizon = self._resolve_int(
            "pred_horizon",
            self.config,
            self.stats,
            int(self.get_parameter("pred_horizon").value),
            default=16,
        )
        hidden_dim = self._resolve_hidden_dim(state_dict)
        timestep_embed_dim = self._resolve_timestep_embed_dim(
            state_dict,
            state_dim,
            action_dim,
            obs_horizon,
            pred_horizon,
        )

        self.state_dim = state_dim
        self.action_dim = action_dim
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon
        self.diffusion_timesteps = self._resolve_int(
            "timesteps",
            self.config,
            self.stats,
            int(self.get_parameter("diffusion_timesteps").value),
            default=100,
        )
        self.beta_start = float(
            self.config.get("beta_start", self.get_parameter("beta_start").value)
        )
        self.beta_end = float(
            self.config.get("beta_end", self.get_parameter("beta_end").value)
        )

        model = DiffusionPolicy(
            state_dim=state_dim,
            action_dim=action_dim,
            obs_horizon=obs_horizon,
            pred_horizon=pred_horizon,
            hidden_dim=hidden_dim,
            timestep_embed_dim=timestep_embed_dim,
        )
        model.load_state_dict(state_dict)
        return model, "diffusion"

    def _resolve_direct_model_metadata(self, model: Any) -> None:
        state_dim_param = int(self.get_parameter("state_dim").value)
        action_dim_param = int(self.get_parameter("action_dim").value)
        obs_horizon_param = int(self.get_parameter("obs_horizon").value)
        pred_horizon_param = int(self.get_parameter("pred_horizon").value)

        self.state_dim = self._resolve_int(
            "state_dim",
            self.config,
            self.stats,
            state_dim_param,
            default=int(getattr(model, "state_dim", 0) or 0),
        )
        self.action_dim = self._resolve_int(
            "action_dim",
            self.config,
            self.stats,
            action_dim_param,
            default=int(getattr(model, "action_dim", 0) or 0),
        )
        self.obs_horizon = self._resolve_int(
            "obs_horizon",
            self.config,
            self.stats,
            obs_horizon_param,
            default=int(getattr(model, "obs_horizon", 2) or 2),
        )
        self.pred_horizon = self._resolve_int(
            "pred_horizon",
            self.config,
            self.stats,
            pred_horizon_param,
            default=int(getattr(model, "pred_horizon", 1) or 1),
        )
        self.diffusion_timesteps = self._resolve_int(
            "timesteps",
            self.config,
            self.stats,
            int(self.get_parameter("diffusion_timesteps").value),
            default=100,
        )
        self.beta_start = float(
            self.config.get("beta_start", self.get_parameter("beta_start").value)
        )
        self.beta_end = float(
            self.config.get("beta_end", self.get_parameter("beta_end").value)
        )

        if self.state_dim <= 0 and self.normalizer["obs_mean"] is not None:
            self.state_dim = int(self.normalizer["obs_mean"].shape[0])
        if self.action_dim <= 0 and self.normalizer["action_mean"] is not None:
            self.action_dim = int(self.normalizer["action_mean"].shape[0])

    def _extract_direct_model(self, checkpoint: Any) -> Optional[Any]:
        if isinstance(checkpoint, nn.Module):
            return checkpoint
        if isinstance(checkpoint, Mapping):
            model = checkpoint.get("model")
            if isinstance(model, nn.Module):
                return model
        return None

    def _extract_state_dict(self, checkpoint: Any) -> Optional[Mapping[str, Any]]:
        if self._looks_like_state_dict(checkpoint):
            return checkpoint
        if not isinstance(checkpoint, Mapping):
            return None
        for key in ("model_state_dict", "state_dict", "model"):
            value = checkpoint.get(key)
            if self._looks_like_state_dict(value):
                return value
        return None

    def _looks_like_state_dict(self, value: Any) -> bool:
        if not isinstance(value, Mapping):
            return False
        return bool(value) and all(isinstance(v, torch.Tensor) for v in value.values())

    def _strip_module_prefix(self, state_dict: Mapping[str, Any]) -> Mapping[str, Any]:
        if not any(key.startswith("module.") for key in state_dict.keys()):
            return state_dict
        stripped = OrderedDict()
        for key, value in state_dict.items():
            stripped[key.removeprefix("module.")] = value
        return stripped

    def _resolve_dims(self, state_dict: Mapping[str, Any]) -> Tuple[int, int]:
        state_dim = self._resolve_int(
            "state_dim",
            self.config,
            self.stats,
            int(self.get_parameter("state_dim").value),
            default=0,
        )
        action_dim = self._resolve_int(
            "action_dim",
            self.config,
            self.stats,
            int(self.get_parameter("action_dim").value),
            default=0,
        )
        if state_dim <= 0 and self.normalizer["obs_mean"] is not None:
            state_dim = int(self.normalizer["obs_mean"].shape[0])
        if action_dim <= 0 and self.normalizer["action_mean"] is not None:
            action_dim = int(self.normalizer["action_mean"].shape[0])

        last_bias = state_dict.get("net.4.bias")
        pred_horizon_param = int(self.get_parameter("pred_horizon").value)
        pred_horizon = int(
            self.config.get(
                "pred_horizon",
                self.stats.get("pred_horizon", pred_horizon_param),
            )
            or 0
        )
        if action_dim <= 0 and last_bias is not None and pred_horizon > 0:
            action_dim = int(last_bias.numel() // pred_horizon)
        if action_dim <= 0:
            raise ValueError("Could not infer action_dim from checkpoint or parameters.")
        if state_dim <= 0:
            raise ValueError("Could not infer state_dim from checkpoint or parameters.")
        return state_dim, action_dim

    def _resolve_int(
        self,
        key: str,
        config: Mapping[str, Any],
        stats: Mapping[str, Any],
        param_value: int,
        default: int,
    ) -> int:
        if param_value > 0:
            return int(param_value)
        if key in config:
            return int(config[key])
        if key in stats:
            return int(stats[key])
        return int(default)

    def _resolve_hidden_dim(self, state_dict: Mapping[str, Any]) -> int:
        param_value = int(self.get_parameter("hidden_dim").value)
        if param_value > 0:
            return param_value
        first_bias = state_dict.get("net.0.bias")
        if first_bias is not None:
            return int(first_bias.numel())
        return 256

    def _resolve_timestep_embed_dim(
        self,
        state_dict: Mapping[str, Any],
        state_dim: int,
        action_dim: int,
        obs_horizon: int,
        pred_horizon: int,
    ) -> int:
        param_value = int(self.get_parameter("timestep_embed_dim").value)
        if param_value > 0:
            return param_value
        first_weight = state_dict.get("net.0.weight")
        if first_weight is not None:
            input_dim = int(first_weight.shape[1])
            action_size = pred_horizon * action_dim
            cond_size = obs_horizon * state_dim
            inferred = input_dim - action_size - cond_size
            if inferred > 0:
                return inferred
        return 64

    def _validate_training_contract(self) -> None:
        expected_state_dim = len(self.state_joint_names) * 2
        if self.state_dim != expected_state_dim:
            raise ValueError(
                f"state_dim mismatch: checkpoint expects {self.state_dim}, "
                f"but joint-state observation builder produces {expected_state_dim}. "
                "This node currently supports right-arm joint position+velocity "
                "observations only."
            )
        if self.action_dim != len(self.action_joint_names):
            raise ValueError(
                f"action_dim mismatch: checkpoint expects {self.action_dim}, "
                f"but action_joint_names has {len(self.action_joint_names)} joints."
            )
        if self.action_horizon > self.pred_horizon:
            self.get_logger().warn(
                f"action_horizon={self.action_horizon} exceeds pred_horizon="
                f"{self.pred_horizon}; using {self.pred_horizon}."
            )
            self.action_horizon = self.pred_horizon
        if self.use_normalizer:
            missing = [
                key
                for key in ("obs_mean", "obs_std", "action_mean", "action_std")
                if self.normalizer[key] is None
            ]
            if missing:
                self.get_logger().warn(
                    "Checkpoint has incomplete normalization values "
                    f"({missing}); inference will use raw tensors for missing parts."
                )
        else:
            self.get_logger().warn("use_normalizer=false; inference uses raw values.")

    def _print_startup_summary(self) -> None:
        self.get_logger().info(
            "\n"
            "Diffusion policy servo node ready.\n"
            f"  checkpoint_path: {self.checkpoint_path}\n"
            f"  device: {self.device}\n"
            f"  model_kind: {self.model_kind}\n"
            f"  obs_dim/action_dim: {self.state_dim}/{self.action_dim}\n"
            f"  obs_horizon/action_horizon/pred_horizon: "
            f"{self.obs_horizon}/{self.action_horizon}/{self.pred_horizon}\n"
            f"  joint_state_topic: {self.joint_state_topic}\n"
            f"  servo command topic: {self.right_joint_topic}\n"
            f"  joint name order: {self.action_joint_names}\n"
            f"  action_type: {self.action_type}\n"
            f"  dry_run: {self.dry_run}\n"
            f"  action_scale: {self.action_scale}\n"
            f"  max_joint_delta/max_velocity: "
            f"{self.max_joint_delta}/{self.max_velocity}"
        )

    def start_servo(self) -> None:
        client = self.create_client(Trigger, self.right_start_service)
        self.get_logger().info(f"Waiting for servo service: {self.right_start_service}")
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"Servo start service unavailable: {self.right_start_service}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Failed to call Servo start service")
        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            raise RuntimeError(f"Servo failed to start: {message}")
        self.get_logger().info(f"Servo started: {result.message}")

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        missing = [name for name in self.state_joint_names if name not in name_to_index]
        if missing:
            self.get_logger().warn(
                f"Missing joints in {self.joint_state_topic}: {missing}",
                throttle_duration_sec=2.0,
            )
            return

        positions = []
        velocities = []
        for joint_name in self.state_joint_names:
            idx = name_to_index[joint_name]
            positions.append(float(msg.position[idx]))
            if idx < len(msg.velocity):
                velocities.append(float(msg.velocity[idx]))
            else:
                velocities.append(0.0)
                self.get_logger().warn(
                    "JointState velocity array is missing or short; using zeros.",
                    throttle_duration_sec=5.0,
                )

        self.latest_joint_positions = dict(zip(self.state_joint_names, positions))
        self.latest_joint_velocities = dict(zip(self.state_joint_names, velocities))
        state = np.asarray(positions + velocities, dtype=np.float32)
        if not np.isfinite(state).all():
            self.get_logger().error("Received non-finite joint state; ignoring sample.")
            return
        self.obs_buffer.append(state)
        self.last_joint_state_time = time.monotonic()

    def emergency_stop_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.stop_motion("emergency_stop topic is true")

    def control_loop(self) -> None:
        ready, reason = self._safety_ready()
        if not ready:
            self.pending_actions.clear()
            self.stop_motion(reason)
            self._maybe_print_summary()
            return

        if not self.pending_actions:
            try:
                action_seq = self._predict_action_sequence()
            except Exception as exc:
                self.get_logger().error(f"Model inference failed: {exc}")
                self.stop_motion("inference_error")
                self._maybe_print_summary()
                return

            for action in action_seq[: self.action_horizon]:
                self.pending_actions.append(action)

        if not self.pending_actions:
            self.stop_motion("empty_action_sequence")
            self._maybe_print_summary()
            return

        action = self.pending_actions.popleft()
        self.last_action_raw = action

        if not np.isfinite(action).all():
            self.get_logger().error("Model output contains NaN or Inf; skipping publish.")
            self.stop_motion("non_finite_action")
            self._maybe_print_summary()
            return

        try:
            velocities = self._action_to_joint_velocities(action)
        except Exception as exc:
            self.get_logger().error(f"Action conversion failed: {exc}")
            self.stop_motion("action_conversion_error")
            self._maybe_print_summary()
            return

        self.last_action_clipped = velocities
        if self.dry_run:
            self.last_publish_state = "dry_run_no_publish"
        else:
            self.publish_joint_jog(velocities)
            self.last_publish_state = "published"
            self.zero_sent = False

        self._maybe_print_summary()

    def _safety_ready(self) -> Tuple[bool, str]:
        elapsed = time.monotonic() - self.start_time
        if elapsed < self.startup_delay_sec:
            return False, f"startup_delay {elapsed:.2f}/{self.startup_delay_sec:.2f}s"
        if self.emergency_stop or bool(self.get_parameter("emergency_stop").value):
            return False, "emergency_stop"
        if len(self.obs_buffer) < self.obs_horizon:
            return False, (
                f"waiting for observation buffer "
                f"{len(self.obs_buffer)}/{self.obs_horizon}"
            )
        if not self.last_joint_state_time:
            return False, "waiting for joint_states"
        stale = time.monotonic() - self.last_joint_state_time
        if stale > self.joint_state_timeout:
            return False, f"joint_states stale ({stale:.3f}s)"
        return True, "ready"

    def _predict_action_sequence(self) -> np.ndarray:
        obs = np.stack(list(self.obs_buffer), axis=0).astype(np.float32)
        obs_for_model = self._normalize_obs(obs)
        cond = torch.as_tensor(obs_for_model, dtype=torch.float32, device=self.device)
        cond = cond.unsqueeze(0)

        with torch.no_grad():
            if self.model_kind == "diffusion":
                output = self._ddpm_sample(cond)
            else:
                output = self.model(cond)

        action = self._output_to_action_sequence(output)
        action = self._denormalize_action(action)
        return action

    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        if not self.use_normalizer or self.normalizer["obs_mean"] is None:
            return obs
        obs_std = self.normalizer["obs_std"]
        if obs_std is None:
            return obs
        return (obs - self.normalizer["obs_mean"]) / obs_std

    def _denormalize_action(self, action: np.ndarray) -> np.ndarray:
        if not self.use_normalizer or self.normalizer["action_mean"] is None:
            return action
        action_std = self.normalizer["action_std"]
        if action_std is None:
            return action
        return action * action_std + self.normalizer["action_mean"]

    def _ddpm_sample(self, cond: Any) -> Any:
        batch_size = cond.shape[0]
        betas = torch.linspace(
            self.beta_start,
            self.beta_end,
            self.diffusion_timesteps,
            device=self.device,
        )
        alphas = 1.0 - betas
        alphas_cumprod = torch.cumprod(alphas, dim=0)
        x = torch.randn(
            batch_size,
            self.pred_horizon,
            self.action_dim,
            device=self.device,
        )

        for t_idx in reversed(range(self.diffusion_timesteps)):
            t = torch.full((batch_size,), t_idx, device=self.device, dtype=torch.long)
            pred_noise = self.model(x, cond, t)
            alpha = alphas[t_idx]
            alpha_bar = alphas_cumprod[t_idx]
            beta = betas[t_idx]
            x = (1.0 / torch.sqrt(alpha)) * (
                x - (beta / torch.sqrt(1.0 - alpha_bar)) * pred_noise
            )
            if t_idx > 0:
                x = x + torch.sqrt(beta) * torch.randn_like(x)
        return x

    def _output_to_action_sequence(self, output: Any) -> np.ndarray:
        if isinstance(output, torch.Tensor):
            arr = output.detach().cpu().numpy().astype(np.float32)
        else:
            arr = np.asarray(output, dtype=np.float32)

        if arr.ndim == 3:
            arr = arr[0]
        elif arr.ndim == 2:
            if arr.shape[0] == 1 and arr.shape[1] == self.action_dim:
                arr = arr[0][None, :]
            elif arr.shape[1] != self.action_dim:
                raise ValueError(f"Unsupported action output shape: {arr.shape}")
        elif arr.ndim == 1:
            arr = arr[None, :]
        else:
            raise ValueError(f"Unsupported action output rank: {arr.ndim}")

        if arr.shape[-1] != self.action_dim:
            raise ValueError(
                f"Model output action_dim={arr.shape[-1]}, expected {self.action_dim}"
            )
        return arr

    def _action_to_joint_velocities(self, action: np.ndarray) -> np.ndarray:
        if action.shape[0] != len(self.action_joint_names):
            raise ValueError(
                f"Action size {action.shape[0]} does not match action_joint_names "
                f"size {len(self.action_joint_names)}"
            )

        if self.action_type == "target_position":
            current = np.asarray(
                [self.latest_joint_positions[name] for name in self.action_joint_names],
                dtype=np.float32,
            )
            delta = (action.astype(np.float32) - current) * self.action_scale
        elif self.action_type == "joint_delta":
            delta = action.astype(np.float32) * self.action_scale
        else:
            velocities = action.astype(np.float32) * self.action_scale
            return self._clip_velocity(velocities)

        clipped_delta = np.clip(delta, -self.max_joint_delta, self.max_joint_delta)
        if not np.allclose(delta, clipped_delta):
            self.get_logger().warn(
                "Action delta exceeded max_joint_delta; clipped before publish.",
                throttle_duration_sec=1.0,
            )
        velocities = clipped_delta * self.control_rate
        return self._clip_velocity(velocities)

    def _clip_velocity(self, velocities: np.ndarray) -> np.ndarray:
        clipped = np.clip(velocities, -self.max_velocity, self.max_velocity)
        if not np.allclose(velocities, clipped):
            self.get_logger().warn(
                "Action velocity exceeded max_velocity; clipped before publish.",
                throttle_duration_sec=1.0,
            )
        return clipped.astype(np.float32)

    def publish_joint_jog(self, velocities: np.ndarray) -> None:
        if self.right_joint_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f"No subscribers on {self.right_joint_topic}; command may be ignored.",
                throttle_duration_sec=2.0,
            )

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.joint_names = list(self.action_joint_names)
        msg.velocities = [float(v) for v in velocities]
        msg.duration = self.command_timeout
        self.right_joint_pub.publish(msg)

    def stop_motion(self, reason: str) -> None:
        self.last_publish_state = f"stopped: {reason}"
        if self.dry_run or self.zero_sent:
            return
        zeros = np.zeros(len(self.action_joint_names), dtype=np.float32)
        self.publish_joint_jog(zeros)
        self.zero_sent = True

    def _maybe_print_summary(self) -> None:
        now = time.monotonic()
        if now - self.last_summary_time < 1.0:
            return
        self.last_summary_time = now

        obs_shape = (len(self.obs_buffer), self.state_dim)
        action_text = "none"
        if self.last_action_raw is not None:
            raw = self.last_action_raw
            clipped = (
                self.last_action_clipped
                if self.last_action_clipped is not None
                else np.asarray([])
            )
            action_text = (
                f"raw_min={raw.min():+.4f}, raw_max={raw.max():+.4f}, "
                f"raw_norm={np.linalg.norm(raw):.4f}, "
                f"cmd_min={clipped.min():+.4f}, cmd_max={clipped.max():+.4f}, "
                f"cmd_norm={np.linalg.norm(clipped):.4f}"
                if clipped.size
                else f"raw_min={raw.min():+.4f}, raw_max={raw.max():+.4f}"
            )

        self.get_logger().info(
            f"obs_shape={obs_shape}, {action_text}, "
            f"dry_run={self.dry_run}, publish_state={self.last_publish_state}"
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[DiffusionPolicyServoNode] = None
    try:
        node = DiffusionPolicyServoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_motion("shutdown")
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
