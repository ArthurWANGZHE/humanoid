from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from .config import RobotTrainingConfig
from .isaac_import import import_urdf, verify_runtime_urdf
from .pose_math import Pose, matrix_to_quat_wxyz, quat_wxyz_to_matrix
from .urdf_validation import build_link_chain, validate_urdf_inputs


_SIMULATION_APP = None


def get_simulation_app(headless: bool):
    global _SIMULATION_APP
    if _SIMULATION_APP is not None:
        print("[lifecycle] reusing SimulationApp", {"headless": headless})
        return _SIMULATION_APP

    try:
        from isaacsim import SimulationApp
    except ImportError:
        from omni.isaac.kit import SimulationApp

    print("[lifecycle] creating SimulationApp", {"headless": headless})
    _SIMULATION_APP = SimulationApp({"headless": headless})
    return _SIMULATION_APP


def load_isaac_modules(include_lula: bool = True) -> dict[str, object]:
    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.prims import get_prim_at_path
        from isaacsim.core.utils.types import ArticulationAction
    except ImportError:
        from omni.isaac.core import World
        from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
        from omni.isaac.core.articulations import Articulation as SingleArticulation
        from omni.isaac.core.utils.prims import get_prim_at_path
        from omni.isaac.core.utils.types import ArticulationAction

    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    modules = {
        "World": World,
        "DynamicCuboid": DynamicCuboid,
        "FixedCuboid": FixedCuboid,
        "SingleArticulation": SingleArticulation,
        "get_prim_at_path": get_prim_at_path,
        "ArticulationAction": ArticulationAction,
        "Gf": Gf,
        "PhysicsSchemaTools": PhysicsSchemaTools,
        "PhysxSchema": PhysxSchema,
        "Usd": Usd,
        "UsdGeom": UsdGeom,
        "UsdPhysics": UsdPhysics,
        "UsdShade": UsdShade,
    }
    if include_lula:
        from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver

        modules["LulaKinematicsSolver"] = LulaKinematicsSolver
    return modules


def _parse_actuated_joint_names(urdf_path: Path) -> list[str]:
    root = ET.parse(urdf_path).getroot()
    actuated = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type", "fixed") in {"fixed", "floating", "planar"}:
            continue
        actuated.append(joint.attrib["name"])
    return actuated


def _parse_urdf_link_names(urdf_path: Path) -> set[str]:
    root = ET.parse(urdf_path).getroot()
    return {link.attrib["name"] for link in root.findall("link")}


def parse_joint_origins(urdf_path: Path) -> dict[str, np.ndarray]:
    root = ET.parse(urdf_path).getroot()
    joint_origins: dict[str, np.ndarray] = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = np.zeros(3, dtype=np.float64)
        if origin is not None:
            xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ", dtype=np.float64)
        joint_origins[joint.attrib["name"]] = xyz
    return joint_origins


def validate_lula_inputs(training_config: RobotTrainingConfig) -> None:
    inspection = validate_urdf_inputs(
        training_config.runtime_urdf_path,
        root_link=training_config.urdf_root_link,
        end_effector_link=training_config.end_effector_link,
    )
    chosen_root_link = training_config.urdf_root_link
    chosen_ee_link = training_config.end_effector_link
    link_chain = build_link_chain(
        training_config.runtime_urdf_path,
        chosen_root_link,
        chosen_ee_link,
    )

    print(
        "[demo] lula config validation",
        {
            "root_link": chosen_root_link,
            "end_effector_link": chosen_ee_link,
            "source_urdf_path": str(training_config.urdf_path),
            "resolved_urdf_path": str(training_config.runtime_urdf_path),
            "link_chain": link_chain,
            "link_count": inspection.total_links,
            "joint_count": inspection.total_joints,
        },
    )


def materialize_lula_robot_description(training_config: RobotTrainingConfig) -> Path:
    runtime_dir = Path(tempfile.gettempdir()) / "rl_train"
    runtime_dir.mkdir(parents=True, exist_ok=True)
    descriptor_path = runtime_dir / "humanoid_right_arm_lula_descriptor.yaml"

    all_actuated = _parse_actuated_joint_names(training_config.urdf_path)
    fixed_rules = []
    for joint_name in all_actuated:
        if joint_name in training_config.arm_joints:
            continue
        fixed_value = training_config.home_joint_positions.get(
            joint_name, training_config.closed_gripper_positions.get(joint_name, 0.0)
        )
        fixed_rules.append(f"  - {{name: {joint_name}, rule: fixed, value: {fixed_value:.6f}}}")

    default_q = ", ".join(
        f"{training_config.home_joint_positions.get(joint_name, 0.0):.6f}"
        for joint_name in training_config.arm_joints
    )
    accel_limits = ", ".join("20.0" for _ in training_config.arm_joints)
    jerk_limits = ", ".join("200.0" for _ in training_config.arm_joints)
    lines = [
        "api_version: 1.0",
        "",
        "cspace:",
        *[f"  - {joint_name}" for joint_name in training_config.arm_joints],
        "",
        f"root_link: {training_config.urdf_root_link}",
        "",
        f"default_q: [{default_q}]",
        f"acceleration_limits: [{accel_limits}]",
        f"jerk_limits: [{jerk_limits}]",
        "",
    ]
    if fixed_rules:
        lines.extend(["cspace_to_urdf_rules:", *fixed_rules])
    else:
        lines.append("cspace_to_urdf_rules: []")
    lines.extend(["", "collision_spheres: []"])

    descriptor_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return descriptor_path


@dataclass(frozen=True)
class DemoSceneConfig:
    physics_dt: float = 1.0 / 60.0
    control_dt: float = 1.0 / 30.0
    table_position: tuple[float, float, float] = (0.48, -0.05, 0.28)
    table_scale: tuple[float, float, float] = (0.62, 0.50, 0.045)
    brick_position: tuple[float, float, float] = (0.42, -0.28, 0.355)
    brick_scale: tuple[float, float, float] = (0.08, 0.04, 0.05)
    place_position: tuple[float, float, float] = (0.42, -0.14, 0.355)
    pushcube_table_color_rgb: tuple[float, float, float] = (0.78, 0.72, 0.64)
    pushcube_cube_mass: float = 0.20
    pushcube_cube_static_friction: float = 1.0
    pushcube_cube_dynamic_friction: float = 0.8
    pushcube_cube_restitution: float = 0.0
    pushcube_cube_linear_damping: float = 0.2
    pushcube_cube_angular_damping: float = 0.2
    pushcube_cube_contact_offset: float = 0.003
    pushcube_cube_rest_offset: float = 0.0
    pushcube_cube_solver_position_iterations: int = 16
    pushcube_cube_solver_velocity_iterations: int = 4
    pushcube_table_static_friction: float = 1.0
    pushcube_table_dynamic_friction: float = 0.8
    pushcube_table_restitution: float = 0.0
    pushcube_table_contact_offset: float = 0.003
    pushcube_table_rest_offset: float = 0.0
    pushcube_gripper_static_friction: float = 1.0
    pushcube_gripper_dynamic_friction: float = 0.8
    pushcube_gripper_restitution: float = 0.0
    pushcube_gripper_contact_offset: float = 0.003
    pushcube_gripper_rest_offset: float = 0.0
    pushcube_target_size_xy: tuple[float, float] = (0.16, 0.16)
    pushcube_spawn_range_size_xy: tuple[float, float] = (0.18, 0.14)
    pushcube_target_range_size_xy: tuple[float, float] = (0.22, 0.18)
    pushcube_spawn_center_offset_xy: tuple[float, float] = (-0.08, -0.03)
    pushcube_target_center_offset_xy: tuple[float, float] = (0.08, 0.06)
    pushcube_overlay_thickness: float = 0.002
    pushcube_overlay_z_offset: float = 0.002

    @property
    def table_height(self) -> float:
        return self.table_position[2] + (self.table_scale[2] / 2.0)


@dataclass(frozen=True)
class PushCubeSceneOptions:
    enabled: bool = False
    brick_size: float = 0.06
    show_ranges: bool = False
    presentation: bool = False
    cube_margin: float = 0.08
    target_margin: float = 0.06
    cube_offset_from_table_center: tuple[float, float] | None = None
    target_offset_from_table_center: tuple[float, float] | None = None
    target_size_xy: tuple[float, float] | None = None


@dataclass(frozen=True)
class MountedCameraSpec:
    camera_name: str
    mount_link: str
    translation_xyz: tuple[float, float, float]
    rotation_xyz_deg: tuple[float, float, float]
    validation_joint_name: str


class HumanoidBrickPickDemoScene:
    def __init__(
        self,
        training_config: RobotTrainingConfig,
        headless: bool,
        setup_cameras: bool = True,
        enable_lula: bool = True,
        pushcube_options: PushCubeSceneOptions | None = None,
    ) -> None:
        self.training_config = training_config
        self.headless = headless
        self.setup_cameras = setup_cameras
        self.enable_lula = enable_lula
        self.scene_config = DemoSceneConfig()
        self.pushcube_options = pushcube_options or PushCubeSceneOptions()
        self.pushcube_show_ranges = bool(self.pushcube_options.show_ranges)
        self.pushcube_layout_enabled = (
            self.pushcube_options.enabled
            or self.pushcube_options.show_ranges
            or self.pushcube_options.presentation
        )
        self._app = get_simulation_app(headless)
        self._modules = load_isaac_modules(include_lula=enable_lula)

        self._World = self._modules["World"]
        self._DynamicCuboid = self._modules["DynamicCuboid"]
        self._FixedCuboid = self._modules["FixedCuboid"]
        self._SingleArticulation = self._modules["SingleArticulation"]
        self._get_prim_at_path = self._modules["get_prim_at_path"]
        self._ArticulationAction = self._modules["ArticulationAction"]
        self._LulaKinematicsSolver = self._modules.get("LulaKinematicsSolver")
        self._Gf = self._modules["Gf"]
        self._PhysicsSchemaTools = self._modules["PhysicsSchemaTools"]
        self._PhysxSchema = self._modules["PhysxSchema"]
        self._Usd = self._modules["Usd"]
        self._UsdGeom = self._modules["UsdGeom"]
        self._UsdPhysics = self._modules["UsdPhysics"]
        self._UsdShade = self._modules["UsdShade"]
        self.layout_visual_prim_paths: dict[str, str] = {}
        self.layout_visibility_state: dict[str, bool] = {
            "target_visible": False,
            "cube_spawn_range_visible": False,
            "target_range_visible": False,
            "ranges_visible": False,
        }
        self.pushcube_layout: dict[str, object] | None = None
        self.layout_gripper_qpos_before = np.zeros(0, dtype=np.float64)
        self.layout_gripper_qpos_after = np.zeros(0, dtype=np.float64)
        self.layout_gripper_max_abs_qpos_change = 0.0
        self.layout_gripper_qpos_restored = False
        self.camera_mount_specs = {
            "head_camera": MountedCameraSpec(
                camera_name="head_camera",
                mount_link="head_camera_link",
                translation_xyz=(0.05, 0.0, 0.05),
                rotation_xyz_deg=(0.0, -25.0, 0.0),
                validation_joint_name="neck_yaw_joint",
            ),
            "left_arm_camera": MountedCameraSpec(
                camera_name="left_arm_camera",
                mount_link="left_camera_link",
                translation_xyz=(0.045, 0.0, 0.025),
                rotation_xyz_deg=(0.0, -25.0, 0.0),
                validation_joint_name="left_wrist_yaw_joint",
            ),
            "right_arm_camera": MountedCameraSpec(
                camera_name="right_arm_camera",
                mount_link="right_camera_link",
                translation_xyz=(0.045, 0.0, 0.025),
                rotation_xyz_deg=(0.0, -25.0, 0.0),
                validation_joint_name="right_wrist_yaw_joint",
            ),
        }
        self.camera_prim_paths: dict[str, str | None] = {
            camera_name: None for camera_name in self.camera_mount_specs
        }
        self._camera_viewport_windows: list[object] = []

        self.world = self._World(
            stage_units_in_meters=1.0,
            physics_dt=self.scene_config.physics_dt,
            rendering_dt=self.scene_config.physics_dt,
        )
        self.world.scene.add_default_ground_plane()
        self.pushcube_layout = self._build_pushcube_layout()
        table_color = (
            self.scene_config.pushcube_table_color_rgb
            if self.pushcube_layout_enabled
            else (0.48, 0.32, 0.18)
        )

        self.table = self.world.scene.add(
            self._FixedCuboid(
                prim_path="/World/demo_table",
                name="demo_table",
                position=np.array(self.scene_config.table_position, dtype=np.float32),
                scale=np.array(self.scene_config.table_scale, dtype=np.float32),
                color=np.array(table_color, dtype=np.float32),
            )
        )
        brick_position = (
            self.pushcube_layout["cube_position"]
            if self.pushcube_layout_enabled and self.pushcube_layout is not None
            else np.array(self.scene_config.brick_position, dtype=np.float32)
        )
        brick_scale = (
            self.pushcube_layout["cube_scale"]
            if self.pushcube_layout_enabled and self.pushcube_layout is not None
            else np.array(self.scene_config.brick_scale, dtype=np.float32)
        )
        self.brick = self.world.scene.add(
            self._DynamicCuboid(
                prim_path="/World/demo_brick",
                name="demo_brick",
                position=np.array(brick_position, dtype=np.float32),
                scale=np.array(brick_scale, dtype=np.float32),
                color=np.array([0.85, 0.18, 0.12], dtype=np.float32),
                mass=self.scene_config.pushcube_cube_mass if self.pushcube_layout_enabled else 0.05,
            )
        )
        self.initial_brick_position = np.array(brick_position, dtype=np.float32)

        print("[demo] source URDF:", self.training_config.urdf_path)
        verify_runtime_urdf(self.training_config.runtime_urdf_path)
        self.robot_prim_path = import_urdf(self.training_config.runtime_urdf_path)
        self.robot_prim = self._get_prim_at_path(self.robot_prim_path)

        self.articulation = self.world.scene.add(
            self._SingleArticulation(prim_path=self.robot_prim_path, name="humanoid")
        )
        self.world.reset()
        self.articulation.initialize()
        self.set_robot_root_pose()

        self.dof_names = list(self.articulation.dof_names)
        self.arm_indices = [self.dof_names.index(name) for name in self.training_config.arm_joints]
        self.gripper_indices = [self.dof_names.index(name) for name in self.training_config.gripper_joints]
        if self.pushcube_layout_enabled:
            self._create_pushcube_visuals_with_pose_guard()
        self.arm_lower, self.arm_upper = self._build_arm_bounds()
        if self.setup_cameras:
            self.setup_onboard_cameras()

        self.base_prim = self._get_prim_at_path(f"{self.robot_prim_path}/base_link")
        self.ee_prim = self._get_prim_at_path(f"{self.robot_prim_path}/{self.training_config.end_effector_link}")
        self.pushcube_physics_summary: dict[str, object] = {}
        if self.pushcube_layout_enabled:
            self._apply_pushcube_physics_defaults()
        self._print_scene_layout_diagnostics()

        self.robot_descriptor_path: Path | None = None
        self.kinematics = None
        if self.enable_lula:
            validate_lula_inputs(self.training_config)
            self.robot_descriptor_path = materialize_lula_robot_description(training_config)
            self.kinematics = self._LulaKinematicsSolver(
                robot_description_path=str(self.robot_descriptor_path),
                urdf_path=str(self.training_config.runtime_urdf_path),
            )
            self.kinematics.set_default_position_tolerance(0.005)
            self.kinematics.set_default_orientation_tolerance(0.08)
            self.kinematics.set_default_cspace_seeds(
                np.array([self.home_arm_positions()], dtype=np.float64)
            )
        else:
            print("[demo] Lula disabled; scene loaded without LulaKinematicsSolver")

    def _build_pushcube_layout(self) -> dict[str, object]:
        table_center_xy = np.array(self.scene_config.table_position[:2], dtype=np.float64)
        table_size_xy = np.array(self.scene_config.table_scale[:2], dtype=np.float64)
        table_top_z = float(self.scene_config.table_height)
        cube_size = float(self.pushcube_options.brick_size)
        cube_offset_xy = self.pushcube_options.cube_offset_from_table_center
        if cube_offset_xy is None:
            cube_offset_xy = self.scene_config.pushcube_spawn_center_offset_xy
        target_offset_xy = self.pushcube_options.target_offset_from_table_center
        if target_offset_xy is None:
            target_offset_xy = self.scene_config.pushcube_target_center_offset_xy
        target_size_xy = self.pushcube_options.target_size_xy
        if target_size_xy is None:
            target_size_xy = self.scene_config.pushcube_target_size_xy
        cube_preferred_xy = table_center_xy + np.array(
            cube_offset_xy,
            dtype=np.float64,
        )
        target_preferred_xy = table_center_xy + np.array(
            target_offset_xy,
            dtype=np.float64,
        )
        cube_layout = self.place_rect_on_table(
            cube_preferred_xy,
            size_xy=np.array([cube_size, cube_size], dtype=np.float64),
            requested_margin=self.pushcube_options.cube_margin,
            fallback_margin=0.05,
            label="cube_xy",
        )
        target_layout = self.place_rect_on_table(
            target_preferred_xy,
            size_xy=np.array(target_size_xy, dtype=np.float64),
            requested_margin=self.pushcube_options.target_margin,
            fallback_margin=0.05,
            label="target_xy",
        )
        cube_center_xy = cube_layout["center_xy"]
        target_center_xy = target_layout["center_xy"]
        spawn_range_layout = self.place_rect_on_table(
            cube_center_xy,
            size_xy=np.array(self.scene_config.pushcube_spawn_range_size_xy, dtype=np.float64),
            requested_margin=0.0,
            fallback_margin=0.0,
            label="cube_spawn_range",
        )
        target_range_layout = self.place_rect_on_table(
            target_center_xy,
            size_xy=np.array(self.scene_config.pushcube_target_range_size_xy, dtype=np.float64),
            requested_margin=0.0,
            fallback_margin=0.0,
            label="target_range",
        )
        spawn_range_center_xy = spawn_range_layout["center_xy"]
        target_range_center_xy = target_range_layout["center_xy"]
        cube_z = table_top_z + (cube_size / 2.0)
        overlay_z = table_top_z + self.scene_config.pushcube_overlay_z_offset
        overlay_thickness = self.scene_config.pushcube_overlay_thickness
        table_bounds = self.table_bounds_xy(margin=0.0)

        return {
            "cube_size": cube_size,
            "cube_scale": np.array([cube_size, cube_size, cube_size], dtype=np.float32),
            "cube_position": np.array([cube_center_xy[0], cube_center_xy[1], cube_z], dtype=np.float32),
            "target_center": np.array([target_center_xy[0], target_center_xy[1], overlay_z], dtype=np.float32),
            "target_size": np.array(
                [
                    target_size_xy[0],
                    target_size_xy[1],
                    overlay_thickness,
                ],
                dtype=np.float32,
            ),
            "spawn_range_center": np.array([spawn_range_center_xy[0], spawn_range_center_xy[1], overlay_z], dtype=np.float32),
            "spawn_range_size": np.array(
                [
                    self.scene_config.pushcube_spawn_range_size_xy[0],
                    self.scene_config.pushcube_spawn_range_size_xy[1],
                    overlay_thickness,
                ],
                dtype=np.float32,
            ),
            "target_range_center": np.array([target_range_center_xy[0], target_range_center_xy[1], overlay_z], dtype=np.float32),
            "target_range_size": np.array(
                [
                    self.scene_config.pushcube_target_range_size_xy[0],
                    self.scene_config.pushcube_target_range_size_xy[1],
                    overlay_thickness,
                ],
                dtype=np.float32,
            ),
            "table_top_z": table_top_z,
            "overlay_z": overlay_z,
            "table_center_xy": table_center_xy,
            "table_size_xy": table_size_xy,
            "table_bounds": table_bounds,
            "cube_preferred_xy": cube_preferred_xy,
            "target_preferred_xy": target_preferred_xy,
            "cube_margin_to_table_edges": cube_layout["margin_to_edges"],
            "target_margin_to_table_edges": target_layout["margin_to_edges"],
            "cube_clamped": cube_layout["clamped"],
            "target_clamped": target_layout["clamped"],
            "cube_requested_margin": cube_layout["requested_margin"],
            "target_requested_margin": target_layout["requested_margin"],
            "cube_effective_margin": cube_layout["effective_margin"],
            "target_effective_margin": target_layout["effective_margin"],
            "cube_max_feasible_margin": cube_layout["max_feasible_margin"],
            "target_max_feasible_margin": target_layout["max_feasible_margin"],
            "spawn_range_clamped": spawn_range_layout["clamped"],
            "target_range_clamped": target_range_layout["clamped"],
        }

    def _create_visual_box(
        self,
        prim_path: str,
        center_xyz: np.ndarray,
        size_xyz: np.ndarray,
        color_rgb: tuple[float, float, float],
        opacity: float,
    ) -> str:
        cube_geom = self._UsdGeom.Cube.Define(self.world.stage, prim_path)
        cube_geom.CreateSizeAttr(1.0)
        cube_geom.CreateDisplayColorAttr([self._Gf.Vec3f(*[float(v) for v in color_rgb])])
        cube_geom.CreateDisplayOpacityAttr([float(opacity)])
        xform = self._UsdGeom.XformCommonAPI(cube_geom)
        xform.SetTranslate(tuple(float(v) for v in center_xyz))
        xform.SetScale(tuple(float(v) for v in size_xyz))
        return prim_path

    def _set_visual_visibility(self, prim_path: str, visible: bool) -> None:
        prim = self._get_prim_at_path(prim_path)
        if prim is None or not prim.IsValid():
            return
        imageable = self._UsdGeom.Imageable(prim)
        visibility = self._UsdGeom.Tokens.inherited if visible else self._UsdGeom.Tokens.invisible
        imageable.GetVisibilityAttr().Set(visibility)

    def _create_pushcube_visuals(self) -> None:
        if self.pushcube_layout is None:
            return

        cube_spawn_range_visible = bool(self.pushcube_show_ranges)
        target_range_visible = bool(self.pushcube_show_ranges)
        self.layout_visual_prim_paths["target"] = self._create_visual_box(
            "/World/pushcube_target",
            np.array(self.pushcube_layout["target_center"], dtype=np.float32),
            np.array(self.pushcube_layout["target_size"], dtype=np.float32),
            (0.12, 0.78, 0.24),
            0.45,
        )
        self.layout_visual_prim_paths["cube_spawn_range"] = self._create_visual_box(
            "/World/cube_spawn_range",
            np.array(self.pushcube_layout["spawn_range_center"], dtype=np.float32),
            np.array(self.pushcube_layout["spawn_range_size"], dtype=np.float32),
            (0.18, 0.48, 0.92),
            0.22,
        )
        self.layout_visual_prim_paths["target_range"] = self._create_visual_box(
            "/World/target_range",
            np.array(self.pushcube_layout["target_range_center"], dtype=np.float32),
            np.array(self.pushcube_layout["target_range_size"], dtype=np.float32),
            (0.45, 0.92, 0.45),
            0.18,
        )
        self._set_visual_visibility(self.layout_visual_prim_paths["target"], visible=True)
        self._set_visual_visibility(
            self.layout_visual_prim_paths["cube_spawn_range"],
            visible=cube_spawn_range_visible,
        )
        self._set_visual_visibility(
            self.layout_visual_prim_paths["target_range"],
            visible=target_range_visible,
        )
        self.layout_visibility_state = {
            "target_visible": True,
            "cube_spawn_range_visible": cube_spawn_range_visible,
            "target_range_visible": target_range_visible,
            "ranges_visible": cube_spawn_range_visible and target_range_visible,
        }

    def _joint_position_debug_map(self, joint_positions: np.ndarray) -> dict[str, float]:
        debug_joint_names = list(dict.fromkeys([*self.training_config.arm_joints, *self.training_config.gripper_joints]))
        return {
            joint_name: round(float(joint_positions[self.dof_names.index(joint_name)]), 6)
            for joint_name in debug_joint_names
            if joint_name in self.dof_names
        }

    def _create_pushcube_visuals_with_pose_guard(self) -> None:
        before_positions = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
        before_gripper_positions = np.array(before_positions[self.gripper_indices], dtype=np.float64)
        print("[pushcube-layout] enabled=True")
        print("[pose-guard] robot qpos before layout =", self._joint_position_debug_map(before_positions))
        self._create_pushcube_visuals()
        after_positions = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
        delta = after_positions - before_positions
        max_abs_change = float(np.max(np.abs(delta))) if delta.size else 0.0
        after_gripper_positions = np.array(after_positions[self.gripper_indices], dtype=np.float64)
        gripper_delta = after_gripper_positions - before_gripper_positions
        gripper_max_abs_change = float(np.max(np.abs(gripper_delta))) if gripper_delta.size else 0.0
        self.layout_gripper_qpos_before = before_gripper_positions
        self.layout_gripper_qpos_after = after_gripper_positions
        self.layout_gripper_max_abs_qpos_change = gripper_max_abs_change
        self.layout_gripper_qpos_restored = False
        print("[pose-guard] robot qpos after layout =", self._joint_position_debug_map(after_positions))
        print(f"[pose-guard] max_abs_robot_qpos_change={max_abs_change:.6f}")
        if gripper_max_abs_change > 1e-6:
            print("[warning] pushcube layout modified gripper qpos; restoring original values")
        if max_abs_change > 1e-6:
            changed_joints = {
                self.dof_names[index]: round(float(delta[index]), 6)
                for index in range(len(self.dof_names))
                if abs(float(delta[index])) > 1e-6
            }
            print("[pose-guard] ERROR layout modified robot pose", changed_joints)
            self.articulation.set_joint_positions(before_positions)
            self.articulation.set_joint_velocities(np.zeros(len(before_positions), dtype=np.float64))
            self.articulation.apply_action(self._ArticulationAction(joint_positions=before_positions))
            self.step_world(steps=1)
            restored_positions = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
            restored_delta = restored_positions - before_positions
            max_abs_change = float(np.max(np.abs(restored_delta))) if restored_delta.size else 0.0
            self.layout_gripper_qpos_after = np.array(restored_positions[self.gripper_indices], dtype=np.float64)
            restored_gripper_delta = self.layout_gripper_qpos_after - before_gripper_positions
            self.layout_gripper_max_abs_qpos_change = (
                float(np.max(np.abs(restored_gripper_delta))) if restored_gripper_delta.size else 0.0
            )
            self.layout_gripper_qpos_restored = True
            print("[pose-guard] robot qpos restored =", self._joint_position_debug_map(restored_positions))
            print(f"[pose-guard] max_abs_robot_qpos_change={max_abs_change:.6f}")
        print(f"[pushcube-layout] robot_pose_modified={max_abs_change > 1e-6}")

    def _prim_has_collision(self, prim) -> bool:
        if prim is None or not prim.IsValid():
            return False
        if prim.HasAPI(self._UsdPhysics.CollisionAPI):
            return True
        for child in prim.GetChildren():
            if self._prim_has_collision(child):
                return True
        return False

    def _iter_descendants(self, prim):
        if prim is None or not prim.IsValid():
            return []
        return self._Usd.PrimRange(prim)

    def _collision_prims_under(self, prim) -> list[object]:
        return [desc for desc in self._iter_descendants(prim) if desc.HasAPI(self._UsdPhysics.CollisionAPI)]

    def _physics_material_prim_path(self, name: str) -> str:
        return f"/World/physics_materials/{name}"

    def _create_or_update_physics_material(
        self,
        *,
        name: str,
        static_friction: float,
        dynamic_friction: float,
        restitution: float,
    ):
        material = self._UsdShade.Material.Define(self.world.stage, self._physics_material_prim_path(name))
        material_api = self._UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr().Set(float(static_friction))
        material_api.CreateDynamicFrictionAttr().Set(float(dynamic_friction))
        material_api.CreateRestitutionAttr().Set(float(restitution))
        self._PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        return material

    def _bind_physics_material(self, prim, material) -> None:
        if prim is None or not prim.IsValid():
            return
        binding_api = self._UsdShade.MaterialBindingAPI.Apply(prim)
        binding_api.Bind(material, self._UsdShade.Tokens.weakerThanDescendants, "physics")

    def _ensure_collision_api(self, prim, *, contact_offset: float, rest_offset: float) -> None:
        if prim is None or not prim.IsValid():
            return
        collision_api = (
            self._UsdPhysics.CollisionAPI(prim)
            if prim.HasAPI(self._UsdPhysics.CollisionAPI)
            else self._UsdPhysics.CollisionAPI.Apply(prim)
        )
        collision_api.CreateCollisionEnabledAttr(True)
        physx_collision_api = (
            self._PhysxSchema.PhysxCollisionAPI(prim)
            if prim.HasAPI(self._PhysxSchema.PhysxCollisionAPI)
            else self._PhysxSchema.PhysxCollisionAPI.Apply(prim)
        )
        physx_collision_api.CreateContactOffsetAttr().Set(float(contact_offset))
        physx_collision_api.CreateRestOffsetAttr().Set(float(rest_offset))

    def _ensure_rigid_body_dynamics(
        self,
        prim,
        *,
        mass: float,
        linear_damping: float,
        angular_damping: float,
        solver_position_iterations: int,
        solver_velocity_iterations: int,
    ) -> None:
        if prim is None or not prim.IsValid():
            return
        mass_api = self._UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(float(mass))
        rigid_body_api = (
            self._UsdPhysics.RigidBodyAPI(prim)
            if prim.HasAPI(self._UsdPhysics.RigidBodyAPI)
            else self._UsdPhysics.RigidBodyAPI.Apply(prim)
        )
        rigid_body_api.CreateRigidBodyEnabledAttr(True)
        physx_rigid_body_api = (
            self._PhysxSchema.PhysxRigidBodyAPI(prim)
            if prim.HasAPI(self._PhysxSchema.PhysxRigidBodyAPI)
            else self._PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        )
        physx_rigid_body_api.CreateLinearDampingAttr().Set(float(linear_damping))
        physx_rigid_body_api.CreateAngularDampingAttr().Set(float(angular_damping))
        physx_rigid_body_api.CreateSolverPositionIterationCountAttr().Set(int(solver_position_iterations))
        physx_rigid_body_api.CreateSolverVelocityIterationCountAttr().Set(int(solver_velocity_iterations))
        physx_rigid_body_api.CreateEnableCCDAttr().Set(True)

    def _set_mesh_collision_approximation(self, prim, approximation: str) -> None:
        if prim is None or not prim.IsValid() or not prim.IsA(self._UsdGeom.Mesh):
            return
        mesh_collision_api = (
            self._UsdPhysics.MeshCollisionAPI(prim)
            if prim.HasAPI(self._UsdPhysics.MeshCollisionAPI)
            else self._UsdPhysics.MeshCollisionAPI.Apply(prim)
        )
        mesh_collision_api.CreateApproximationAttr().Set(str(approximation))

    def _collect_robot_link_collision_prims(self, link_names: list[str]) -> list[object]:
        collision_prims: list[object] = []
        seen_paths: set[str] = set()
        for link_name in link_names:
            link_prim = self._get_prim_at_path(f"{self.robot_prim_path}/{link_name}")
            for collision_prim in self._collision_prims_under(link_prim):
                prim_path = str(collision_prim.GetPath())
                if prim_path in seen_paths:
                    continue
                seen_paths.add(prim_path)
                collision_prims.append(collision_prim)
        return collision_prims

    def _collect_right_gripper_collision_prims(self) -> tuple[list[object], list[object]]:
        gripper_collision_prims = self._collect_robot_link_collision_prims(
            ["right_gripper1_link", "right_gripper2_link"]
        )
        wrist_collision_prims = self._collect_robot_link_collision_prims(
            ["right_wrist_yaw_link", "right_wrist_pitch_link"]
        )
        if not gripper_collision_prims:
            gripper_collision_prims = list(wrist_collision_prims)
        return gripper_collision_prims, wrist_collision_prims

    def _apply_pushcube_physics_defaults(self) -> None:
        cube_material = self._create_or_update_physics_material(
            name="pushcube_cube",
            static_friction=self.scene_config.pushcube_cube_static_friction,
            dynamic_friction=self.scene_config.pushcube_cube_dynamic_friction,
            restitution=self.scene_config.pushcube_cube_restitution,
        )
        table_material = self._create_or_update_physics_material(
            name="pushcube_table",
            static_friction=self.scene_config.pushcube_table_static_friction,
            dynamic_friction=self.scene_config.pushcube_table_dynamic_friction,
            restitution=self.scene_config.pushcube_table_restitution,
        )
        gripper_material = self._create_or_update_physics_material(
            name="pushcube_gripper",
            static_friction=self.scene_config.pushcube_gripper_static_friction,
            dynamic_friction=self.scene_config.pushcube_gripper_dynamic_friction,
            restitution=self.scene_config.pushcube_gripper_restitution,
        )

        self._bind_physics_material(self.brick.prim, cube_material)
        self._ensure_collision_api(
            self.brick.prim,
            contact_offset=self.scene_config.pushcube_cube_contact_offset,
            rest_offset=self.scene_config.pushcube_cube_rest_offset,
        )
        self._ensure_rigid_body_dynamics(
            self.brick.prim,
            mass=self.scene_config.pushcube_cube_mass,
            linear_damping=self.scene_config.pushcube_cube_linear_damping,
            angular_damping=self.scene_config.pushcube_cube_angular_damping,
            solver_position_iterations=self.scene_config.pushcube_cube_solver_position_iterations,
            solver_velocity_iterations=self.scene_config.pushcube_cube_solver_velocity_iterations,
        )

        self._bind_physics_material(self.table.prim, table_material)
        for collision_prim in self._collision_prims_under(self.table.prim):
            self._ensure_collision_api(
                collision_prim,
                contact_offset=self.scene_config.pushcube_table_contact_offset,
                rest_offset=self.scene_config.pushcube_table_rest_offset,
            )

        gripper_collision_prims, wrist_collision_prims = self._collect_right_gripper_collision_prims()
        for collision_prim in gripper_collision_prims:
            self._bind_physics_material(collision_prim, gripper_material)
            self._ensure_collision_api(
                collision_prim,
                contact_offset=self.scene_config.pushcube_gripper_contact_offset,
                rest_offset=self.scene_config.pushcube_gripper_rest_offset,
            )
            approximation = "convexHull" if "gripper" in str(collision_prim.GetPath()) else "boundingCube"
            self._set_mesh_collision_approximation(collision_prim, approximation)
        for collision_prim in wrist_collision_prims:
            self._bind_physics_material(collision_prim, gripper_material)
            self._ensure_collision_api(
                collision_prim,
                contact_offset=self.scene_config.pushcube_gripper_contact_offset,
                rest_offset=self.scene_config.pushcube_gripper_rest_offset,
            )
            self._set_mesh_collision_approximation(collision_prim, "boundingCube")

        self.pushcube_physics_summary = {
            "cube_material_path": str(cube_material.GetPath()),
            "table_material_path": str(table_material.GetPath()),
            "gripper_material_path": str(gripper_material.GetPath()),
            "gripper_collision_prim_paths": [str(prim.GetPath()) for prim in gripper_collision_prims],
            "wrist_collision_prim_paths": [str(prim.GetPath()) for prim in wrist_collision_prims],
        }
        print(f"[cube-physics] mass={self.scene_config.pushcube_cube_mass:.3f}")
        print(f"[cube-physics] static_friction={self.scene_config.pushcube_cube_static_friction:.3f}")
        print(f"[cube-physics] dynamic_friction={self.scene_config.pushcube_cube_dynamic_friction:.3f}")
        print(f"[cube-physics] restitution={self.scene_config.pushcube_cube_restitution:.3f}")
        print(
            "[cube-physics] damping="
            f"{{'linear': {self.scene_config.pushcube_cube_linear_damping:.3f}, "
            f"'angular': {self.scene_config.pushcube_cube_angular_damping:.3f}}}"
        )
        print(f"[table-physics] static={not self.table.prim.HasAPI(self._UsdPhysics.RigidBodyAPI)}")
        print(f"[table-physics] collision={self._prim_has_collision(self.table.prim)}")
        print(
            "[table-physics] friction="
            f"{{'static': {self.scene_config.pushcube_table_static_friction:.3f}, "
            f"'dynamic': {self.scene_config.pushcube_table_dynamic_friction:.3f}, "
            f"'restitution': {self.scene_config.pushcube_table_restitution:.3f}}}"
        )
        print("[gripper-physics] fixed_gripper=True")
        print(
            "[gripper-physics] collision_prims="
            f"{[str(prim.GetPath()) for prim in gripper_collision_prims]}"
        )
        print(
            "[gripper-physics] contact_pad_paths="
            f"{[str(prim.GetPath()) for prim in wrist_collision_prims]}"
        )
        print(f"[gripper-physics] restitution={self.scene_config.pushcube_gripper_restitution:.3f}")
        print(
            "[gripper-physics] friction="
            f"{{'static': {self.scene_config.pushcube_gripper_static_friction:.3f}, "
            f"'dynamic': {self.scene_config.pushcube_gripper_dynamic_friction:.3f}}}"
        )

    def table_center_xy(self) -> np.ndarray:
        return np.array(self.scene_config.table_position[:2], dtype=np.float64)

    def table_size_xy(self) -> np.ndarray:
        return np.array(self.scene_config.table_scale[:2], dtype=np.float64)

    def table_bounds_xy(self, margin: float = 0.0) -> dict[str, float]:
        center = self.table_center_xy()
        half = (self.table_size_xy() / 2.0) - float(margin)
        return {
            "x_min": float(center[0] - half[0]),
            "x_max": float(center[0] + half[0]),
            "y_min": float(center[1] - half[1]),
            "y_max": float(center[1] + half[1]),
        }

    def rect_bounds_xy(self, center_xy: np.ndarray, size_xy: np.ndarray) -> dict[str, float]:
        half = np.array(size_xy, dtype=np.float64) / 2.0
        center = np.array(center_xy, dtype=np.float64)
        return {
            "x_min": float(center[0] - half[0]),
            "x_max": float(center[0] + half[0]),
            "y_min": float(center[1] - half[1]),
            "y_max": float(center[1] + half[1]),
        }

    def rect_margin_to_table_edges(self, center_xy: np.ndarray, size_xy: np.ndarray) -> dict[str, float]:
        table_bounds = self.table_bounds_xy(margin=0.0)
        rect_bounds = self.rect_bounds_xy(center_xy, size_xy)
        margins = {
            "x_min": float(rect_bounds["x_min"] - table_bounds["x_min"]),
            "x_max": float(table_bounds["x_max"] - rect_bounds["x_max"]),
            "y_min": float(rect_bounds["y_min"] - table_bounds["y_min"]),
            "y_max": float(table_bounds["y_max"] - rect_bounds["y_max"]),
        }
        margins["min"] = min(margins.values())
        return margins

    def max_margin_inside_table(self, size_xy: np.ndarray) -> float:
        table_half = self.table_size_xy() / 2.0
        rect_half = np.array(size_xy, dtype=np.float64) / 2.0
        feasible = np.min(table_half - rect_half)
        return max(0.0, float(feasible))

    def place_rect_on_table(
        self,
        center_xy: np.ndarray,
        *,
        size_xy: np.ndarray,
        requested_margin: float,
        fallback_margin: float,
        label: str,
    ) -> dict[str, object]:
        requested_margin = max(0.0, float(requested_margin))
        fallback_margin = max(0.0, float(fallback_margin))
        size_xy = np.array(size_xy, dtype=np.float64)
        max_feasible_margin = self.max_margin_inside_table(size_xy)
        effective_margin = min(requested_margin, max_feasible_margin)
        if requested_margin > 0.0 and max_feasible_margin < requested_margin:
            effective_margin = max_feasible_margin
        if fallback_margin > 0.0 and requested_margin <= 0.0:
            effective_margin = min(fallback_margin, max_feasible_margin)
        center = self.clamp_xy_inside_table(
            center_xy,
            size_xy=size_xy,
            margin=effective_margin,
            label=label,
        )
        clamped = not np.allclose(center, np.array(center_xy, dtype=np.float64))
        return {
            "center_xy": center,
            "requested_margin": requested_margin,
            "fallback_margin": fallback_margin,
            "effective_margin": effective_margin,
            "max_feasible_margin": max_feasible_margin,
            "margin_to_edges": self.rect_margin_to_table_edges(center, size_xy),
            "clamped": clamped,
        }

    def clamp_xy_inside_table(
        self,
        center_xy: np.ndarray,
        *,
        size_xy: np.ndarray,
        margin: float,
        label: str,
    ) -> np.ndarray:
        center = np.array(center_xy, dtype=np.float64)
        half = np.array(size_xy, dtype=np.float64) / 2.0
        bounds = self.table_bounds_xy(margin=0.0)
        x_min = bounds["x_min"] + half[0] + float(margin)
        x_max = bounds["x_max"] - half[0] - float(margin)
        y_min = bounds["y_min"] + half[1] + float(margin)
        y_max = bounds["y_max"] - half[1] - float(margin)
        if x_min > x_max:
            x_min = x_max = float(self.scene_config.table_position[0])
        if y_min > y_max:
            y_min = y_max = float(self.scene_config.table_position[1])
        corrected = np.array(
            [
                np.clip(center[0], x_min, x_max),
                np.clip(center[1], y_min, y_max),
            ],
            dtype=np.float64,
        )
        if not np.allclose(corrected, center):
            print(
                "[warning] pushcube layout clamped inside table",
                {
                    "label": label,
                    "requested_xy": np.round(center, 4).tolist(),
                    "corrected_xy": np.round(corrected, 4).tolist(),
                    "table_bounds": {k: round(v, 4) for k, v in bounds.items()},
                    "margin": round(float(margin), 4),
                },
            )
        return corrected

    def _print_scene_layout_diagnostics(self) -> None:
        table_position, table_quaternion = self.table.get_world_pose()
        brick_position, brick_quaternion = self.brick.get_world_pose()
        table_bounds = self.table_bounds_xy(margin=0.0)
        diagnostics: dict[str, object] = {
            "table_pose": {
                "position": np.round(np.array(table_position, dtype=np.float64), 4).tolist(),
                "quaternion_wxyz": np.round(np.array(table_quaternion, dtype=np.float64), 4).tolist(),
            },
            "table_center": np.round(np.array(self.scene_config.table_position[:2], dtype=np.float64), 4).tolist(),
            "table_size": np.round(np.array(self.scene_config.table_scale[:2], dtype=np.float64), 4).tolist(),
            "table_bounds": {key: round(value, 4) for key, value in table_bounds.items()},
            "table_top_z": round(float(self.scene_config.table_height), 4),
            "table_collision_enabled": self._prim_has_collision(self.table.prim),
            "brick_size": np.round(
                np.array(
                    self.pushcube_layout["cube_scale"] if self.pushcube_layout_enabled else self.scene_config.brick_scale,
                    dtype=np.float64,
                ),
                4,
            ).tolist(),
            "brick_pose": {
                "position": np.round(np.array(brick_position, dtype=np.float64), 4).tolist(),
                "quaternion_wxyz": np.round(np.array(brick_quaternion, dtype=np.float64), 4).tolist(),
            },
            "brick_collision_enabled": self._prim_has_collision(self.brick.prim),
        }
        if self.pushcube_layout_enabled and self.pushcube_layout is not None:
            diagnostics.update(
                {
                    "cube_size": round(float(self.pushcube_layout["cube_size"]), 4),
                    "cube_pose": np.round(np.array(self.pushcube_layout["cube_position"], dtype=np.float64), 4).tolist(),
                    "cube_xy": np.round(
                        np.array(self.pushcube_layout["cube_position"][:2], dtype=np.float64),
                        4,
                    ).tolist(),
                    "expected_cube_z": round(float(self.pushcube_layout["cube_position"][2]), 4),
                    "cube_margin_to_table_edges": {
                        key: round(value, 4)
                        for key, value in dict(self.pushcube_layout["cube_margin_to_table_edges"]).items()
                    },
                    "cube_clamped": bool(self.pushcube_layout["cube_clamped"]),
                    "target_center": np.round(np.array(self.pushcube_layout["target_center"], dtype=np.float64), 4).tolist(),
                    "target_xy": np.round(
                        np.array(self.pushcube_layout["target_center"][:2], dtype=np.float64),
                        4,
                    ).tolist(),
                    "target_size": np.round(np.array(self.pushcube_layout["target_size"], dtype=np.float64), 4).tolist(),
                    "target_z": round(float(self.pushcube_layout["target_center"][2]), 4),
                    "target_margin_to_table_edges": {
                        key: round(value, 4)
                        for key, value in dict(self.pushcube_layout["target_margin_to_table_edges"]).items()
                    },
                    "target_clamped": bool(self.pushcube_layout["target_clamped"]),
                    "cube_requested_margin": round(float(self.pushcube_layout["cube_requested_margin"]), 4),
                    "target_requested_margin": round(float(self.pushcube_layout["target_requested_margin"]), 4),
                    "cube_effective_margin": round(float(self.pushcube_layout["cube_effective_margin"]), 4),
                    "target_effective_margin": round(float(self.pushcube_layout["target_effective_margin"]), 4),
                    "cube_spawn_range": {
                        "center": np.round(
                            np.array(self.pushcube_layout["spawn_range_center"], dtype=np.float64), 4
                        ).tolist(),
                        "bounds": {
                            key: round(value, 4)
                            for key, value in self.rect_bounds_xy(
                                np.array(self.pushcube_layout["spawn_range_center"][:2], dtype=np.float64),
                                np.array(self.pushcube_layout["spawn_range_size"][:2], dtype=np.float64),
                            ).items()
                        },
                        "size": np.round(
                            np.array(self.pushcube_layout["spawn_range_size"], dtype=np.float64), 4
                        ).tolist(),
                    },
                    "target_range": {
                        "center": np.round(
                            np.array(self.pushcube_layout["target_range_center"], dtype=np.float64), 4
                        ).tolist(),
                        "bounds": {
                            key: round(value, 4)
                            for key, value in self.rect_bounds_xy(
                                np.array(self.pushcube_layout["target_range_center"][:2], dtype=np.float64),
                                np.array(self.pushcube_layout["target_range_size"][:2], dtype=np.float64),
                            ).items()
                        },
                        "size": np.round(
                            np.array(self.pushcube_layout["target_range_size"], dtype=np.float64), 4
                        ).tolist(),
                    },
                    "spawn_range_clamped": bool(self.pushcube_layout["spawn_range_clamped"]),
                    "target_range_clamped": bool(self.pushcube_layout["target_range_clamped"]),
                    "target_collision_enabled": self._prim_has_collision(
                        self._get_prim_at_path(self.layout_visual_prim_paths["target"])
                    ),
                    "cube_spawn_range_collision_enabled": self._prim_has_collision(
                        self._get_prim_at_path(self.layout_visual_prim_paths["cube_spawn_range"])
                    )
                    if "cube_spawn_range" in self.layout_visual_prim_paths
                    else False,
                    "target_range_collision_enabled": self._prim_has_collision(
                        self._get_prim_at_path(self.layout_visual_prim_paths["target_range"])
                    )
                    if "target_range" in self.layout_visual_prim_paths
                    else False,
                }
            )
        print("[demo] scene layout", diagnostics)

    def _camera_prim_path(self, spec: MountedCameraSpec) -> str:
        return f"{self.robot_prim_path}/{spec.mount_link}/{spec.camera_name}"

    def _camera_mount_parent_path(self, spec: MountedCameraSpec) -> str:
        return f"{self.robot_prim_path}/{spec.mount_link}"

    def _camera_rotation_matrix(self, rotation_xyz_deg: tuple[float, float, float]) -> np.ndarray:
        rx, ry, rz = [math.radians(value) for value in rotation_xyz_deg]
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
        rot_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
        rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        return rot_z @ rot_y @ rot_x

    def _is_arm_camera(self, spec: MountedCameraSpec) -> bool:
        return spec.camera_name in {"left_arm_camera", "right_arm_camera"}

    def _arm_target_link(self, spec: MountedCameraSpec) -> str:
        if spec.camera_name == "left_arm_camera":
            return "left_wrist_yaw_link"
        return "right_wrist_yaw_link"

    def _look_at_rotation(self, camera_position: np.ndarray, target_position: np.ndarray) -> np.ndarray:
        forward = np.array(target_position - camera_position, dtype=np.float64)
        forward /= max(np.linalg.norm(forward), 1e-9)
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        lateral = np.cross(world_up, forward)
        if np.linalg.norm(lateral) < 1e-6:
            world_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            lateral = np.cross(world_up, forward)
        lateral /= max(np.linalg.norm(lateral), 1e-9)
        vertical = np.cross(forward, lateral)
        vertical /= max(np.linalg.norm(vertical), 1e-9)
        return np.column_stack([forward, lateral, vertical])

    def _camera_local_transform(self, spec: MountedCameraSpec) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        local_translation = np.array(spec.translation_xyz, dtype=np.float64)
        if not self._is_arm_camera(spec):
            return local_translation, self._camera_rotation_matrix(spec.rotation_xyz_deg), None

        mount_link_pose = self.get_link_pose(spec.mount_link)
        target_link_pose = self.get_link_pose(self._arm_target_link(spec))
        camera_world_position = mount_link_pose.position + mount_link_pose.rotation @ local_translation
        approach_direction = np.array(target_link_pose.rotation[:, 0], dtype=np.float64)
        approach_direction /= max(np.linalg.norm(approach_direction), 1e-9)
        look_at_target = target_link_pose.position + (0.12 * approach_direction) + np.array([0.0, 0.0, -0.07], dtype=np.float64)
        world_rotation = self._look_at_rotation(camera_world_position, look_at_target)
        local_rotation = mount_link_pose.rotation.T @ world_rotation
        return local_translation, local_rotation, look_at_target

    def _find_named_camera_prims(self, camera_name: str) -> list[object]:
        matches = []
        stage = self.world.stage
        for prim in stage.Traverse():
            if not prim.IsValid() or not prim.IsA(self._UsdGeom.Camera):
                continue
            path_string = prim.GetPath().pathString
            if path_string.startswith(self.robot_prim_path) and prim.GetName() == camera_name:
                matches.append(prim)
        return matches

    def _inspect_camera_candidates(self) -> None:
        for camera_name, spec in self.camera_mount_specs.items():
            matches = self._find_named_camera_prims(camera_name)
            if not matches:
                print(f"[demo] inspect {camera_name}: no existing camera prims found before mounting")
                continue
            for prim in matches:
                parent_path = prim.GetParent().GetPath().pathString
                expected_parent = self._camera_mount_parent_path(spec)
                status = "OK" if parent_path == expected_parent else "MISMATCH"
                print(
                    "[demo] inspect camera",
                    {
                        "camera_name": camera_name,
                        "prim_path": prim.GetPath().pathString,
                        "parent_path": parent_path,
                        "expected_parent": expected_parent,
                        "status": status,
                    },
                )

    def _ensure_mounted_camera(self, spec: MountedCameraSpec) -> str | None:
        from pxr import Gf

        mount_parent_path = self._camera_mount_parent_path(spec)
        mount_parent = self._get_prim_at_path(mount_parent_path)
        if not mount_parent or not mount_parent.IsValid():
            print(
                f"[warning] {spec.camera_name} could not be mounted because link prim is missing: {mount_parent_path}"
            )
            return None

        camera_path = self._camera_prim_path(spec)
        camera_prim = self._UsdGeom.Camera.Define(self.world.stage, camera_path)
        xformable = self._UsdGeom.Xformable(camera_prim)
        local_translation, local_rotation, _ = self._camera_local_transform(spec)
        local_quaternion = matrix_to_quat_wxyz(local_rotation)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(*local_translation.tolist()))
        xformable.AddOrientOp().Set(
            Gf.Quatf(
                float(local_quaternion[0]),
                float(local_quaternion[1]),
                float(local_quaternion[2]),
                float(local_quaternion[3]),
            )
        )
        camera_prim.CreateProjectionAttr("perspective")
        camera_prim.CreateFocalLengthAttr(18.0)
        camera_prim.CreateHorizontalApertureAttr(20.955)
        camera_prim.CreateVerticalApertureAttr(15.2908)
        camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
        return camera_path

    def _camera_local_pose(self, spec: MountedCameraSpec) -> tuple[Pose, np.ndarray | None]:
        local_translation, local_rotation, look_at_target = self._camera_local_transform(spec)
        return Pose(position=local_translation, rotation=local_rotation), look_at_target

    def _print_camera_mount_report(self, camera_name: str, spec: MountedCameraSpec, camera_path: str | None) -> None:
        if camera_path is None:
            print(f"[warning] {camera_name} prim path unresolved")
            return

        camera_prim = self._get_prim_at_path(camera_path)
        if not camera_prim or not camera_prim.IsValid():
            print(f"[warning] {camera_name} resolved to an invalid prim: {camera_path}")
            return

        parent_path = camera_prim.GetParent().GetPath().pathString
        local_pose, look_at_target = self._camera_local_pose(spec)
        world_pose = self.get_prim_pose(camera_prim)
        world_forward = np.array(world_pose.rotation[:, 0], dtype=np.float64)
        report = {
            "camera_name": camera_name,
            "prim_path": camera_path,
            "parent_path": parent_path,
            "mount_link": spec.mount_link,
            "local_translation": np.round(local_pose.position, 4).tolist(),
            "local_rotation_quaternion_wxyz": np.round(matrix_to_quat_wxyz(local_pose.rotation), 4).tolist(),
            "world_position": np.round(world_pose.position, 4).tolist(),
            "world_quaternion_wxyz": np.round(world_pose.quaternion_wxyz, 4).tolist(),
            "world_forward_direction": np.round(world_forward, 4).tolist(),
        }
        if look_at_target is not None:
            report["look_at_target"] = np.round(look_at_target, 4).tolist()
        print("[demo] mounted camera", report)

        if self._is_arm_camera(spec):
            if abs(float(world_forward[2])) > 0.92:
                print(f"[warning] {camera_name} optical axis is too close to vertical")
            base_prim = self._get_prim_at_path(f"{self.robot_prim_path}/base_link")
            if base_prim and base_prim.IsValid():
                base_pose = self.get_prim_pose(base_prim)
                to_robot_body = base_pose.position - world_pose.position
                to_robot_body /= max(np.linalg.norm(to_robot_body), 1e-9)
                if float(np.dot(world_forward, to_robot_body)) > 0.25:
                    print(f"[warning] {camera_name} optical axis points back toward the robot body")

    def _validate_camera_attachment(self, camera_name: str, spec: MountedCameraSpec, camera_path: str | None) -> None:
        if camera_path is None:
            return
        if spec.validation_joint_name not in self.dof_names:
            print(f"[warning] could not validate {camera_name}; missing joint {spec.validation_joint_name}")
            return

        camera_prim = self._get_prim_at_path(camera_path)
        if not camera_prim or not camera_prim.IsValid():
            print(f"[warning] could not validate {camera_name}; invalid prim {camera_path}")
            return

        joint_index = self.dof_names.index(spec.validation_joint_name)
        original_positions = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
        before_pose = self.get_prim_pose(camera_prim)
        moved_positions = np.array(original_positions, dtype=np.float64)
        moved_positions[joint_index] += 0.08

        self.articulation.set_joint_positions(moved_positions)
        self.articulation.set_joint_velocities(np.zeros(len(self.dof_names), dtype=np.float64))
        self.step_world(steps=4)
        after_pose = self.get_prim_pose(camera_prim)

        self.articulation.set_joint_positions(original_positions)
        self.articulation.set_joint_velocities(np.zeros(len(self.dof_names), dtype=np.float64))
        self.step_world(steps=4)

        translation_delta = float(np.linalg.norm(after_pose.position - before_pose.position))
        rotation_delta = float(np.linalg.norm(after_pose.quaternion_wxyz - before_pose.quaternion_wxyz))
        before_link_pose = self.get_link_pose(spec.mount_link)
        link_offset_before = np.array(before_pose.position - before_link_pose.position, dtype=np.float64)
        if np.linalg.norm(link_offset_before) < 0.01:
            print(f"[warning] {camera_name} may still be inside link geometry")
        print(
            "[demo] camera attachment validation",
            {
                "camera_name": camera_name,
                "joint_name": spec.validation_joint_name,
                "translation_delta": round(translation_delta, 6),
                "rotation_delta": round(rotation_delta, 6),
                "offset_from_link_origin": np.round(link_offset_before, 4).tolist(),
            },
        )
        if translation_delta < 1e-5 and rotation_delta < 1e-5:
            print("Camera is not actually attached to robot link")

    def _setup_camera_viewports(self) -> None:
        if self.headless:
            return

        try:
            from omni.kit.viewport.utility import create_viewport_window, get_active_viewport_window
        except Exception as exc:
            print(f"[warning] viewport utility import failed; skipping onboard camera GUI setup: {exc}")
            return

        main_viewport_window = get_active_viewport_window()
        if main_viewport_window is None:
            print("[warning] main perspective viewport was not found; skipping onboard camera GUI setup")
            return

        viewport_specs = [
            ("Head Camera", "head_camera", 20, 120, 480, 270),
            ("Left Arm Camera", "left_arm_camera", 520, 120, 480, 270),
            ("Right Arm Camera", "right_arm_camera", 1020, 120, 480, 270),
        ]
        self._camera_viewport_windows = []

        for window_name, camera_name, pos_x, pos_y, width, height in viewport_specs:
            camera_path = self.camera_prim_paths.get(camera_name)
            if camera_path is None:
                print(f"[warning] skipping viewport '{window_name}' because {camera_name} is unavailable")
                continue

            viewport_window = create_viewport_window(
                name=window_name,
                width=width,
                height=height,
                position_x=pos_x,
                position_y=pos_y,
                camera_path=camera_path,
            )
            if viewport_window is None:
                print(f"[warning] failed to create viewport '{window_name}' for camera {camera_path}")
                continue

            viewport_window.viewport_api.camera_path = camera_path
            self._camera_viewport_windows.append(viewport_window)
            print(f"[demo] viewport '{window_name}' -> {camera_path}")

    def setup_onboard_cameras(self) -> None:
        self._inspect_camera_candidates()
        self.camera_prim_paths = {}
        for camera_name, spec in self.camera_mount_specs.items():
            camera_path = self._ensure_mounted_camera(spec)
            self.camera_prim_paths[camera_name] = camera_path
            self._print_camera_mount_report(camera_name, spec, camera_path)
            self._validate_camera_attachment(camera_name, spec, camera_path)
        self._setup_camera_viewports()

    def _build_arm_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lower = []
        upper = []
        debug_bounds = {}
        for joint_name in self.training_config.arm_joints:
            limit = self.training_config.joint_limits.get(joint_name, {})
            low = float(limit.get("min_position", -3.14))
            high = float(limit.get("max_position", 3.14))
            lower.append(low)
            upper.append(high)
            debug_bounds[joint_name] = {
                "min": round(low, 4),
                "max": round(high, 4),
                "source": limit.get("limit_source", "urdf"),
            }
        print("[demo] arm joint bounds", debug_bounds)
        return np.array(lower, dtype=np.float64), np.array(upper, dtype=np.float64)

    def get_link_pose(self, link_name: str) -> Pose:
        prim = self._get_prim_at_path(f"{self.robot_prim_path}/{link_name}")
        return self.get_prim_pose(prim)

    def get_prim_pose(self, prim) -> Pose:
        transform = self._UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
        translation = transform.ExtractTranslation()
        rotation = np.array(transform.ExtractRotationMatrix(), dtype=np.float64)
        return Pose(
            position=np.array([float(translation[0]), float(translation[1]), float(translation[2])], dtype=np.float64),
            rotation=rotation,
        )

    def get_base_pose(self) -> Pose:
        return self.get_prim_pose(self.base_prim)

    def get_end_effector_pose(self) -> Pose:
        return self.get_prim_pose(self.ee_prim)

    def get_brick_pose(self) -> Pose:
        position, orientation = self.brick.get_world_pose()
        return Pose(position=np.array(position, dtype=np.float64), rotation=quat_wxyz_to_matrix(np.array(orientation)))

    def current_arm_positions(self) -> np.ndarray:
        positions = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
        return positions[self.arm_indices]

    def current_gripper_positions(self) -> np.ndarray:
        positions = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
        return positions[self.gripper_indices]

    def apply_joint_targets(self, arm_target: np.ndarray, gripper_target: np.ndarray) -> None:
        full_target = np.array(self.articulation.get_joint_positions(), dtype=np.float64)
        full_target[self.arm_indices] = np.clip(np.array(arm_target, dtype=np.float64), self.arm_lower, self.arm_upper)
        full_target[self.gripper_indices] = np.array(gripper_target, dtype=np.float64)
        self.articulation.apply_action(self._ArticulationAction(joint_positions=full_target))

    def set_robot_root_pose(self) -> None:
        root_position = np.array(self.training_config.robot_root_position, dtype=np.float32)
        if hasattr(self.articulation, "set_world_pose"):
            self.articulation.set_world_pose(position=root_position)
            return
        self._UsdGeom.XformCommonAPI(self.robot_prim).SetTranslate(tuple(float(v) for v in root_position))

    def home_arm_positions(self) -> np.ndarray:
        return np.array(
            [
                self.training_config.home_joint_positions.get(joint_name, 0.0)
                for joint_name in self.training_config.arm_joints
            ],
            dtype=np.float64,
        )

    def set_robot_home(self) -> None:
        full_positions = np.zeros(len(self.dof_names), dtype=np.float64)
        for joint_name, value in self.training_config.home_joint_positions.items():
            if joint_name in self.dof_names:
                full_positions[self.dof_names.index(joint_name)] = float(value)
        for joint_name, value in self.training_config.open_gripper_positions.items():
            if joint_name in self.dof_names:
                full_positions[self.dof_names.index(joint_name)] = float(value)
        self.set_robot_root_pose()
        self.articulation.set_joint_positions(full_positions)
        self.articulation.set_joint_velocities(np.zeros(len(self.dof_names), dtype=np.float64))
        self.articulation.apply_action(self._ArticulationAction(joint_positions=full_positions))

    def reset_scene(self) -> None:
        self.world.reset()
        self.articulation.initialize()
        self.set_robot_root_pose()
        self.set_robot_home()
        self.brick.set_world_pose(position=np.array(self.initial_brick_position, dtype=np.float32))
        self.brick.set_linear_velocity(np.zeros(3, dtype=np.float32))
        self.brick.set_angular_velocity(np.zeros(3, dtype=np.float32))
        self.step_world(steps=15)

    def step_world(self, steps: int = 1) -> None:
        for _ in range(max(1, steps)):
            self.world.step(render=not self.headless)

    def sync_kinematics_base_pose(self) -> None:
        if self.kinematics is None:
            raise RuntimeError("Lula is disabled for this scene.")
        base_pose = self.get_base_pose()
        self.kinematics.set_robot_base_pose(base_pose.position, base_pose.quaternion_wxyz)

    def solve_ik(
        self,
        target_pose: Pose,
        warm_start: np.ndarray | None = None,
        extra_seed: np.ndarray | None = None,
        position_tolerance: float = 0.005,
        orientation_tolerance: float = 0.1,
    ) -> tuple[np.ndarray | None, bool]:
        if self.kinematics is None:
            raise RuntimeError("Lula is disabled for this scene.")
        self.sync_kinematics_base_pose()
        if warm_start is None:
            warm_start = self.current_arm_positions()
        seeds = [np.array(warm_start, dtype=np.float64), np.zeros(len(self.training_config.arm_joints), dtype=np.float64)]
        if extra_seed is not None:
            seeds.insert(1, np.array(extra_seed, dtype=np.float64))
        self.kinematics.set_default_cspace_seeds(
            np.array(seeds, dtype=np.float64)
        )
        result, success = self.kinematics.compute_inverse_kinematics(
            self.training_config.end_effector_link,
            target_pose.position,
            target_pose.quaternion_wxyz,
            warm_start=np.array(warm_start, dtype=np.float64),
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )
        if not success:
            return None, False
        return np.array(result, dtype=np.float64), True

    def close(self) -> None:
        return None
