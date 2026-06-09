#!/usr/bin/env python3
"""Draw black-and-white architecture flowcharts for the humanoid project.

The diagrams are intentionally generated with plain matplotlib primitives so
they do not depend on Graphviz or Mermaid rendering.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Literal

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, Rectangle


Theme = Literal["bw", "dark"]


@dataclass(frozen=True)
class Box:
    key: str
    x: float
    y: float
    w: float
    h: float
    text: str

    @property
    def center(self) -> tuple[float, float]:
        return self.x + self.w / 2.0, self.y + self.h / 2.0


def configure_fonts() -> None:
    candidates = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["pdf.fonttype"] = 42
    plt.rcParams["svg.fonttype"] = "none"


class Diagram:
    def __init__(
        self,
        title: str,
        *,
        width: float = 14.0,
        height: float = 8.0,
        theme: Theme = "bw",
    ) -> None:
        self.title = title
        self.width = width
        self.height = height
        self.theme = theme
        self.boxes: dict[str, Box] = {}
        self.groups: list[tuple[float, float, float, float, str]] = []
        self.arrows: list[tuple[str | tuple[float, float], str | tuple[float, float], str]] = []
        self.poly_arrows: list[tuple[list[tuple[float, float]], str]] = []

    @property
    def colors(self) -> dict[str, str]:
        if self.theme == "dark":
            return {
                "bg": "#050505",
                "fg": "#f4f4f4",
                "muted": "#c8c8c8",
                "box_face": "#111111",
                "group_face": "#050505",
                "edge": "#f4f4f4",
                "arrow": "#f4f4f4",
            }
        return {
            "bg": "#ffffff",
            "fg": "#000000",
            "muted": "#303030",
            "box_face": "#ffffff",
            "group_face": "#ffffff",
            "edge": "#000000",
            "arrow": "#000000",
        }

    def box(self, key: str, x: float, y: float, w: float, h: float, text: str) -> None:
        self.boxes[key] = Box(key=key, x=x, y=y, w=w, h=h, text=text)

    def group(self, x: float, y: float, w: float, h: float, text: str) -> None:
        self.groups.append((x, y, w, h, text))

    def arrow(
        self,
        start: str | tuple[float, float],
        end: str | tuple[float, float],
        label: str = "",
    ) -> None:
        self.arrows.append((start, end, label))

    def poly_arrow(self, points: Iterable[tuple[float, float]], label: str = "") -> None:
        self.poly_arrows.append((list(points), label))

    def _point(self, value: str | tuple[float, float]) -> tuple[float, float]:
        if isinstance(value, str):
            return self.boxes[value].center
        return value

    def _edge_points(self, start: str | tuple[float, float], end: str | tuple[float, float]) -> tuple[tuple[float, float], tuple[float, float]]:
        p1 = self._point(start)
        p2 = self._point(end)
        if not isinstance(start, str) or not isinstance(end, str):
            return p1, p2

        b1 = self.boxes[start]
        b2 = self.boxes[end]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        if abs(dx) >= abs(dy):
            x1 = b1.x + b1.w if dx >= 0 else b1.x
            y1 = p1[1]
            x2 = b2.x if dx >= 0 else b2.x + b2.w
            y2 = p2[1]
        else:
            x1 = p1[0]
            y1 = b1.y + b1.h if dy >= 0 else b1.y
            x2 = p2[0]
            y2 = b2.y if dy >= 0 else b2.y + b2.h
        return (x1, y1), (x2, y2)

    def draw(self, output_prefix: Path) -> None:
        colors = self.colors
        fig, ax = plt.subplots(figsize=(self.width, self.height))
        fig.patch.set_facecolor(colors["bg"])
        ax.set_facecolor(colors["bg"])
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.axis("off")

        ax.text(
            50,
            97,
            self.title,
            ha="center",
            va="center",
            fontsize=18,
            fontweight="bold",
            color=colors["fg"],
        )

        for x, y, w, h, text in self.groups:
            rect = Rectangle(
                (x, y),
                w,
                h,
                linewidth=1.4,
                edgecolor=colors["edge"],
                facecolor=colors["group_face"],
                linestyle="--",
            )
            ax.add_patch(rect)
            ax.text(
                x + 1.2,
                y + h + 1.1,
                text,
                ha="left",
                va="bottom",
                fontsize=11,
                fontweight="bold",
                color=colors["fg"],
                bbox=dict(
                    boxstyle="square,pad=0.15",
                    facecolor=colors["bg"],
                    edgecolor="none",
                ),
            )

        for box in self.boxes.values():
            rect = Rectangle(
                (box.x, box.y),
                box.w,
                box.h,
                linewidth=1.6,
                edgecolor=colors["edge"],
                facecolor=colors["box_face"],
            )
            ax.add_patch(rect)
            ax.text(
                box.x + box.w / 2.0,
                box.y + box.h / 2.0,
                box.text,
                ha="center",
                va="center",
                fontsize=9.5,
                linespacing=1.25,
                color=colors["fg"],
            )

        for start, end, label in self.arrows:
            p1, p2 = self._edge_points(start, end)
            arrow = FancyArrowPatch(
                p1,
                p2,
                arrowstyle="-|>",
                mutation_scale=13,
                linewidth=1.4,
                color=colors["arrow"],
                shrinkA=2,
                shrinkB=2,
            )
            ax.add_patch(arrow)
            if label:
                lx = (p1[0] + p2[0]) / 2.0
                ly = (p1[1] + p2[1]) / 2.0 + 1.8
                ax.text(
                    lx,
                    ly,
                    label,
                    ha="center",
                    va="bottom",
                    fontsize=8.5,
                    color=colors["muted"],
                    bbox=dict(
                        boxstyle="square,pad=0.15",
                        facecolor=colors["bg"],
                        edgecolor="none",
                    ),
                )

        for points, label in self.poly_arrows:
            if len(points) < 2:
                continue
            for p1, p2 in zip(points[:-2], points[1:-1]):
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=colors["arrow"], linewidth=1.4)
            arrow = FancyArrowPatch(
                points[-2],
                points[-1],
                arrowstyle="-|>",
                mutation_scale=13,
                linewidth=1.4,
                color=colors["arrow"],
                shrinkA=0,
                shrinkB=0,
            )
            ax.add_patch(arrow)
            if label:
                mid = points[len(points) // 2]
                ax.text(
                    mid[0],
                    mid[1] + 1.5,
                    label,
                    ha="center",
                    va="bottom",
                    fontsize=8.5,
                    color=colors["muted"],
                    bbox=dict(
                        boxstyle="square,pad=0.15",
                        facecolor=colors["bg"],
                        edgecolor="none",
                    ),
                )

        output_prefix.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_prefix.with_suffix(".png"), dpi=300, bbox_inches="tight", facecolor=fig.get_facecolor())
        fig.savefig(output_prefix.with_suffix(".svg"), bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)


def draw_isaac(theme: Theme, output_dir: Path) -> None:
    d = Diagram("Isaac Sim 强化学习架构", width=15.0, height=9.0, theme=theme)

    # groups 完全包住各自的 box
    d.group( 3, 44, 26, 46, "机器人资产层")       # y=44~90, 包住 urdf(78~85) cfg(65~72)
    d.group(34, 44, 28, 46, "Isaac Sim 仿真层")   # y=44~90, 包住 runtime/import/world/env
    d.group(67, 44, 28, 46, "PPO 训练层")         # y=44~90, 包住 vec/ppo/cb
    d.group(10, 10, 78, 22, "输出与评估")

    # Col A
    d.box("urdf", 6,  78, 22, 7, "assets/robot_description\nURDF / mesh")
    d.box("cfg",  6,  65, 22, 7, "rl_train/config.py\n关节 / 限位 / 初始位姿")

    # Col B
    d.box("runtime", 37, 78, 22, 7, "resolved URDF\n重写 mesh 路径")
    d.box("import",  37, 68, 22, 7, "isaac_import.py\nURDF -> Articulation")
    d.box("world",   37, 58, 22, 7, "World\n地面 / 桌面 / 方块")
    d.box("env",     37, 47, 22, 7, "HumanoidBrickPickEnv\nobs / reward / done")

    # Col C
    d.box("vec", 70, 78, 22, 7, "DummyVecEnv\nSubprocVecEnv")
    d.box("ppo", 70, 68, 22, 7, "Stable-Baselines3 PPO\nMlpPolicy")
    d.box("cb",  70, 58, 22, 7, "Callbacks\ncheckpoint / metrics / eval")

    # 输出行
    d.box("logs",  13, 23, 18, 6, "logs/ppo_runs\nCSV / TensorBoard")
    d.box("ckpt",  36, 23, 18, 6, "checkpoints / ppo_*")
    d.box("model", 59, 23, 18, 6, "models\nfinal / latest / best")
    d.box("eval",  36, 13, 18, 6, "eval_policy.py\ndemo_pick_brick.py")

    # 资产层 -> 仿真层
    d.arrow("urdf", "runtime")
    d.arrow("cfg",  "runtime")
    d.arrow("runtime", "import")
    d.arrow("import",  "world")
    d.arrow("world",   "env")

    # 仿真层 <-> 训练层（用 poly_arrow 避免双向箭头重叠）
    d.arrow("env", "vec")
    d.arrow("vec", "ppo")
    # ppo->env 走左侧偏移，env->ppo 走右侧偏移，避免重叠
    d.poly_arrow([(70, 71.5), (64, 71.5), (64, 50.5), (59, 50.5)], "action")
    d.poly_arrow([(59, 53.5), (64, 53.5), (64, 74.5), (70, 74.5)], "obs/reward")

    # 训练层 -> 输出
    d.arrow("ppo", "cb")
    d.arrow("cb",  "logs")
    d.arrow("cb",  "ckpt")
    d.arrow("ppo", "model")
    d.arrow("ckpt",  "eval")
    d.arrow("model", "eval")

    d.draw(output_dir / "01_isaac_sim_rl_architecture")


def draw_src_control(theme: Theme, output_dir: Path) -> None:
    # Layout (top → bottom, left → right):
    #   Row top:    启动配置 (desc, moveitcfg) → launch
    #   Col B mid:  键盘路径 (kbd → cmdr)
    #   Col C mid:  Diffusion路径 (servo_launch, ckpt → dp → safe → servo)
    #   Row bottom: ros2_control层 (cm → jsb / controllers → hw → motor)
    d = Diagram("src 控制部署架构：键盘控制与 Diffusion Servo", width=16.0, height=9.5, theme=theme)

    # ── groups ──────────────────────────────────────────────────────────────
    d.group(3,  74, 30, 16, "控制栈启动配置")
    d.group(37, 60, 22, 30, "键盘 / MoveIt 路径")
    d.group(63, 56, 33, 34, "Diffusion Servo 路径")
    d.group(18, 13, 66, 28, "ros2_control 与硬件执行层")

    # ── 启动配置 ─────────────────────────────────────────────────────────────
    d.box("desc",      6,  82, 12, 6, "robot_description\nURDF / controllers")
    d.box("moveitcfg", 20, 82, 12, 6, "MoveIt config\nSRDF / kinematics")
    d.box("launch",    6,  74, 26, 6, "robot_moveit.launch.xml\n启动控制栈")

    # ── 键盘路径 ─────────────────────────────────────────────────────────────
    d.box("kbd",  40, 80, 16, 6, "robot_keyboard_control\njoint / cartesian")
    d.box("cmdr", 40, 68, 16, 7, "robot_commander\nMoveGroupInterface")

    # ── Diffusion Servo 路径 ─────────────────────────────────────────────────
    d.box("servo_launch", 66, 82, 18, 6, "servo_control.launch.py\nMoveIt Servo")
    d.box("ckpt",         87, 82,  8, 6, "latest.pt")
    d.box("dp",           87, 72,  8, 7, "diffusion\nservo_node")
    d.box("safe",         66, 72, 18, 6, "安全转换\nclip / dry_run / estop")
    d.box("servo",        66, 62, 18, 6, "right_servo\nJointJog 接口")

    # ── ros2_control 层 ──────────────────────────────────────────────────────
    d.box("cm",          22, 32, 18, 6, "controller_manager\nros2_control_node")
    d.box("jsb",         22, 21, 18, 7, "joint_state_broadcaster\n/joint_states\n供键盘与策略订阅")
    d.box("controllers", 46, 28, 18, 7, "JointTrajectory\nControllers")
    d.box("hw",          46, 17, 18, 6, "robot_hardware\nSystemInterface")
    d.box("motor",       70, 17, 12, 6, "CAN-FD\n电机驱动")

    # ── arrows ───────────────────────────────────────────────────────────────
    # 启动配置
    d.arrow("desc",      "launch")
    d.arrow("moveitcfg", "launch")
    # launch → controller_manager (折线向下)
    d.poly_arrow([(19, 74), (19, 50), (31, 38)], "启动")

    # 键盘路径
    d.arrow("kbd",  "cmdr", "关节/夹爪/位姿命令")
    # cmdr → controllers (折线)
    d.poly_arrow([(48, 68), (48, 50), (55, 35)], "规划执行")

    # Diffusion 路径
    d.arrow("servo_launch", "safe")
    d.arrow("ckpt", "dp")
    d.arrow("dp",   "safe")
    d.arrow("safe", "servo")
    # servo → controllers (折线)
    d.poly_arrow([(75, 62), (75, 50), (61, 35)], "JointJog")

    # ros2_control 层
    d.arrow("cm",          "jsb")
    d.arrow("cm",          "controllers")
    d.arrow("controllers", "hw")
    d.arrow("hw",          "motor")

    d.draw(output_dir / "02_src_keyboard_diffusion_servo_architecture")


def draw_real_training_collection(theme: Theme, output_dir: Path) -> None:
    """03a — 示教采集流程：控制栈 → 键盘示教 → 机器人 → 录制节点 → raw data
    
    严格自上而下单列布局，所有箭头走垂直/折线，不走斜线：
    
      [控制栈]
         ↓              ↓
    [键盘示教]      [真实机器人] ──→ [right_wrist_camera]
                        ↓                    ↓
                  [/joint_states]             |
                        ↓                    |
                  [/right_joint_cmd]          |
                        ↓         ←──────────┘
                  [demo_recorder]  ←── [demo_control]
                        ↓
                  [data/imitation_raw]
    """
    d = Diagram("03a  示教采集流程", width=14.0, height=10.0, theme=theme)

    # ── groups ──────────────────────────────────────────────────────────────
    d.group( 2, 12, 94, 80, "示教采集流程")

    # ── 顶部：控制栈（横跨全宽）────────────────────────────────────────────
    #   x=5, y=84, w=55, h=7
    d.box("stack",    5, 84, 55, 7,
          "robot_commander  +  robot_moveit.launch.xml\nuse_simulation:=false")

    # ── 第二行：键盘示教 | 真实机器人 | 相机 ────────────────────────────────
    #   keyboard: x=5,  y=72, w=22, h=7
    #   robot:    x=32, y=72, w=22, h=7
    #   camera:   x=65, y=72, w=28, h=7
    d.box("keyboard", 5,  72, 22, 7, "keyboard_control\n专家示教")
    d.box("robot",    32, 72, 22, 7, "真实机器人\n右臂 / 夹爪")
    d.box("camera",   65, 72, 28, 7, "right_wrist_camera\n图像流")

    # ── 第三行：/joint_states | /right_joint_command ─────────────────────
    #   joints: x=5,  y=59, w=22, h=7
    #   cmds:   x=32, y=59, w=22, h=7
    d.box("joints",   5,  59, 22, 7, "/joint_states")
    d.box("cmds",     32, 59, 22, 7, "/right_joint_command\n/open_right_gripper")

    # ── 第四行：demo_control | demo_recorder_node ────────────────────────
    #   control:  x=5,  y=44, w=22, h=7
    #   recorder: x=32, y=44, w=35, h=7
    d.box("control",  5,  44, 22, 7, "demo_control\nstart / stop")
    d.box("recorder", 32, 44, 35, 7, "demo_recorder_node\n订阅 joint_states / cmds / camera")

    # ── 底部：raw data ───────────────────────────────────────────────────
    #   raw: x=5, y=24, w=62, h=8
    d.box("raw",      5,  24, 62, 8,
          "data/imitation_raw / episode_xxxxxx\n"
          "joint_pos.npy   actions.npy   meta.json   [images/]")

    # ── arrows ───────────────────────────────────────────────────────────────
    # stack → keyboard（左下，折线：stack底部左侧 → keyboard顶部）
    d.poly_arrow([(16, 84), (16, 79)], "")
    # stack → robot（正下，折线：stack底部中间 → robot顶部）
    d.poly_arrow([(43, 84), (43, 79)], "")

    # keyboard → joints（正下）
    d.poly_arrow([(16, 72), (16, 66)], "")
    # robot → joints（折线：robot底部 → joints右侧）
    d.poly_arrow([(43, 72), (43, 68), (27, 68), (27, 66)], "")
    # robot → cmds（正下）
    d.poly_arrow([(43, 72), (43, 66)], "")
    # robot → camera（右侧水平）
    d.poly_arrow([(54, 75.5), (65, 75.5)], "")

    # joints → recorder（正下）
    d.poly_arrow([(16, 59), (16, 51)], "")
    # cmds → recorder（正下）
    d.poly_arrow([(43, 59), (43, 51)], "")
    # camera → recorder（折线：camera底部 → recorder右侧）
    d.poly_arrow([(79, 72), (79, 47.5), (67, 47.5)], "")

    # control → recorder（水平右）
    d.poly_arrow([(27, 47.5), (32, 47.5)], "")

    # recorder → raw（正下）
    d.poly_arrow([(49.5, 44), (49.5, 32)], "")

    d.draw(output_dir / "03a_collection")


def draw_real_training_processing(theme: Theme, output_dir: Path) -> None:
    """03b — 数据处理流程：raw data / rosbag -> ImitationDataset"""
    d = Diagram("03b  数据处理流程", width=13.0, height=8.5, theme=theme)

    d.group( 3, 55, 42, 33, "在线录制路径")
    d.group( 3, 18, 42, 32, "rosbag 离线路径")
    d.group(50, 18, 45, 70, "数据集构建")

    # 在线录制路径
    d.box("raw",       6, 72, 36, 8, "data/imitation_raw / episode_xxxxxx\njoint_pos.npy  actions.npy  meta.json")
    d.box("raw_info",  6, 60, 36, 7, "采样率对齐 / 时间戳校验\n剔除失败 episode")

    # rosbag 离线路径
    d.box("rosbag",    6, 42, 36, 7, "data/rosbag / rosbag2_*\n原始 ROS2 bag")
    d.box("process",   6, 31, 36, 7, "process_rosbag_data.py\n提取 controller state")
    d.box("processed", 6, 20, 36, 8, "data/processed/real_robot / training_episodes\nstate 12维  action 6维  timestamps")

    # 数据集构建（右列，全部在 y=20~88 范围内）
    d.box("stats",    53, 72, 38, 7, "compute_stats.py\nmean / std  per-dim")
    d.box("dataset",  53, 58, 38, 9, "ImitationDataset\n窗口切片  obs_horizon / pred_horizon\nnormalize -> [-1, 1]")
    d.box("loader",   53, 44, 38, 7, "DataLoader\nbatch_size / shuffle / pin_memory")
    d.box("split",    53, 30, 38, 7, "train / val split\n保存 stats.json")

    # 左列内部
    d.arrow("raw",       "raw_info")
    d.arrow("rosbag",    "process")
    d.arrow("process",   "processed")

    # 两路 -> stats（都走水平方向，右边出发）
    d.poly_arrow([(42, 63.5), (53, 75.5)], "")   # raw_info 右边中点 -> stats 左边中点
    d.poly_arrow([(42, 24),   (53, 75.5)], "")   # processed 右边中点 -> stats 左边（折线）

    # 右列内部
    d.arrow("stats",   "dataset")
    d.arrow("dataset", "loader")
    d.arrow("loader",  "split")

    d.draw(output_dir / "03b_data_processing")


def draw_real_training_train(theme: Theme, output_dir: Path) -> None:
    """03c — 训练与部署流程：DataLoader → DiffusionPolicy → 评估 / 部署"""
    # Layout (top → bottom, two output branches at bottom):
    #
    #   [DataLoader]
    #        ↓
    #   [DiffusionPolicy  MLP + noise_pred_net]
    #        ↓
    #   [DDPM 训练  MSE loss + lr_scheduler]
    #        ↓
    #   [latest.pt  config + stats]
    #      ↓           ↓
    # [离线评估]   [servo_node 真机部署]
    #
    d = Diagram("03c  训练与部署流程", width=13.0, height=9.5, theme=theme)

    # ── groups ──────────────────────────────────────────────────────────────
    d.group( 3, 52, 92, 38, "Diffusion Policy 训练")
    d.group( 3, 13, 42, 30, "离线评估")
    d.group(52, 13, 43, 30, "真机部署")

    # ── 训练流程 ─────────────────────────────────────────────────────────────
    d.box("loader",  6, 82, 86, 6, "DataLoader  (train split  batch_size=64  shuffle=True)")
    d.box("model",   6, 72, 86, 7, "DiffusionPolicy\nMLP backbone  +  noise_pred_net  +  timestep embedding")
    d.box("ddpm",    6, 62, 86, 7, "DDPM 训练循环\nforward diffusion → denoise → MSE loss → AdamW + cosine lr")
    d.box("ckpt",    6, 53, 86, 6, "latest.pt  /  best_val.pt        (config + stats 一并保存)")

    # ── 离线评估 ─────────────────────────────────────────────────────────────
    d.box("offline_eval", 6,  37, 38, 7, "eval_diffusion.py\n滚动推理  T=100 步去噪")
    d.box("metrics",      6,  27, 38, 7, "误差指标\nMAE / RMSE  per-joint")
    d.box("plots",        6,  17, 38, 7, "可视化输出\npred vs true  /  error dist")

    # ── 真机部署 ─────────────────────────────────────────────────────────────
    d.box("servo",   55, 37, 37, 7, "servo_node\n加载 latest.pt + stats")
    d.box("safe",    55, 27, 37, 7, "安全层\nclip / dry_run / e-stop")
    d.box("robot",   55, 17, 37, 7, "真实机器人\nJointJog → right_servo")

    # ── arrows ───────────────────────────────────────────────────────────────
    d.arrow("loader", "model")
    d.arrow("model",  "ddpm")
    d.arrow("ddpm",   "ckpt")

    # ckpt → 两个分支
    d.arrow("ckpt", "offline_eval")
    d.arrow("ckpt", "servo")

    # 离线评估链
    d.arrow("offline_eval", "metrics")
    d.arrow("metrics",      "plots")

    # 部署链
    d.arrow("servo", "safe")
    d.arrow("safe",  "robot")

    d.draw(output_dir / "03c_training_deployment")


def draw_pushcube(theme: Theme, output_dir: Path) -> None:
    # Layout: 2 parallel columns (2D baseline | Isaac 3D), shared abstraction row at bottom
    # Each column: env/scene → collect/record → dataset → train → eval  (top→bottom)
    d = Diagram("PushCube 架构", width=16.0, height=9.5, theme=theme)

    # ── groups ──────────────────────────────────────────────────────────────
    d.group( 3, 38, 44, 52, "PushCube2D baseline")
    d.group(52, 38, 44, 52, "Isaac PushCube v0")
    d.group(10, 10, 79, 22, "共同抽象")

    # ── Col A: 2D baseline ───────────────────────────────────────────────────
    d.box("env2d",    6,  80, 19, 6, "env.py\n7维 obs / 2维 action")
    d.box("expert2d", 28, 80, 16, 6, "scripted_policy.py\n几何专家")
    d.box("collect2d", 6, 70, 19, 7, "collect_data\nkeyboard_collect")
    d.box("npz",      28, 70, 16, 7, "*.npz 数据集\nobs / actions / success")
    d.box("bc2d",      6, 58, 19, 7, "train_bc.py\nNumPy MLP")
    d.box("eval2d",   28, 58, 16, 7, "eval / visualize\nsuccess / distance")

    # ── Col B: Isaac 3D ──────────────────────────────────────────────────────
    d.box("scene",   55, 80, 19, 6, "Isaac 场景\n桌面 / 方块 / 目标区")
    d.box("record",  77, 80, 16, 6, "record_manual\n人工示教")
    d.box("hdf5",    55, 70, 19, 7, "HDF5 数据\nobs_dim 13 / action_dim 2")
    d.box("inspect", 77, 70, 16, 7, "inspect / plot\nfilter / augment")
    d.box("bc3d",    55, 58, 19, 7, "train_pushcube_bc\nPyTorch MLP")
    d.box("eval3d",  77, 58, 16, 7, "eval_policy\n闭环评估 / 视频")

    # ── Row bottom: 共同抽象 ─────────────────────────────────────────────────
    d.box("obs",    13, 24, 18, 7, "低维状态\nEE / cube / target\nrelative vector")
    d.box("action", 40, 24, 18, 7, "平面动作\ndx / dy")
    d.box("metric", 67, 24, 18, 7, "指标\nsuccess rate\nfinal distance")
    d.box("future", 40, 13, 18, 6, "后续扩展\nDiffusion Policy")

    # ── arrows: Col A ────────────────────────────────────────────────────────
    d.arrow("env2d",    "expert2d")
    d.arrow("expert2d", "collect2d")
    d.arrow("collect2d","npz")
    d.arrow("npz",      "bc2d")
    d.arrow("bc2d",     "eval2d")

    # ── arrows: Col B ────────────────────────────────────────────────────────
    d.arrow("scene",   "record")
    d.arrow("record",  "hdf5")
    d.arrow("hdf5",    "inspect")
    d.arrow("inspect", "bc3d")
    d.arrow("bc3d",    "eval3d")

    # ── arrows: 共同抽象 ─────────────────────────────────────────────────────
    d.arrow("obs",    "action")
    d.arrow("action", "metric")
    d.arrow("action", "future")

    # ── arrows: 两列 → 共同抽象（全部用折线，不走斜线）────────────────────
    # bc2d(x=6~25,y=58~65) -> obs(x=13~31,y=24~31): 先下后右
    d.poly_arrow([(15, 58), (15, 44), (15, 38), (22, 38), (22, 31)], "")
    # eval2d(x=28~44,y=58~65) -> metric(x=67~85,y=24~31): 先下后右
    d.poly_arrow([(36, 58), (36, 44), (76, 44), (76, 31)], "")
    # bc3d(x=55~74,y=58~65) -> obs(x=13~31,y=24~31): 先下后左
    d.poly_arrow([(60, 58), (60, 44), (22, 44), (22, 31)], "")
    # eval3d(x=77~93,y=58~65) -> metric(x=67~85,y=24~31): 先下后左
    d.poly_arrow([(85, 58), (85, 44), (85, 38), (76, 38), (76, 31)], "")

    d.draw(output_dir / "04_pushcube_architecture")


def draw_net_diffusion_policy(theme: Theme, output_dir: Path) -> None:
    """net_01 — DiffusionPolicy 网络结构"""
    d = Diagram("net_01  DiffusionPolicy 网络结构", width=15.0, height=10.5, theme=theme)

    d.group( 2, 72, 94, 20, "输入层（三路）")
    d.group(28, 30, 42, 38, "MLP  noise_pred_net")
    d.group( 2, 10, 94, 16, "输出")

    d.box("noisy",  5,  80, 26, 8,
          "noisy_action\n[B, pred_horizon=16, action_dim=6]\nflatten -> [B, 96]")
    d.box("cond",  38,  80, 22, 8,
          "cond  (robot_state)\n[B, obs_horizon=2, state_dim=12]\nflatten -> [B, 24]")
    d.box("temb",  65,  80, 28, 8,
          "timestep  t  [B]\n-> SinusoidalEmbedding(dim=64)\n-> [B, 64]")

    d.box("cat",   30,  66, 38, 7,
          "torch.cat([x, c, te], dim=1)\n[B, 96 + 24 + 64 = 184]")

    d.box("fc1",   30,  56, 38, 7, "Linear(184 -> 256)  +  ReLU")
    d.box("fc2",   30,  46, 38, 7, "Linear(256 -> 256)  +  ReLU")
    d.box("fc3",   30,  36, 38, 7, "Linear(256 -> 96)")

    d.box("out",   20,  14, 58, 8,
          "reshape -> [B, pred_horizon=16, action_dim=6]\npred_noise\n训练时：MSE(pred_noise, true_noise)")

    d.box("sin_detail", 68, 52, 26, 20,
          "SinusoidalEmbedding\n─────────────────\nhalf_dim = 32\nfreqs = exp(-k * log10000/31)\nargs = t * freqs\nemb = [sin(args), cos(args)]\n-> [B, 64]")

    d.poly_arrow([(18,  80), (18,  73), (49, 73)], "")
    d.poly_arrow([(49,  80), (49,  73)], "")
    d.poly_arrow([(79,  80), (79,  73), (68, 73)], "")

    d.arrow("cat", "fc1")
    d.arrow("fc1", "fc2")
    d.arrow("fc2", "fc3")

    d.poly_arrow([(49, 36), (49, 22)], "")
    d.poly_arrow([(81, 80), (81, 72)], "")

    d.draw(output_dir / "net_01_diffusion_policy_network")


def draw_net_ddpm_training(theme: Theme, output_dir: Path) -> None:
    """net_02 — DDPM 训练与推理流程"""
    d = Diagram("net_02  DDPM 训练 & 推理流程", width=15.0, height=10.0, theme=theme)

    d.group( 2, 52, 94, 38, "训练阶段  (Forward Diffusion + Denoising)")
    d.group( 2, 10, 94, 36, "推理阶段  (Reverse Diffusion  T=100 步)")

    # 训练阶段
    d.box("a0",      5,  78, 20, 8,
          "clean action  a0\n[B, 16, 6]")
    d.box("noise",  30,  78, 20, 8,
          "eps ~ N(0, I)\n高斯噪声\n[B, 16, 6]")
    d.box("sched",  55,  78, 20, 8,
          "noise schedule\nbeta_1...beta_T  线性\n[1e-4 -> 0.02]")
    d.box("at",      5,  63, 20, 8,
          "noisy action  at\n= sqrt(a_bar_t)*a0\n  + sqrt(1-a_bar_t)*eps")
    d.box("net_tr", 30,  63, 45, 8,
          "DiffusionPolicy(at, cond, t)  ->  eps_pred\n[B, 16, 6]")
    d.box("loss",   30,  53, 45, 7,
          "Loss = MSE(eps_pred, eps)\n-> AdamW  lr=1e-4  epochs=50")

    # 推理阶段
    d.box("aT",      5,  36, 20, 8,
          "aT ~ N(0, I)\n纯噪声初始化\n[1, 16, 6]")
    d.box("loop",   30,  36, 45, 8,
          "for t = T-1 ... 0:\n  eps_pred = DiffusionPolicy(at, cond, t)\n  a(t-1) = DDPM reverse step")
    d.box("a0_out",  5,  22, 20, 8,
          "a0  预测动作\n[1, 16, 6]")
    d.box("denorm", 30,  22, 45, 8,
          "反归一化\na = a0 * action_std + action_mean\n-> 关节角度序列  [16, 6]")
    d.box("exec",   30,  12, 45, 7,
          "取前 N 步执行  ->  servo_node  ->  真实机器人")

    d.arrow("a0",    "at")
    d.arrow("noise", "at")
    d.arrow("sched", "at")
    d.arrow("at",    "net_tr")
    d.arrow("noise", "loss")
    d.arrow("net_tr","loss")

    d.arrow("aT",    "loop")
    d.arrow("loop",  "a0_out")
    d.arrow("a0_out","denorm")
    d.arrow("denorm","exec")

    d.draw(output_dir / "net_02_ddpm_training_inference")


def draw_net_ppo(theme: Theme, output_dir: Path) -> None:
    """net_03 — Isaac Sim PPO MlpPolicy 网络结构"""
    d = Diagram("net_03  Isaac Sim PPO  MlpPolicy 网络结构", width=14.0, height=9.5, theme=theme)

    # ── groups ──────────────────────────────────────────────────────────────
    d.group( 2, 60, 94, 28, "共享特征提取（Stable-Baselines3 MlpPolicy）")
    d.group( 2, 28, 44, 28, "策略网络  π(a|s)")
    d.group(52, 28, 44, 28, "价值网络  V(s)")
    d.group( 2, 10, 94, 14, "PPO 超参数")

    # ── 输入 ─────────────────────────────────────────────────────────────────
    d.box("obs",    35, 82, 28, 7,
          "观测  obs\n[关节位置 / 速度 / 目标位置 / …]")

    # ── 共享层（SB3 默认无共享层，直接分叉）────────────────────────────────
    d.box("split",  35, 72, 28, 6, "特征分叉（无共享层）")

    # ── 策略网络 ─────────────────────────────────────────────────────────────
    d.box("pi1",     5, 52, 38, 7, "Linear(obs_dim -> 256)  +  ReLU")
    d.box("pi2",     5, 42, 38, 7, "Linear(256 -> 256)  +  ReLU")
    d.box("pi_out",  5, 32, 38, 7, "Linear(256 -> action_dim)\n-> 动作均值  mu")

    # ── 价值网络 ─────────────────────────────────────────────────────────────
    d.box("vf1",    55, 52, 38, 7, "Linear(obs_dim -> 256)  +  ReLU")
    d.box("vf2",    55, 42, 38, 7, "Linear(256 -> 256)  +  ReLU")
    d.box("vf_out", 55, 32, 38, 7, "Linear(256 -> 1)\n-> 状态价值  V(s)")

    # ── 超参数 ───────────────────────────────────────────────────────────────
    d.box("hparam",  5, 12, 88, 9,
          "lr=1e-4   n_steps=4096   batch=512   n_epochs=8\ngamma=0.99   gae_lambda=0.95   clip=0.15   ent_coef=0.005")

    # ── arrows ───────────────────────────────────────────────────────────────
    d.arrow("obs",   "split")
    # split → 两侧
    d.poly_arrow([(49, 72), (49, 68), (24, 68), (24, 59)], "")
    d.poly_arrow([(49, 72), (49, 68), (74, 68), (74, 59)], "")
    d.arrow("pi1",   "pi2")
    d.arrow("pi2",   "pi_out")
    d.arrow("vf1",   "vf2")
    d.arrow("vf2",   "vf_out")

    d.draw(output_dir / "net_03_ppo_mlppolicy")


def draw_net_bc_mlp(theme: Theme, output_dir: Path) -> None:
    """net_04 — BC MLP 网络结构（PushCube 2D / Isaac 3D / Robot Imitation）"""
    d = Diagram("net_04  BC MLP 网络结构对比", width=15.0, height=10.0, theme=theme)

    # ── groups ──────────────────────────────────────────────────────────────
    d.group( 2, 52, 28, 38, "PushCube 2D  BC")
    d.group(34, 52, 28, 38, "Isaac PushCube  BC")
    d.group(66, 52, 30, 38, "Robot Imitation  BC")
    d.group( 2, 10, 94, 36, "共同设计原则")

    # ── PushCube 2D ──────────────────────────────────────────────────────────
    d.box("2d_in",   4,  82, 24, 6, "obs  [7]")
    d.box("2d_h1",   4,  73, 24, 6, "Linear(7→128) + SiLU")
    d.box("2d_h2",   4,  64, 24, 6, "Linear(128→128) + SiLU")
    d.box("2d_h3",   4,  55, 24, 6, "Linear(128→128) + SiLU")
    d.box("2d_out",  4,  46, 24, 6, "Linear(128→2)\naction  [dx, dy]")

    # ── Isaac PushCube ────────────────────────────────────────────────────────
    d.box("3d_in",  36,  82, 24, 6, "obs  [13]")
    d.box("3d_h1",  36,  73, 24, 6, "Linear(13→256) + ReLU")
    d.box("3d_h2",  36,  64, 24, 6, "Linear(256→256) + ReLU")
    d.box("3d_h3",  36,  55, 24, 6, "Linear(256→128) + ReLU")
    d.box("3d_out", 36,  46, 24, 6, "Linear(128→2)\naction  [dx, dy]")

    # ── Robot Imitation ───────────────────────────────────────────────────────
    d.box("ri_in",  68,  82, 26, 6, "obs_state  [input_dim]")
    d.box("ri_h1",  68,  73, 26, 6, "Linear(in→256) + ReLU")
    d.box("ri_h2",  68,  64, 26, 6, "Linear(256→256) + ReLU")
    d.box("ri_h3",  68,  55, 26, 6, "Linear(256→256) + ReLU")
    d.box("ri_out", 68,  46, 26, 6, "Linear(256→action_dim)\naction  [关节角度]")

    # ── 共同设计原则 ──────────────────────────────────────────────────────────
    d.box("common_loss", 5,  34, 40, 7,
          "损失函数：MSELoss\n优化器：Adam / AdamW\n学习率调度：Cosine Annealing")
    d.box("common_norm", 50, 34, 44, 7,
          "输入归一化：(x - mean) / std\n输出：直接预测动作（非噪声）\n与 DiffusionPolicy 的区别：单步预测")
    d.box("common_use",  5,  22, 89, 8,
          "用途：快速 baseline 验证 / 数据质量检查\n"
          "局限：单步预测，无时序建模，动作抖动较大\n"
          "生产部署使用 DiffusionPolicy（多步去噪，平滑轨迹）")

    # ── arrows: 三列各自向下 ──────────────────────────────────────────────────
    for prefix, cx in [("2d", 16), ("3d", 48), ("ri", 81)]:
        d.poly_arrow([(cx, 82), (cx, 79)], "")
        d.poly_arrow([(cx, 73), (cx, 70)], "")
        d.poly_arrow([(cx, 64), (cx, 61)], "")
        d.poly_arrow([(cx, 55), (cx, 52)], "")

    d.draw(output_dir / "net_04_bc_mlp_comparison")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("docs") / "figures" / "architecture",
        help="Directory for generated PNG/SVG diagrams.",
    )
    parser.add_argument(
        "--theme",
        choices=["bw", "dark"],
        default="bw",
        help="bw: white background with black lines; dark: black background.",
    )
    args = parser.parse_args()

    configure_fonts()
    draw_isaac(args.theme, args.output_dir)
    draw_src_control(args.theme, args.output_dir)
    draw_real_training_collection(args.theme, args.output_dir)
    draw_real_training_processing(args.theme, args.output_dir)
    draw_real_training_train(args.theme, args.output_dir)
    draw_pushcube(args.theme, args.output_dir)
    draw_net_diffusion_policy(args.theme, args.output_dir)
    draw_net_ddpm_training(args.theme, args.output_dir)
    draw_net_ppo(args.theme, args.output_dir)
    draw_net_bc_mlp(args.theme, args.output_dir)
    print(f"Generated diagrams in: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
