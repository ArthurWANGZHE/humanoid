
#!/usr/bin/env python3
"""
论文程序流程图 - 四张
  flow_01  DDPM 推理流程
  flow_02  模仿学习完整 Pipeline
  flow_03  真机部署控制循环
  flow_04  数据采集与处理流程
"""
from __future__ import annotations
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["pdf.fonttype"] = 42

OUT = Path("figure/flowcharts")
OUT.mkdir(parents=True, exist_ok=True)

# ── 颜色：纯黑白，无任何填充 ─────────────────────────────────────────────────
# 所有节点：白底 + 黑边 + 黑字
# 形状本身区分节点类型（椭圆/矩形/菱形/平行四边形）
BK = "#000000"   # 黑
WH = "#FFFFFF"   # 白
C = {
    "start":    (WH, BK),
    "process":  (WH, BK),
    "decision": (WH, BK),
    "io":       (WH, BK),
    "data":     (WH, BK),
    "arrow":    BK,
    "loop":     (WH, BK),
}


# ─────────────────────────────────────────────────────────────────────────────
# 基础绘图工具
# ─────────────────────────────────────────────────────────────────────────────
class FC:
    """Flowchart canvas helper."""

    def __init__(self, title: str, w: float = 10, h: float = 16):
        self.fig, self.ax = plt.subplots(figsize=(w, h))
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.axis("off")
        self.fig.patch.set_facecolor("white")
        self.ax.set_facecolor("white")
        self.ax.text(50, 97.5, title, ha="center", va="center",
                     fontsize=15, fontweight="bold", color="#000000")

    # ── 形状 ─────────────────────────────────────────────────────────────────
    def rect(self, cx, cy, w, h, text, style="process", fs=9.5):
        r = FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                           boxstyle="round,pad=0.4",
                           facecolor=WH, edgecolor=BK,
                           linewidth=1.6, zorder=3)
        self.ax.add_patch(r)
        self.ax.text(cx, cy, text, ha="center", va="center",
                     fontsize=fs, color=BK,
                     linespacing=1.3, zorder=4)
        return cx, cy, w, h

    def diamond(self, cx, cy, w, h, text, fs=9):
        xs = [cx, cx + w/2, cx, cx - w/2, cx]
        ys = [cy + h/2, cy, cy - h/2, cy, cy + h/2]
        self.ax.fill(xs, ys, color=WH, zorder=3)
        self.ax.plot(xs, ys, color=BK, linewidth=1.6, zorder=4)
        self.ax.text(cx, cy, text, ha="center", va="center",
                     fontsize=fs, color=BK,
                     linespacing=1.3, zorder=5)
        return cx, cy, w, h

    def oval(self, cx, cy, w, h, text, style="start", fs=10):
        ell = mpatches.Ellipse((cx, cy), w, h,
                               facecolor=WH, edgecolor=BK,
                               linewidth=2.0, zorder=3)
        self.ax.add_patch(ell)
        self.ax.text(cx, cy, text, ha="center", va="center",
                     fontsize=fs, color=BK, zorder=4)

    def parallelogram(self, cx, cy, w, h, text, style="io", fs=9):
        skew = 2.5
        xs = [cx - w/2 + skew, cx + w/2 + skew,
              cx + w/2 - skew, cx - w/2 - skew]
        ys = [cy + h/2, cy + h/2, cy - h/2, cy - h/2]
        self.ax.fill(xs, ys, color=WH, zorder=3)
        self.ax.plot(xs + [xs[0]], ys + [ys[0]],
                     color=BK, linewidth=1.6, zorder=4)
        self.ax.text(cx, cy, text, ha="center", va="center",
                     fontsize=fs, color=BK,
                     linespacing=1.3, zorder=5)

    # ── 箭头 ─────────────────────────────────────────────────────────────────
    def arrow(self, x1, y1, x2, y2, label="", lside="right"):
        arr = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle="-|>", mutation_scale=14,
            linewidth=1.5, color=C["arrow"],
            shrinkA=2, shrinkB=2, zorder=2,
        )
        self.ax.add_patch(arr)
        if label:
            mx = (x1 + x2) / 2
            my = (y1 + y2) / 2
            dx = 1.5 if lside == "right" else -1.5
            self.ax.text(mx + dx, my, label, ha="center", va="center",
                         fontsize=8, color=BK,
                         bbox=dict(boxstyle="square,pad=0.1",
                                   facecolor=WH, edgecolor="none"))

    def polyarrow(self, pts, label="", lside="right"):
        for (x1, y1), (x2, y2) in zip(pts[:-2], pts[1:-1]):
            self.ax.plot([x1, x2], [y1, y2],
                         color=C["arrow"], linewidth=1.5, zorder=2)
        self.arrow(pts[-2][0], pts[-2][1], pts[-1][0], pts[-1][1],
                   label=label, lside=lside)

    def save(self, name: str):
        p = OUT / name
        self.fig.savefig(p.with_suffix(".png"), dpi=200,
                         bbox_inches="tight", facecolor="white")
        self.fig.savefig(p.with_suffix(".svg"),
                         bbox_inches="tight", facecolor="white")
        plt.close(self.fig)
        print(f"  saved: {p}.png")


# ─────────────────────────────────────────────────────────────────────────────
# flow_01  DDPM 推理流程
# ─────────────────────────────────────────────────────────────────────────────
def flow_ddpm_inference():
    """
    开始
      -> 加载模型 & 参数
      -> 获取当前观测 obs_seq [2, 12]
      -> 归一化 obs_seq
      -> 初始化 a_T ~ N(0,I)  [1,16,6]
      -> 循环 t = T-1 ... 0
           -> eps_pred = DiffusionPolicy(a_t, cond, t)
           -> a_{t-1} = DDPM reverse step
           -> t -= 1
      -> [t < 0?] 是 -> 取 a_0[:exec_steps]
      -> 反归一化动作
      -> 安全检查 (clip / e-stop)
      -> [通过?] 否 -> 丢弃, 等待下次
                 是 -> 发布 JointJog
      -> 等待下一控制周期
      -> 回到"获取当前观测"
    """
    fc = FC("flow_01  DDPM 推理流程（推理阶段）", w=9, h=18)

    # 纵坐标从上到下
    Y = [95, 89, 83, 77, 71, 65,   # 开始~初始化
         59, 53, 47,                # 循环体
         41, 35, 29, 23, 17, 11]   # 后处理~结束

    cx = 50

    # 开始
    fc.oval(cx, Y[0], 28, 5, "开始  (收到控制触发信号)")

    fc.arrow(cx, Y[0]-2.5, cx, Y[1]+2.5)
    fc.rect(cx, Y[1], 52, 5, "加载 DiffusionPolicy 模型\n读取 config / stats (mean, std)", "process")

    fc.arrow(cx, Y[1]-2.5, cx, Y[2]+2.5)
    fc.parallelogram(cx, Y[2], 52, 5,
                     "输入: /joint_states\n-> obs_seq  [obs_horizon=2, state_dim=12]", "io")

    fc.arrow(cx, Y[2]-2.5, cx, Y[3]+2.5)
    fc.rect(cx, Y[3], 52, 5,
            "归一化观测\ncond = (obs_seq - mean) / std", "process")

    fc.arrow(cx, Y[3]-2.5, cx, Y[4]+2.5)
    fc.rect(cx, Y[4], 52, 5,
            "初始化噪声动作\na_T ~ N(0, I)   shape [1, 16, 6]", "data")

    # 循环框
    fc.arrow(cx, Y[4]-2.5, cx, Y[5]+2.5)
    fc.rect(cx, Y[5], 52, 5,
            "设置扩散步  t = T-1 = 99", "loop")

    # 循环体
    fc.arrow(cx, Y[5]-2.5, cx, Y[6]+2.5)
    fc.rect(cx, Y[6], 52, 5,
            "eps_pred = DiffusionPolicy(a_t, cond, t)\n预测当前步噪声  [1, 16, 6]", "process")

    fc.arrow(cx, Y[6]-2.5, cx, Y[7]+2.5)
    fc.rect(cx, Y[7], 52, 5,
            "DDPM reverse step\na_{t-1} = (a_t - beta_t * eps_pred) / sqrt(alpha_t)\n+ sigma_t * z", "process")

    fc.arrow(cx, Y[7]-2.5, cx, Y[8]+2.5)
    fc.diamond(cx, Y[8], 30, 7, "t > 0 ?")

    # 是 -> 回到循环顶部 (左侧折线)
    fc.ax.text(cx - 17, Y[8], "是  t -= 1", ha="center", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-15, Y[8]+3.5), (cx-15, Y[6]+2.5), (cx-26, Y[6]+2.5)],
                 lside="left")
    # 否 -> 向下
    fc.ax.text(cx + 4, Y[8]-4.5, "否", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.arrow(cx, Y[8]-3.5, cx, Y[9]+2.5)

    # 后处理
    fc.rect(cx, Y[9], 52, 5,
            "取前 exec_steps 步动作\na_exec = a_0[:exec_steps]  [exec_steps, 6]", "process")

    fc.arrow(cx, Y[9]-2.5, cx, Y[10]+2.5)
    fc.rect(cx, Y[10], 52, 5,
            "反归一化\naction = a_exec * action_std + action_mean", "process")

    fc.arrow(cx, Y[10]-2.5, cx, Y[11]+2.5)
    fc.diamond(cx, Y[11], 36, 7, "安全检查\nclip 关节限位 / e-stop 触发?")

    # 不通过 -> 右侧折线 -> 等待
    fc.ax.text(cx + 20, Y[11], "不通过", ha="center", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.polyarrow([(cx+18, Y[11]-3.5), (cx+18, Y[13]), (cx+26, Y[13])],
                 lside="right")
    fc.ax.text(cx + 30, Y[13], "丢弃\n本次动作", ha="left", va="center",
               fontsize=8.5, color="#000000")

    # 通过 -> 向下
    fc.ax.text(cx + 4, Y[11]-4.5, "通过", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.arrow(cx, Y[11]-3.5, cx, Y[12]+2.5)

    fc.parallelogram(cx, Y[12], 52, 5,
                     "发布 JointJog 命令\n-> /right_servo/moveit_servo/delta_joint_cmds", "io")

    fc.arrow(cx, Y[12]-2.5, cx, Y[13]+2.5)
    fc.rect(cx, Y[13], 52, 5,
            "等待下一控制周期\n(控制频率 ~10 Hz)", "loop")

    # 回到"获取观测"的循环箭头 (右侧)
    fc.polyarrow([(cx+26, Y[13]), (cx+26, Y[2]), (cx+26, Y[2])], lside="right")
    fc.arrow(cx+26, Y[2], cx+26, Y[2])
    # 实际画一条从 Y[13] 右边回到 Y[2] 右边的折线
    fc.ax.annotate("", xy=(cx+26, Y[2]+2.5),
                   xytext=(cx+26, Y[13]-2.5),
                   arrowprops=dict(arrowstyle="-|>", color=C["arrow"],
                                   lw=1.5, mutation_scale=14,
                                   connectionstyle="arc3,rad=0.0"))
    fc.ax.plot([cx+26, cx+26], [Y[13]-2.5, Y[2]+2.5],
               color=C["arrow"], linewidth=1.5, zorder=2)
    fc.ax.text(cx+30, (Y[13]+Y[2])/2, "下一周期", ha="left", va="center",
               fontsize=8, color="#000000", rotation=90)

    fc.save("flow_01_ddpm_inference")


# ─────────────────────────────────────────────────────────────────────────────
# flow_02  模仿学习完整 Pipeline
# ─────────────────────────────────────────────────────────────────────────────
def flow_imitation_pipeline():
    fc = FC("flow_02  模仿学习完整 Pipeline", w=11, h=20)
    cx = 50

    # 五个阶段：只用虚线边框区分，无填充
    def phase_bg(y_top, y_bot, label, _color):
        fc.ax.add_patch(mpatches.FancyBboxPatch(
            (2, y_bot), 96, y_top - y_bot,
            boxstyle="round,pad=0.3",
            facecolor=WH, edgecolor=BK,
            linewidth=0.8, linestyle="--",
            zorder=0))
        fc.ax.text(3.5, (y_top + y_bot)/2, label,
                   ha="left", va="center", fontsize=8.5,
                   color=BK, fontweight="bold", rotation=90, zorder=1)

    phase_bg(96, 80, "① 示教采集", "")
    phase_bg(79, 62, "② 数据处理", "")
    phase_bg(61, 44, "③ 模型训练", "")
    phase_bg(43, 26, "④ 离线评估", "")
    phase_bg(25,  4, "⑤ 真机部署", "")

    Y = list(range(93, 3, -9))   # 93,84,75,66,57,48,39,30,21,12

    # ① 示教采集
    fc.oval(cx, Y[0], 32, 5, "开始  (专家操作员就位)")
    fc.arrow(cx, Y[0]-2.5, cx, Y[1]+2.5)
    fc.rect(cx, Y[1], 60, 6,
            "启动控制栈\nrobot_moveit.launch.xml  use_simulation:=false", "process")
    fc.arrow(cx, Y[1]-3, cx, Y[2]+3)
    fc.rect(cx, Y[2], 60, 6,
            "键盘遥操作 + demo_recorder_node 录制\n订阅 /joint_states / /right_joint_command / camera", "io")
    fc.arrow(cx, Y[2]-3, cx, Y[3]+3)
    fc.diamond(cx, Y[3], 38, 7, "任务完成?")
    fc.ax.text(cx-22, Y[3], "否\n继续操作", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-19, Y[3]), (cx-19, Y[2]), (cx-30, Y[2])], lside="left")
    fc.ax.text(cx+4, Y[3]-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.arrow(cx, Y[3]-3.5, cx, Y[4]+3)

    # ② 数据处理
    fc.parallelogram(cx, Y[4], 60, 6,
                     "保存 episode\ndata/imitation_raw/episode_XXXXXX/\njoint_pos.npy  actions.npy  meta.json", "io")
    fc.arrow(cx, Y[4]-3, cx, Y[5]+3)
    fc.rect(cx, Y[5], 60, 6,
            "process_rosbag_data.py\n提取 controller state / 时间戳对齐 / 剔除异常帧", "process")
    fc.arrow(cx, Y[5]-3, cx, Y[6]+3)
    fc.rect(cx, Y[6], 60, 6,
            "ImitationDataset\n窗口切片 obs_horizon=2 / pred_horizon=16\n归一化 -> stats.json", "data")

    # ③ 模型训练
    fc.arrow(cx, Y[6]-3, cx, Y[7]+3)
    fc.rect(cx, Y[7], 60, 6,
            "train_diffusion_policy.py\nDDPM 训练  T=100  MSE loss\nAdamW lr=1e-4  epochs=50  batch=64", "process")
    fc.arrow(cx, Y[7]-3, cx, Y[8]+3)
    fc.diamond(cx, Y[8], 38, 7, "val loss 收敛?")
    fc.ax.text(cx-22, Y[8], "否\n继续训练", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-19, Y[8]), (cx-19, Y[7]), (cx-30, Y[7])], lside="left")
    fc.ax.text(cx+4, Y[8]-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.arrow(cx, Y[8]-3.5, cx, Y[9]+3)

    # ④ 离线评估
    fc.parallelogram(cx, Y[9], 60, 6,
                     "保存 latest.pt / best_val.pt\nconfig + stats 一并打包", "io")

    # 在 Y[9] 下方继续
    y_eval = Y[9] - 9
    fc.arrow(cx, Y[9]-3, cx, y_eval+3)
    fc.rect(cx, y_eval, 60, 6,
            "eval_diffusion_policy.py\n滚动推理 T=100 步  计算 MAE/RMSE per-joint\n生成 pred vs true 误差图", "process")

    y_dec2 = y_eval - 9
    fc.arrow(cx, y_eval-3, cx, y_dec2+3)
    fc.diamond(cx, y_dec2, 38, 7, "误差满足部署要求?")
    fc.ax.text(cx-22, y_dec2, "否\n重新采集/训练", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-19, y_dec2), (cx-19, Y[2]), (cx-30, Y[2])], lside="left")
    fc.ax.text(cx+4, y_dec2-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")

    # ⑤ 真机部署
    y_deploy = y_dec2 - 9
    fc.arrow(cx, y_dec2-3.5, cx, y_deploy+3)
    fc.rect(cx, y_deploy, 60, 6,
            "启动 servo_control.launch.py\n加载 latest.pt + stats", "process")

    y_end = y_deploy - 8
    fc.arrow(cx, y_deploy-3, cx, y_end+2.5)
    fc.oval(cx, y_end, 32, 5, "真机执行任务")

    fc.save("flow_02_imitation_pipeline")


# ─────────────────────────────────────────────────────────────────────────────
# flow_03  真机部署控制循环
# ─────────────────────────────────────────────────────────────────────────────
def flow_deploy_control_loop():
    fc = FC("flow_03  真机部署控制循环", w=10, h=17)
    cx = 50

    Y = [93, 86, 79, 72, 65, 58, 51, 44, 37, 30, 23, 16, 9]

    fc.oval(cx, Y[0], 36, 5, "节点启动  diffusion_policy_servo_node")

    fc.arrow(cx, Y[0]-2.5, cx, Y[1]+2.5)
    fc.rect(cx, Y[1], 58, 5,
            "加载检查点\nlatest.pt -> DiffusionPolicy + config + stats", "process")

    fc.arrow(cx, Y[1]-2.5, cx, Y[2]+2.5)
    fc.rect(cx, Y[2], 58, 5,
            "初始化观测缓冲区\nobs_buffer = deque(maxlen=obs_horizon=2)", "process")

    fc.arrow(cx, Y[2]-2.5, cx, Y[3]+2.5)
    fc.rect(cx, Y[3], 58, 5,
            "订阅 /joint_states\njoint_state_callback() 持续更新 obs_buffer", "io")

    fc.arrow(cx, Y[3]-2.5, cx, Y[4]+2.5)
    fc.diamond(cx, Y[4], 36, 7, "obs_buffer 已满\n(>= obs_horizon 帧)?")
    fc.ax.text(cx-22, Y[4], "否\n等待", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-18, Y[4]), (cx-18, Y[3]), (cx-29, Y[3])], lside="left")
    fc.ax.text(cx+4, Y[4]-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")

    fc.arrow(cx, Y[4]-3.5, cx, Y[5]+2.5)
    fc.rect(cx, Y[5], 58, 5,
            "归一化观测\ncond = (obs_buffer - state_mean) / state_std", "process")

    fc.arrow(cx, Y[5]-2.5, cx, Y[6]+2.5)
    fc.rect(cx, Y[6], 58, 5,
            "DDPM 去噪推理\na_T ~ N(0,I)  ->  T=100 步反向去噪  ->  a_0\n输出动作序列 [pred_horizon=16, action_dim=6]", "data")

    fc.arrow(cx, Y[6]-2.5, cx, Y[7]+2.5)
    fc.rect(cx, Y[7], 58, 5,
            "反归一化 + 取执行步\naction = a_0[:exec_steps] * std + mean", "process")

    fc.arrow(cx, Y[7]-2.5, cx, Y[8]+2.5)
    fc.diamond(cx, Y[8], 40, 7, "关节限位检查\n& e-stop 信号?")

    fc.ax.text(cx+23, Y[8], "超限/急停", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx+20, Y[8]-3.5), (cx+20, Y[11]), (cx+29, Y[11])], lside="right")
    fc.ax.text(cx+32, Y[11], "发布\n急停信号", ha="left", va="center",
               fontsize=8.5, color="#000000")

    fc.ax.text(cx+4, Y[8]-5, "正常", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")
    fc.arrow(cx, Y[8]-3.5, cx, Y[9]+2.5)

    fc.parallelogram(cx, Y[9], 58, 5,
                     "发布 JointJog\n-> /right_servo/moveit_servo/delta_joint_cmds", "io")

    fc.arrow(cx, Y[9]-2.5, cx, Y[10]+2.5)
    fc.rect(cx, Y[10], 58, 5,
            "ros2_control 执行\nJointTrajectoryController -> 硬件接口 -> CAN-FD 电机", "process")

    fc.arrow(cx, Y[10]-2.5, cx, Y[11]+2.5)
    fc.rect(cx, Y[11], 58, 5,
            "等待下一控制周期  (~10 Hz)\n更新 obs_buffer", "loop")

    # 回到 obs_buffer 满判断
    fc.ax.plot([cx-29, cx-29], [Y[11]-2.5, Y[4]], color=C["arrow"],
               linewidth=1.5, zorder=2)
    fc.ax.annotate("", xy=(cx-18, Y[4]),
                   xytext=(cx-29, Y[4]),
                   arrowprops=dict(arrowstyle="-|>", color=C["arrow"],
                                   lw=1.5, mutation_scale=14))
    fc.ax.text(cx-33, (Y[11]+Y[4])/2, "循环", ha="right", va="center",
               fontsize=8, color="#000000", rotation=90)

    fc.arrow(cx, Y[11]-2.5, cx, Y[12]+2.5)
    fc.oval(cx, Y[12], 28, 5, "节点关闭 / 任务结束")

    fc.save("flow_03_deploy_control_loop")


# ─────────────────────────────────────────────────────────────────────────────
# flow_04  数据采集与处理流程
# ─────────────────────────────────────────────────────────────────────────────
def flow_data_pipeline():
    fc = FC("flow_04  数据采集与处理流程", w=10, h=17)
    cx = 50

    Y = [93, 86, 79, 72, 65, 58, 51, 44, 37, 30, 23, 16, 9]

    fc.oval(cx, Y[0], 28, 5, "开始")

    fc.arrow(cx, Y[0]-2.5, cx, Y[1]+2.5)
    fc.rect(cx, Y[1], 60, 5,
            "启动 demo_recorder_node\n配置采样率 / 输出目录 / 话题列表", "process")

    fc.arrow(cx, Y[1]-2.5, cx, Y[2]+2.5)
    fc.rect(cx, Y[2], 60, 5,
            "demo_control start\n开始录制 episode", "io")

    fc.arrow(cx, Y[2]-2.5, cx, Y[3]+2.5)
    fc.rect(cx, Y[3], 60, 5,
            "同步采样\n/joint_states + /right_joint_command\n时间戳对齐 -> npy 缓冲", "process")

    fc.arrow(cx, Y[3]-2.5, cx, Y[4]+2.5)
    fc.diamond(cx, Y[4], 36, 7, "任务完成?")
    fc.ax.text(cx-22, Y[4], "否\n继续", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-18, Y[4]), (cx-18, Y[3]), (cx-30, Y[3])], lside="left")
    fc.ax.text(cx+4, Y[4]-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")

    fc.arrow(cx, Y[4]-3.5, cx, Y[5]+2.5)
    fc.rect(cx, Y[5], 60, 5,
            "demo_control stop --success / --fail\n保存 meta.json (success flag)", "io")

    fc.arrow(cx, Y[5]-2.5, cx, Y[6]+2.5)
    fc.diamond(cx, Y[6], 36, 7, "标记成功?")
    fc.ax.text(cx-22, Y[6], "否\n丢弃", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-18, Y[6]), (cx-18, Y[2]), (cx-30, Y[2])], lside="left")
    fc.ax.text(cx+4, Y[6]-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")

    fc.arrow(cx, Y[6]-3.5, cx, Y[7]+2.5)
    fc.parallelogram(cx, Y[7], 60, 5,
                     "data/imitation_raw/episode_XXXXXX/\njoint_pos.npy  actions.npy  timestamps.npy  meta.json", "io")

    fc.arrow(cx, Y[7]-2.5, cx, Y[8]+2.5)
    fc.rect(cx, Y[8], 60, 5,
            "process_rosbag_data.py\n提取 controller state / 剔除异常帧\n-> data/processed/real_robot/training_episodes/", "process")

    fc.arrow(cx, Y[8]-2.5, cx, Y[9]+2.5)
    fc.rect(cx, Y[9], 60, 5,
            "ImitationDataset\n滑动窗口切片  obs_horizon=2 / pred_horizon=16\n计算 mean/std -> stats.json", "data")

    fc.arrow(cx, Y[9]-2.5, cx, Y[10]+2.5)
    fc.diamond(cx, Y[10], 40, 7, "样本数量\n>= 最小训练要求?")
    fc.ax.text(cx-24, Y[10], "否\n继续采集", ha="center", va="center",
               fontsize=8, color="#000000", fontweight="bold")
    fc.polyarrow([(cx-20, Y[10]), (cx-20, Y[2]), (cx-30, Y[2])], lside="left")
    fc.ax.text(cx+4, Y[10]-5, "是", ha="left", va="center",
               fontsize=8.5, color="#000000", fontweight="bold")

    fc.arrow(cx, Y[10]-3.5, cx, Y[11]+2.5)
    fc.rect(cx, Y[11], 60, 5,
            "DataLoader\ntrain/val split  batch_size=64  shuffle=True", "process")

    fc.arrow(cx, Y[11]-2.5, cx, Y[12]+2.5)
    fc.oval(cx, Y[12], 28, 5, "数据就绪  -> 进入训练")

    fc.save("flow_04_data_pipeline")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("生成程序流程图...")
    flow_ddpm_inference()
    flow_imitation_pipeline()
    flow_deploy_control_loop()
    flow_data_pipeline()
    print(f"\n全部完成，保存在: {OUT.resolve()}")
