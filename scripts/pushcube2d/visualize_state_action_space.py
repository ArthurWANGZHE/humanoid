"""
Visualize the state/observation space and action space of PushCube2D.
Produces a publication-style diagram saved to figure/pushcube2d/state_action_space.png
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, Circle, FancyBboxPatch
from matplotlib.gridspec import GridSpec

REPO_ROOT = Path(__file__).resolve().parents[2]
PUSHCUBE_ROOT = REPO_ROOT / "psuhcube"
FIGURE_DIR = REPO_ROOT / "figure" / "pushcube2d"

sys.path.insert(0, str(PUSHCUBE_ROOT))
from pushcube2d.env import PushCube2DEnv

# ── 1. Get a rendered frame ──────────────────────────────────────────────────
env = PushCube2DEnv()
obs, _ = env.reset(seed=7)
frame = env.render(mode="rgb_array")

pusher_xy  = env.pusher_xy.copy()
cube_xy    = env.cube_xy.copy()
goal_xy    = env.goal_xy.copy()
cube_theta = env.cube_theta

def w2px(xy, size=512):
    x = int(np.clip(xy[0] * (size - 1), 0, size - 1))
    y = int(np.clip((1.0 - xy[1]) * (size - 1), 0, size - 1))
    return x, y

px_pusher = w2px(pusher_xy)
px_cube   = w2px(cube_xy)
px_goal   = w2px(goal_xy)

# ── 2. Layout ────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 7), facecolor="#F8F7F2")
gs  = GridSpec(1, 3, figure=fig, width_ratios=[1.15, 1.0, 0.85],
               left=0.03, right=0.97, top=0.88, bottom=0.08, wspace=0.35)

# ── Panel A: annotated scene ─────────────────────────────────────────────────
ax_scene = fig.add_subplot(gs[0])
ax_scene.imshow(frame, origin="upper")
ax_scene.set_xlim(0, 511); ax_scene.set_ylim(511, 0)
ax_scene.axis("off")
ax_scene.set_title("Simulation Scene", fontsize=13, fontweight="bold",
                   color="#2C2C2C", pad=6)

ann_kw = dict(fontsize=9.5, color="white",
              arrowprops=dict(arrowstyle="->", color="white", lw=1.4),
              bbox=dict(boxstyle="round,pad=0.3", fc="#333333", ec="none", alpha=0.82))

ax_scene.annotate("Pusher\n(circle)", xy=px_pusher,
                  xytext=(px_pusher[0]+80, px_pusher[1]-70), **ann_kw)
ax_scene.annotate("Cube\n(object)", xy=px_cube,
                  xytext=(px_cube[0]-130, px_cube[1]-80), **ann_kw)
ax_scene.annotate("Goal\nregion", xy=px_goal,
                  xytext=(px_goal[0]+60, px_goal[1]+80), **ann_kw)

# action arrow on scene
ax_scene.annotate("", xy=(px_pusher[0]+55, px_pusher[1]-45),
                  xytext=px_pusher,
                  arrowprops=dict(arrowstyle="-|>", color="#FFD166", lw=2.2))
ax_scene.text(px_pusher[0]+62, px_pusher[1]-52, "action\n(dx, dy)",
              fontsize=8.5, color="#FFD166",
              bbox=dict(boxstyle="round,pad=0.25", fc="#333333", ec="none", alpha=0.75))

# ── Panel B: observation vector diagram ──────────────────────────────────────
ax_obs = fig.add_subplot(gs[1])
ax_obs.set_xlim(0, 10); ax_obs.set_ylim(0, 10)
ax_obs.axis("off")
ax_obs.set_title("Observation Space  $o \\in \\mathbb{R}^7$",
                 fontsize=13, fontweight="bold", color="#2C2C2C", pad=6)

groups = [
    ("Pusher position",  ["pusher_x", "pusher_y"],              "#E07B54", "[0, 1]"),
    ("Cube position",    ["cube_x",   "cube_y"],                "#5B8DB8", "[0, 1]"),
    ("Cube orientation", ["cube_θ"],                            "#8E6BBF", "[−π, π]"),
    ("Goal position",    ["goal_x",   "goal_y"],                "#4CAF7D", "[0, 1]"),
]

y_top = 9.2
row_h = 0.62
gap   = 0.28
y = y_top
for label, dims, color, rng in groups:
    # group label
    ax_obs.text(0.3, y, label, fontsize=9.5, fontweight="bold",
                color=color, va="top")
    y -= 0.42
    for d in dims:
        rect = FancyBboxPatch((0.3, y - row_h + 0.06), 9.2, row_h - 0.08,
                              boxstyle="round,pad=0.05",
                              linewidth=1.2, edgecolor=color,
                              facecolor=color + "28" if len(color) == 7 else color,
                              alpha=0.18)
        # manual alpha fill
        rect2 = FancyBboxPatch((0.3, y - row_h + 0.06), 9.2, row_h - 0.08,
                               boxstyle="round,pad=0.05",
                               linewidth=1.2, edgecolor=color,
                               facecolor=color, alpha=0.15)
        ax_obs.add_patch(rect2)
        ax_obs.text(1.0, y - row_h/2 + 0.06, d, fontsize=9.5,
                    va="center", color="#1A1A1A", fontfamily="monospace")
        ax_obs.text(8.8, y - row_h/2 + 0.06, rng, fontsize=8.5,
                    va="center", ha="right", color="#555555")
        y -= row_h
    y -= gap

# bracket label
ax_obs.text(5.0, 0.35, "continuous, normalized to unit table  [0, 1]²",
            fontsize=8.5, ha="center", color="#777777", style="italic")

# ── Panel C: action space diagram ────────────────────────────────────────────
ax_act = fig.add_subplot(gs[2])
ax_act.set_xlim(-1.6, 1.6); ax_act.set_ylim(-1.6, 1.6)
ax_act.set_aspect("equal")
ax_act.set_facecolor("#F8F7F2")
ax_act.set_title("Action Space  $a \\in [-1,1]^2$",
                 fontsize=13, fontweight="bold", color="#2C2C2C", pad=6)

# bounding box
box = patches.Rectangle((-1, -1), 2, 2, linewidth=1.8,
                         edgecolor="#AAAAAA", facecolor="#EEEEEE", zorder=1)
ax_act.add_patch(box)

# axes
ax_act.axhline(0, color="#BBBBBB", lw=0.8, zorder=2)
ax_act.axvline(0, color="#BBBBBB", lw=0.8, zorder=2)

# sample action arrows
rng = np.random.default_rng(0)
for _ in range(14):
    a = rng.uniform(-0.9, 0.9, 2)
    ax_act.annotate("", xy=a, xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="#E07B54",
                                   lw=1.3, alpha=0.55), zorder=3)

# highlight one canonical arrow
ax_act.annotate("", xy=(0.65, 0.55), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#E07B54", lw=2.5), zorder=4)
ax_act.text(0.68, 0.58, "(dx, dy)", fontsize=9, color="#C0392B", fontweight="bold")

# origin dot
ax_act.plot(0, 0, "o", ms=6, color="#333333", zorder=5)

# corner labels
for xv, yv, lbl in [(-1,-1,"(−1,−1)"),( 1,-1,"(1,−1)"),
                     (-1, 1,"(−1, 1)"),( 1, 1,"(1, 1)")]:
    ax_act.text(xv*1.02, yv*1.08, lbl, fontsize=7.5,
                ha="center", va="center", color="#888888")

ax_act.set_xticks([-1, -0.5, 0, 0.5, 1])
ax_act.set_yticks([-1, -0.5, 0, 0.5, 1])
ax_act.tick_params(labelsize=8, colors="#666666")
ax_act.set_xlabel("dx  (normalized)", fontsize=9, color="#444444")
ax_act.set_ylabel("dy  (normalized)", fontsize=9, color="#444444")
for spine in ax_act.spines.values():
    spine.set_visible(False)

# scale note
ax_act.text(0, -1.48,
            "scaled by action_scale = 0.035 m/step",
            fontsize=8, ha="center", color="#888888", style="italic")

# ── Super-title removed ───────────────────────────────────────────────────────

# ── Save ─────────────────────────────────────────────────────────────────────
FIGURE_DIR.mkdir(parents=True, exist_ok=True)
out_path = FIGURE_DIR / "state_action_space.png"
fig.savefig(out_path, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())
plt.close(fig)
print(f"Saved → {out_path}")
