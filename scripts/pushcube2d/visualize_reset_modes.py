"""
Render three side-by-side scenes showing aligned / near / random reset modes.
Saved to figure/pushcube2d/reset_modes.png
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[2]
PUSHCUBE_ROOT = REPO_ROOT / "psuhcube"
FIGURE_DIR = REPO_ROOT / "figure" / "pushcube2d"

sys.path.insert(0, str(PUSHCUBE_ROOT))
from pushcube2d.env import PushCube2DEnv

modes = ["aligned", "near", "random"]
titles = [
    "Aligned\n(pusher behind cube, facing goal)",
    "Near\n(pusher close, random angle)",
    "Random\n(pusher anywhere on table)",
]

frames = []
for mode in modes:
    env = PushCube2DEnv(pusher_reset_mode=mode)
    env.reset(seed=42)
    if mode == "random":
        # Force pusher to a far corner to make the randomness visually obvious
        env.pusher_xy = np.array([0.92, 0.08], dtype=np.float32)
    frames.append(env.render(mode="rgb_array"))

fig, axes = plt.subplots(1, 3, figsize=(13, 4.8), facecolor="#F8F7F2")
fig.subplots_adjust(left=0.02, right=0.98, top=0.82, bottom=0.02, wspace=0.06)

for ax, frame, title in zip(axes, frames, titles):
    ax.imshow(frame, origin="upper")
    ax.axis("off")
    ax.set_title(title, fontsize=11.5, fontweight="bold",
                 color="#2C2C2C", pad=8, linespacing=1.5)

FIGURE_DIR.mkdir(parents=True, exist_ok=True)
out_path = FIGURE_DIR / "reset_modes.png"
fig.savefig(out_path, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())
plt.close(fig)
print(f"Saved → {out_path}")
