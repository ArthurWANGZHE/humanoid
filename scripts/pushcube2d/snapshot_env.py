"""Render a single frame of the PushCube2D environment and save as PNG."""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PUSHCUBE_ROOT = REPO_ROOT / "psuhcube"
FIGURE_DIR = REPO_ROOT / "figure" / "pushcube2d"

sys.path.insert(0, str(PUSHCUBE_ROOT))

from pushcube2d.env import PushCube2DEnv
import numpy as np

env = PushCube2DEnv()
obs, _ = env.reset(seed=42)
frame = env.render(mode="rgb_array")  # shape: (512, 512, 3)

# Save using matplotlib (no PIL dependency needed)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(5, 5))
ax.imshow(frame)
ax.axis("off")
ax.set_title("PushCube2D — Initial Scene", fontsize=13, pad=8)
out_path = FIGURE_DIR / "env_snapshot.png"
out_path.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out_path, dpi=150, bbox_inches="tight")
plt.close(fig)

print(f"Saved: {out_path}")
