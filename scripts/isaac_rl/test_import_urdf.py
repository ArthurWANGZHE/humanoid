from pathlib import Path
import sys

ISAAC_RL_ROOT = Path(__file__).resolve().parents[2] / "RL_training" / "Isaac_RL"
sys.path.insert(0, str(ISAAC_RL_ROOT))

from rl_train.test_import_urdf import main


if __name__ == "__main__":
    main()
