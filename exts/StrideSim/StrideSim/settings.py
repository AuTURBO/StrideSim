import os
from pathlib import Path

# Build paths inside the project like this: BASE_DIR / 'subdir'.
BASE_DIR = Path(__file__).resolve().parent.parent.parent.parent

# Environment variables
AGENT_DIR = os.path.join(BASE_DIR, "logs")


# FIXME: This is a hack to find the IsaacLab directory
def find_isaaclab_dir(start_path="/"):
    for root, dirs, _ in os.walk(start_path):
        if "IsaacLab" in dirs:
            return os.path.abspath(os.path.join(root, "IsaacLab"))
    return None


# Use the function to set ISAACLAB_DIR
ISAACLAB_DIR = find_isaaclab_dir() or "/home/jin/Documents/IsaacLab"
# ISAACLAB_DIR = "/home/jin/Documents/IsaacLab"

ISAACLAB_LAB = os.path.join(ISAACLAB_DIR, "source/extensions/omni.isaac.lab")
ISAACLAB_LAB_TASKS = os.path.join(ISAACLAB_DIR, "source/extensions/omni.isaac.lab_tasks")
ISAACLAB_LAB_ASSETS = os.path.join(ISAACLAB_DIR, "source/extensions/omni.isaac.lab_assets")


RL_DIR = os.path.join(BASE_DIR, "rl")
