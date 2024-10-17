import os
from pathlib import Path

import omni.isaac.nucleus as nucleus

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

# environment settings

# Setup the default simulation environments path
NVIDIA_ASSETS_PATH = str(nucleus.get_assets_root_path())
ISAAC_SIM_ENVIRONMENTS = "/Isaac/Environments"
NVIDIA_SIMULATION_ENVIRONMENTS = {
    "Default Environment": "Grid/default_environment.usd",
    "Black Gridroom": "Grid/gridroom_black.usd",
    "Curved Gridroom": "Grid/gridroom_curved.usd",
    "Hospital": "Hospital/hospital.usd",
    "Office": "Office/office.usd",
    "Simple Room": "Simple_Room/simple_room.usd",
    "Warehouse": "Simple_Warehouse/warehouse.usd",
    "Warehouse with Forklifts": "Simple_Warehouse/warehouse_with_forklifts.usd",
    "Warehouse with Shelves": "Simple_Warehouse/warehouse_multiple_shelves.usd",
    "Full Warehouse": "Simple_Warehouse/full_warehouse.usd",
    "Flat Plane": "Terrains/flat_plane.usd",
    "Rough Plane": "Terrains/rough_plane.usd",
    "Slope Plane": "Terrains/slope.usd",
    "Stairs Plane": "Terrains/stairs.usd",
}

SIMULATION_ENVIRONMENTS = {}

# Add the Isaac Sim assets to the list
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = (
        NVIDIA_ASSETS_PATH + ISAAC_SIM_ENVIRONMENTS + "/" + NVIDIA_SIMULATION_ENVIRONMENTS[asset]
    )

# Define the default settings for the simulation environment
DEFAULT_WORLD_SETTINGS = {
    "physics_dt": 0.005,
    "stage_units_in_meters": 1.0,
    "rendering_dt": 0.02,
}
