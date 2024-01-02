import os
from pathlib import Path

import omni.isaac.core.utils.nucleus as nucleus

# Get the current directory of where this extension is located
EXTENSION_FOLDER_PATH = Path(os.path.dirname(os.path.realpath(__file__)))
ROOT = str(EXTENSION_FOLDER_PATH.parent.parent.parent.resolve())

# Get the configurations file path
CONFIG_FILE = ROOT + "/stride.simulator/config/configs.yaml"

# Setup the default simulation environments path
NVIDIA_ASSETS_PATH = str(nucleus.get_assets_root_path())

print(NVIDIA_ASSETS_PATH)

ISAAC_SIM_ROBOTS = "Isaac/Robots"

# Define the built in robots of the extension
NVIDIA_SIMULATION_ROBOTS = {
    "Anymal C": "ANYbotics/anymal_c.usd",
    "go1": "Unitree/go1.usd",
}

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

ROBOTS_ENVIRONMNETS = {}

# Add the Isaac Sim assets to the list
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    ROBOTS_ENVIRONMNETS[asset] = (NVIDIA_ASSETS_PATH + ISAAC_SIM_ROBOTS + "/" + NVIDIA_SIMULATION_ROBOTS[asset])

SIMULATION_ENVIRONMENTS = {}

# Add the Isaac Sim assets to the list
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = (NVIDIA_ASSETS_PATH + ISAAC_SIM_ENVIRONMENTS + "/" +
                                      NVIDIA_SIMULATION_ENVIRONMENTS[asset])
