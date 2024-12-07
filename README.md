# StrideSim

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
  - [Docker Installation (Recommended)](#docker-installation-recommended)
  - [Local Installation](#local-installation)
- [Tips](#tips)
- [Usage](#usage)


[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.0.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview

StrideSim is a project based on Isaac Lab. This repository is designed to allow development in an independent environment outside the core repository of Isaac Lab.

## Docker Installation (Recommended)

You can build and run the container using the Dockerfile.

[Docker Installation Guide](docker/README.md)

## Local Installation

1. Install Isaac Sim 4.0.0: Refer to the [Installation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html).

2. Install [Isaac Lab v1.0.0](https://github.com/isaac-sim/IsaacLab/tree/v1.0.0): Refer to the [Installation Guide](https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html).

3. Install StrideSim:

   ```bash
   git clone https://github.com/AuTURBO/StrideSim.git
   ```

   ```bash
   sudo apt-get install -y git-lfs
   git lfs install

   cd StrideSim
   git lfs pull
   ```

## Tips

1. Set environment variables:

   ```bash
   # Isaac Sim root directory
   export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.0.0"
   # Isaac Sim python executable
   alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
   # Isaac Sim app
   alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"
   ```

## Usage

1. Install the reinforcement learning library:

   ```bash
   cd rl
   python -m pip install -e .
   ```

2. Run reinforcement learning independently:

   ```bash
   python rl/train.py --task Template-Isaac-Velocity-Rough-Anymal-D-v0
   ```

3. Run StrideSim:

   3-1. Execute the program:

   ```bash
   # After setting environment variables
   ISAACSIM
   ```

   3-2. Configure the extension program:

   - Navigate to window -> extension.
   - Click the trident button to insert the extension program path (include up to the exts of this project).
   - Click the simulation button on the left (enabling AUTOLOAD is convenient).

   3-3. Run the extension program:
   ![Description of image](Asset/readme/image-1.png)
   You can now see the StrideSim_AnymalD tab in the Isaac Examples tab.

   ![Description of image](Asset/readme/image-2.png)
   By clicking the button, you can view scenes like the one above, and perform tasks such as calling anymalD, training, and parallel execution.

## Code Formatting

pre-commit hook is used to automate code formatting.

Install pre-commit:

```bash
pip install pre-commit
```

Run pre-commit:

```bash
pre-commit run --all-files
```

## License

This project is distributed under the MIT license.
