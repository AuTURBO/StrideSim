# StrideSim

<p align="center">
  <img src="exts/stride.simulator/data/icon.png" alt="" width="300" />
</p>

StrideSim is a Nvidia Isaac Sim-based robot simulator, offering a realistic environment and compatibility with ROS2 for efficient robotics development.
We are implementing a **quadruped robot** as our first target.

## Folders

-   `app` - It is a folder link to the location of your _Omniverse Kit_ based app. When you clone this repository, there is no app folder.
-   `exts` - It is a folder where you can add new extensions. It was automatically added to extension search path. (Extension Manager -> Gear Icon -> Extension Search Path).

Open this folder using Visual Studio Code. It will suggest you to install few extensions that will make python experience better.

Look for "stride.simulator" extension in extension manager and enable it. Try applying changes to any python files, it will hot-reload and you can observe results immediately.

Alternatively, you can launch your app from console with this folder added to search path and your extension enabled, e.g.:

```
> app/isaac-sim.sh --ext-folder exts --enable stride.simulator
```

## App Link Setup

If `app` folder link doesn't exist or broken it can be created again. For better developer experience it is recommended to create a folder link named `app` to the _Omniverse Kit_ app installed from _Omniverse Launcher_. Convenience script to use is included.

Run:

```
> ./link_app.sh
```

If successful you should see `app` folder link in the root of this repo.

If multiple Omniverse apps is installed script will select recommended one. Or you can explicitly pass an app:

```
> ./link_app.sh --path "/home/${USER}/.local/share/ov/pkg/${ISAAC_FOLDER}"
```

## Set Environment variables

Set environment variables to facilitate running Isaac Sim in the `~/.bashrc`.

```bash
# ~/.bashrc

# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/${ISAAC_FOLDER}"
# Isaac Sim python executable
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
# Isaac Sim app
alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"
```

## Acknowledge

We developed this project by referencing the code framework of [the Pegasus simulator](https://github.com/PegasusSimulator/PegasusSimulator).

```
@misc{jacinto2023pegasus,
      title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation},
      author={Marcelo Jacinto and João Pinto and Jay Patrikar and John Keller and Rita Cunha and Sebastian Scherer and António Pascoal},
      year={2023},
      eprint={2307.05263},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## TODO

-   [ ] Implement MVP of quadruped robot simulation until March 2024.
