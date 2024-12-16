# Unitree Go1 and Go2 in Isaac Sim

[![IsaacSim](https://img.shields.io/badge/IsaacSim-orbit-gold.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)

Implementation of Unitree Go1 and Go2 in Isaac Sim including:
  - RL Controller for both quadrupeds
  - Lidar, IMU, Odometry and Camera Image Data publisher via ROS2
  - different Lidar configuration available (Ouster OS1 and Unitree L1)
  - Quadrupeds controllable via Keyboard and ROS2 commands
  - different environments (office, park, warehouse)

## System requirements and installation
You need to install:
1. Ubuntu 22.04
2. Nvidia Isaac Sim 2023.1.1
3. Ros2 Humble
4. Nvidia Orbit 0.3.0


Full instruction:

After installation of Nvidia Isaac Sim 2023.1.1 and Ros2 Humble:

1. Clone this specific IsaacLab repo version: https://github.com/isaac-sim/IsaacLab/releases/tag/v0.3.1
2. Execute in ubuntu terminal:
```
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-2023.1.1"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```
and also put it inside .bashrc file

3. Inside the root folder of Orbit repo (https://github.com/isaac-sim/IsaacLab/releases/tag/v0.3.1) execute `ln -s ${ISAACSIM_PATH} _isaac_sim`
4. Execute `./orbit.sh --conda`
5. Execute `conda activate orbit`
6. Execute `sudo apt install cmake build-essential`
7. Execute `./orbit.sh --install`
8. Execute `./orbit.sh --extra rsl_rl`
9. Verify the installation using "python source/standalone/tutorials/00_sim/create_empty.py" You should be inside conda env.
10.   You need to check that you have "Isaac Sim Python 2023.1.1 - New Stage*" on the top of the window.
11.   Clone this repo with `git clone https://github.com/... --recurse-submodules -j8 --depth=1`
12. Copy the config file Unitree_L1.json (located in "Lidar config files") for the Unitree L1 LiDAR to the folder IsaacLab-0.3.1/source/data/sensors/lidar/Unitree_L1.json (if the path doesnt exists, create it)
13. Copy all material files in the isaac-sim-2023.1.1/data/material_files folder to IsaacLab_v0.3.1/source/data/material_files (if the path doesnt exists, create it)
14. Execute ./run_sim.sh (without activated conda orbit env)


Some suggestions:
1. You need to check `nvidia-smi`, it should work, before installing Isaac Sim
2. You need to install Miniconda and execute: `conda config --set auto_activate_base false`
3. Install Omniverse launcher and then install Isaac Sim.
4. You need to install ROS2 on your system and configure it: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros


## Usage
The current project was tested on Ubuntu 22.04, IsaacSim 2023.1.1 with IsaacLab 0.3.1 and Nvidia Driver Version: 550.
To start the project, execute:

`./run_sim.sh`

You can control the quadruped robot using "WASD, Q and E" keyboard commands. Change the run_sim.sh file for Unitree Go1 or Go2 and different environments.


## Select custom env

To use predifined custom envs, you need to download files from *URL* and place them to /envs folder.
Then you can execute it modifying run_sim.sh script with --custom_env=office commands. 
