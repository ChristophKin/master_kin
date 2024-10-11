# Source the ROS environment setup
source /opt/ros/${ROS_DISTRO}/setup.bash

cd IsaacSim-ros_workspaces/${ROS_DISTRO}_ws

# Install dependencies specified in package.xml files inside the src directory
rosdep install --from-paths src --ignore-src -r -y

# colcon build compiles all packages. Optionally, '--cmake-clean-cache' can be uncommented to clean the cache before building.
colcon build #--cmake-clean-cache

# Source the newly built workspace to overlay the environment with new packages.
source install/setup.bash

cd ../..
cd go2_omniverse_ws

# Install the necessary dependencies for this workspace
rosdep install --from-paths src --ignore-src -r -y

# colcon build compiles all packages. Optionally, '--cmake-clean-cache' can be uncommented to clean the cache before building.
colcon build #--cmake-clean-cache

# Source the workspace to make the built packages available for use
source install/setup.bash

cd ..

# Initialize Conda shell environment and hook it into the current shell session
eval "$(conda shell.bash hook)"

# Activate the 'orbit' conda environment
conda activate orbit

# Preload the standard C++ library (libstdc++) before executing the Python script.
# This is necessary for resolving compatibility issues with dynamic libraries.
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Run the main Python script with arguments:
# --robot: go1 or go2
# --custom_env: little_park, office or warehouse_full
python main.py --robot go2 --custom_env little_park &

# Execute a external shell script to run the point cloud modifier.
# This is to make the published Lidar data format from Isaac suitable for Autoware.
/home/kinbergeradm/pointcloud_processor/run_pointcloudmodifier.sh