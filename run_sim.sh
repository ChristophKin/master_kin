# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
# --robot go1, go2 or g1
# --terrain flat or rough
# --custom_env littlepark, office or warehouse_full
python main.py --robot go1 --terrain rough --custom_env littlepark