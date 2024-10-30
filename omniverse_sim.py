"""Script to play a checkpoint if an RL agent from RSL-RL."""
from __future__ import annotations

"""Launch Isaac Sim Simulator first."""
import argparse
from omni.isaac.orbit.app import AppLauncher

import cli_args  
import time
import os
import threading

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--custom_env", type=str, default="office", help="Setup the environment")
parser.add_argument("--robot", type=str, default="go1", help="Setup the robot")
parser.add_argument("--terrain", type=str, default="rough", help="Setup the robot")
parser.add_argument("--robot_amount", type=int, default=1, help="Setup the robot amount")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

"""Rest everything follows."""
import gymnasium as gym
import torch
import carb
from omni.isaac.core.utils.carb import set_carb_setting

from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper)
import omni.isaac.orbit.sim as sim_utils
import omni.appwindow
from rsl_rl.runners import OnPolicyRunner

import rclpy
from ros2 import RobotBaseNode, add_camera, add_rtx_lidar, pub_robo_data_ros2
from geometry_msgs.msg import Twist

from agent_cfg import unitree_go2_agent_cfg, unitree_go1_agent_cfg
from custom_rl_env import UnitreeGo2CustomEnvCfg, UnitreeGo1CustomEnvCfg
import custom_rl_env

from omnigraph import create_front_cam_omnigraph


def sub_keyboard_event(event) -> bool:
    if len(custom_rl_env.base_command) > 0:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == 'W':
                custom_rl_env.base_command["0"] = [1, 0, 0]
            if event.input.name == 'S':
                custom_rl_env.base_command["0"] = [-1, 0, 0]
            if event.input.name == 'A':
                custom_rl_env.base_command["0"] = [1, 0, 1]
            if event.input.name == 'D':
                custom_rl_env.base_command["0"] = [1, 0, -1]
            if event.input.name == 'Q':
                custom_rl_env.base_command["0"] = [0, 0, 1]
            if event.input.name == 'E':
                custom_rl_env.base_command["0"] = [0, 0, -1]

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            custom_rl_env.base_command["0"] = [0, 0, 0]
    return True


def setup_custom_env():
    try:
        if args_cli.custom_env == "park":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/littlepark_cars.usd")
            cfg_scene.func("/World/park", cfg_scene, translation=(0.0, 0.0, 0.0))

        if args_cli.custom_env == "office":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/office_walls.usd")
            cfg_scene.func("/World/office", cfg_scene, translation=(0.0, 0.0, 0.0))

        if args_cli.custom_env == "warehouse":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/warehouse_full.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(0.0, 0.0, 0.0))

        if args_cli.custom_env == "brownstone":
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/brownstone.usd")
            cfg_scene.func("/World/brownstone", cfg_scene, translation=(0.0, 0.0, 0.0))

    except:
        print("[ERROR]: Exception loading custom environment.")


def cmd_vel_cb(msg):
    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z
    custom_rl_env.base_command["0"] = [x, y, z]


def add_cmd_sub():
    node_test = rclpy.create_node('position_velocity_publisher')
    node_test.create_subscription(Twist, 'robot/cmd_vel', lambda msg, i=1: cmd_vel_cb(msg), 10)
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node_test,), daemon=True)
    thread.start()


def specify_cmd_for_robots():
    custom_rl_env.base_command["0"] = [0, 0, 0]


def run_sim():
    # acquire input interface
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, sub_keyboard_event)

    """Play with RSL-RL agent."""
    # parse configuration
    if args_cli.robot == "go1":
        env_cfg = UnitreeGo1CustomEnvCfg() 
    else:
        env_cfg = UnitreeGo2CustomEnvCfg()

    # add N robots to env 
    env_cfg.scene.num_envs = 1

    # create ros2 camera stream omnigraph
    create_front_cam_omnigraph(args_cli.robot)
        
    specify_cmd_for_robots()

    if args_cli.robot == "go1":
        agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go1_agent_cfg
    else:
        agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go2_agent_cfg

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)
    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ppo_runner.load(resume_path)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # reset environment
    obs, _ = env.get_observations()

    # initialize ROS2 node
    rclpy.init()
    base_node = RobotBaseNode()
    add_cmd_sub()

    annotator_lst = add_rtx_lidar(args_cli.robot, False)
    add_camera(args_cli.robot)
    setup_custom_env()

    # render settings interface
    carb_settings = carb.settings.get_settings()
    # set render values
    set_carb_setting(carb_settings, "/rtx/ecoMode/enabled", True)
    set_carb_setting(carb_settings, "/rtx/indirectDiffuse/enabled", False)
    set_carb_setting(carb_settings, "/rtx/ambientOcclusion/enabled", False)
    set_carb_setting(carb_settings, "/rtx/reflections/enabled", True)
    set_carb_setting(carb_settings, "/rtx/raytracing/subsurface/enabled", False)

    start_time = time.time()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, _, _ = env.step(actions)
            pub_robo_data_ros2(base_node, env, annotator_lst)
    env.close()