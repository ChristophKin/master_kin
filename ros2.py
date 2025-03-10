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

import asyncio
import time

import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TwistWithCovarianceStamped

from omni.isaac.orbit.sensors import CameraCfg, Camera
from omni.isaac.sensor import LidarRtx
import omni.replicator.core as rep
import omni.isaac.orbit.sim as sim_utils


def add_rtx_lidar(robot_type, debug=False):
    annotator_lst = []
    if robot_type == "go1":
        lidar_sensor = LidarRtx(f'/World/envs/env_0/Robot/trunk/lidar_sensor',
                                rotation_frequency = 200,
                                pulse_time=1, 
                                translation=(0.1, -0.1, -0.1),
                                orientation=(1.0, 0.0, 0.0, 0.0),   #Isaac Sim Core API: (QW, QX, QY, QZ)
                                #config_file_name= "Unitree_L1",
                                config_file_name= "OS1_REV6_32ch20hz1024res",
                                )

    else:
        lidar_sensor = LidarRtx(f'/World/envs/env_0/Robot/base/lidar_sensor',
                                rotation_frequency = 200,
                                pulse_time=1, 
                                translation=(0.0, 0.0, 0.1),
                                orientation=(1.0, 0.0, 0.0, 0.0),
                                config_file_name= "Unitree_L1",)

    if debug:
        # Create the debug draw pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
        writer.attach([lidar_sensor.get_render_product_path()])

    annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
    annotator.attach(lidar_sensor.get_render_product_path())
    annotator_lst.append(annotator)
    return annotator_lst


def add_camera(robot_type):
    cameraCfg = CameraCfg(
        prim_path=f"/World/envs/env_0/Robot/trunk/front_cam",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)),
        offset=CameraCfg.OffsetCfg(pos=(0.32487, -0.1, -0.1), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),)

    if robot_type == "go2":
        cameraCfg.prim_path = f"/World/envs/env_0/Robot/base/front_cam"
        cameraCfg.offset = CameraCfg.OffsetCfg(pos=(0.32487, -0.00095, 0.05362), rot=(0.5, -0.5, 0.5, -0.5), convention="ros")

    Camera(cameraCfg)


def pub_robo_data_ros2(base_node, env, annotator_lst, start_time):
    base_node.publish_joints(env.env.scene["robot"].data.joint_names, env.env.scene["robot"].data.joint_pos[0])
    base_node.publish_tf(env.env.scene["robot"].data.root_state_w[0, :3], 
                           env.env.scene["robot"].data.root_state_w[0, 3:7])
    base_node.publish_imu(env.env.scene["robot"].data.root_state_w[0, 3:7], 
                          env.env.scene["robot"].data.body_lin_acc_w[0, 1, :], 
                          env.env.scene["robot"].data.root_ang_vel_b[0, :])
    base_node.publish_twist(env.env.scene["robot"].data.root_lin_vel_b[0, :], 
                            env.env.scene["robot"].data.root_ang_vel_b[0, :])
                            
    try:
        if (time.time() - start_time) > 1/20:
            base_node.publish_lidar(annotator_lst[0].get_data()['data'])
            start_time = time.time()
    except:
        pass


class RobotBaseNode(Node):
    def __init__(self):
        super().__init__('go_driver_node')
        qos_profile = QoSProfile(depth=10)

        self.joint_pub = []
        self.go2_lidar_pub = []
        self.odom_pub = []
        self.imu_pub = []
        self.twist_pub = []

        self.joint_pub.append(self.create_publisher(JointState, f'/joint_states', qos_profile))
        self.go2_lidar_pub.append(self.create_publisher(PointCloud2, '/sensing/lidar/top/points', qos_profile))
        self.imu_pub.append(self.create_publisher(Imu, '/sensing/imu/imu_data', qos_profile))
        self.twist_pub.append(self.create_publisher(TwistWithCovarianceStamped, '/sensing/vehicle_velocity_converter/twist_with_covariance', qos_profile)) 
        self.broadcaster= TransformBroadcaster(self, qos=qos_profile)


    def publish_joints(self, joint_names_lst, joint_state_lst):
        # Create message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        joint_state_names_formated = []
        for joint_name in joint_names_lst:
            joint_state_names_formated.append(joint_name)

        joint_state_formated = []
        for joint_state_val in joint_state_lst:
            joint_state_formated.append(joint_state_val.item())

        joint_state.name = joint_state_names_formated
        joint_state.position = joint_state_formated
        self.joint_pub[0].publish(joint_state)


    def publish_tf(self, base_pos, base_rot):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = "map"
        odom_trans.child_frame_id = "base_link_ground_truth"
        odom_trans.transform.translation.x = base_pos[0].item()# + 0.25 #offset of actor frame to center of mass
        odom_trans.transform.translation.y = base_pos[1].item()# - 0.1
        odom_trans.transform.translation.z = base_pos[2].item()# - 0.15
        odom_trans.transform.rotation.x = base_rot[1].item()
        odom_trans.transform.rotation.y = base_rot[2].item()
        odom_trans.transform.rotation.z = base_rot[3].item()
        odom_trans.transform.rotation.w = base_rot[0].item()
        self.broadcaster.sendTransform(odom_trans)


    def publish_twist(self, lin_vel, ang_vel):
        twist_topic = TwistWithCovarianceStamped()
        twist_topic.header.stamp = self.get_clock().now().to_msg()
        twist_topic.header.frame_id = "base_link"
        twist_topic.twist.twist.linear.x = lin_vel[0].item()
        twist_topic.twist.twist.linear.y = lin_vel[1].item()
        twist_topic.twist.twist.linear.z = lin_vel[2].item()
        twist_topic.twist.twist.angular.x = ang_vel[0].item()
        twist_topic.twist.twist.angular.y = ang_vel[1].item()
        twist_topic.twist.twist.angular.z = ang_vel[2].item()

        cov = np.zeros((6, 6), dtype=np.float64)
        np.fill_diagonal(cov, [0.651, 0.2979, 1.087, 21, 19.88, 0.615]) # best: [0.05, 0.05, 0.5, 5, 5, 5]
        twist_topic.twist.covariance = cov.flatten()
        self.twist_pub[0].publish(twist_topic)
        

    def publish_imu(self, base_rot, base_lin_acc, base_ang_vel):
        imu_trans = Imu()
        imu_trans.header.stamp = self.get_clock().now().to_msg()
        imu_trans.header.frame_id = "base_link"
        imu_trans.linear_acceleration.x = base_lin_acc[0].item()/10
        imu_trans.linear_acceleration.y = base_lin_acc[1].item()/10
        imu_trans.linear_acceleration.z = base_lin_acc[2].item()/10
        imu_trans.angular_velocity.x = base_ang_vel[0].item()
        imu_trans.angular_velocity.y = base_ang_vel[1].item()
        imu_trans.angular_velocity.z = base_ang_vel[2].item()
        imu_trans.orientation.x = base_rot[1].item()
        imu_trans.orientation.y = base_rot[2].item()
        imu_trans.orientation.z = base_rot[3].item()
        imu_trans.orientation.w = base_rot[0].item()

        cov = np.zeros((3, 3), dtype=np.float64)
        np.fill_diagonal(cov, [108,	20,	420]) # best: [25, 25, 50]
        imu_trans.linear_acceleration_covariance = cov.flatten()
        np.fill_diagonal(cov, [2.1,	1.99, 0.0615]) # best: [5, 5, 5]
        imu_trans.angular_velocity_covariance = cov.flatten()
        np.fill_diagonal(cov, [0.0491, 0.02239, 0.009925]) # best: [0.05, 0.05, 10]
        imu_trans.orientation_covariance = cov.flatten()
        self.imu_pub[0].publish(imu_trans)


    def publish_lidar(self, points):
        point_cloud = PointCloud2()
        point_cloud.header = Header(frame_id="lidar_top")
        point_cloud.header.stamp = self.get_clock().now().to_msg()        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),]
        point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
        self.go2_lidar_pub[0].publish(point_cloud)


    async def run(self):
        while True:
            self.publish_lidar()
            await asyncio.sleep(0.1)