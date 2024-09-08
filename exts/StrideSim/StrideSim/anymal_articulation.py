# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import io
from typing import List, Optional

import carb
import numpy as np
import omni
import omni.kit.commands
import torch
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.rotations import euler_to_rot_matrix, quat_to_euler_angles, quat_to_rot_matrix
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.quadruped.utils import LstmSeaNetwork
from pxr import Gf


class AnymalD_Atriculation(Articulation):
    """The ANYmal quadruped"""

    def __init__(
        self,
        prim_path: str,
        name: str = "anymal",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        [Summary]

        initialize robot, set up sensors and controller

        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            usd_path {str} -- robot usd filepath in the directory
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot

        """
        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)

        assets_root_path = get_assets_root_path()
        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")

                asset_path = assets_root_path + "/Isaac/Robots/ANYbotics/anymal_c.usd"

                carb.log_warn("asset path is: " + asset_path)
                prim.GetReferences().AddReference(asset_path)

        super().__init__(prim_path=self._prim_path, name=name, position=position, orientation=orientation)

        self._dof_control_modes: List[int] = list()

        # Policy
        file_content = omni.client.read_file(assets_root_path + "/Isaac/Samples/Quadruped/Anymal_Policies/policy_1.pt")[
            2
        ]
        file = io.BytesIO(memoryview(file_content).tobytes())

        self._policy = torch.jit.load(file)
        self._base_vel_lin_scale = 2.0
        self._base_vel_ang_scale = 0.25
        self._joint_pos_scale = 1.0
        self._joint_vel_scale = 0.05
        self._action_scale = 0.5
        self._default_joint_pos = np.array([0.0, 0.4, -0.8, 0.0, -0.4, 0.8, -0.0, 0.4, -0.8, -0.0, -0.4, 0.8])
        self._previous_action = np.zeros(12)
        self._policy_counter = 0

        # Actuator network
        file_content = omni.client.read_file(
            assets_root_path + "/Isaac/Samples/Quadruped/Anymal_Policies/sea_net_jit2.pt"
        )[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self._actuator_network = LstmSeaNetwork()
        self._actuator_network.setup(file, self._default_joint_pos)
        self._actuator_network.reset()

        # Height scaner
        y = np.arange(-0.5, 0.6, 0.1)
        x = np.arange(-0.8, 0.9, 0.1)
        grid_x, grid_y = np.meshgrid(x, y)
        self._scan_points = np.zeros((grid_x.size, 3))
        self._scan_points[:, 0] = grid_x.transpose().flatten()
        self._scan_points[:, 1] = grid_y.transpose().flatten()
        self.physx_query_interface = omni.physx.get_physx_scene_query_interface()
        self._query_info = []

    def _hit_report_callback(self, hit):
        current_hit_body = hit.rigid_body
        if "/World/GroundPlane" in current_hit_body:
            self._query_info.append(hit.distance)
        return True

    def _compute_observation(self, command):
        """[summary]

        compute the observation vector for the policy

        Argument:
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.get_linear_velocity()
        ang_vel_I = self.get_angular_velocity()
        pos_IB, q_IB = self.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(235)
        # Base lin vel
        obs[:3] = self._base_vel_lin_scale * lin_vel_b
        # Base ang vel
        obs[3:6] = self._base_vel_ang_scale * ang_vel_b
        # Gravity
        obs[6:9] = gravity_b
        # Command
        obs[9] = self._base_vel_lin_scale * command[0]
        obs[10] = self._base_vel_lin_scale * command[1]
        obs[11] = self._base_vel_ang_scale * command[2]
        # Joint states
        # joint_state from the DC interface now has the order of
        # 'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
        # 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        # 'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'

        # while the learning controller uses the order of
        # FL_hip_joint FL_thigh_joint FL_calf_joint
        # FR_hip_joint FR_thigh_joint FR_calf_joint
        # RL_hip_joint RL_thigh_joint RL_calf_joint
        # RR_hip_joint RR_thigh_joint RR_calf_joint
        # Convert DC order to controller order for joint info
        current_joint_pos = self.get_joint_positions()
        current_joint_vel = self.get_joint_velocities()
        current_joint_pos = np.array(current_joint_pos.reshape([3, 4]).T.flat)
        current_joint_vel = np.array(current_joint_vel.reshape([3, 4]).T.flat)
        obs[12:24] = self._joint_pos_scale * (current_joint_pos - self._default_joint_pos)
        obs[24:36] = self._joint_vel_scale * current_joint_vel

        obs[36:48] = self._previous_action

        # height_scanner
        rpy = -quat_to_euler_angles(q_IB)
        rpy[:2] = 0.0
        yaw_rot = np.transpose(euler_to_rot_matrix(rpy))

        world_scan_points = np.matmul(yaw_rot, self._scan_points.T).T + pos_IB

        for i in range(world_scan_points.shape[0]):
            self._query_info.clear()
            self.physx_query_interface.raycast_all(
                tuple(world_scan_points[i]), (0.0, 0.0, -1.0), 100, self._hit_report_callback
            )
            if self._query_info:
                distance = min(self._query_info)
                obs[48 + i] = np.clip(distance - 0.5, -1.0, 1.0)
            else:
                print("No hit")
        return obs

    def advance(self, dt, command):
        """[summary]

        compute the desired torques and apply them to the articulation

        Argument:
        dt {float} -- Timestep update in the world.
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % 4 == 0:
            obs = self._compute_observation(command)
            with torch.no_grad():
                obs = torch.from_numpy(obs).view(1, -1).float()
                self.action = self._policy(obs).detach().view(-1).numpy()
            self._previous_action = self.action.copy()

        # joint_state from the DC interface now has the order of
        # 'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
        # 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        # 'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'

        # while the learning controller uses the order of
        # FL_hip_joint FL_thigh_joint FL_calf_joint
        # FR_hip_joint FR_thigh_joint FR_calf_joint
        # RL_hip_joint RL_thigh_joint RL_calf_joint
        # RR_hip_joint RR_thigh_joint RR_calf_joint
        # Convert DC order to controller order for joint info
        current_joint_pos = self.get_joint_positions()
        current_joint_vel = self.get_joint_velocities()
        current_joint_pos = np.array(current_joint_pos.reshape([3, 4]).T.flat)
        current_joint_vel = np.array(current_joint_vel.reshape([3, 4]).T.flat)
        joint_torques, _ = self._actuator_network.compute_torques(
            current_joint_pos, current_joint_vel, self._action_scale * self.action
        )

        # finally convert controller order to DC order for command torque
        torque_reorder = np.array(joint_torques.reshape([4, 3]).T.flat)
        self.set_joint_efforts(torque_reorder)

        self._policy_counter += 1

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize the articulation interface, set up drive mode
        """
        super().initialize(physics_sim_view=physics_sim_view)
        self.get_articulation_controller().set_effort_modes("force")
        self.get_articulation_controller().switch_control_mode("effort")

    def post_reset(self) -> None:
        """[summary]

        post reset articulation
        """
        super().post_reset()
