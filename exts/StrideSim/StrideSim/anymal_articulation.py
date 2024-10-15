import io
import numpy as np
import sys
import torch
from typing import List, Optional

import carb
import omni
import omni.kit.commands
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.rotations import euler_to_rot_matrix, quat_to_euler_angles, quat_to_rot_matrix
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path

from StrideSim.settings import ISAACLAB_LAB, ISAACLAB_LAB_ASSETS, ISAACLAB_LAB_TASKS

# from pxr import Gf


sys.path.append(ISAACLAB_LAB)
sys.path.append(ISAACLAB_LAB_TASKS)
sys.path.append(ISAACLAB_LAB_ASSETS)

from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR


class AnymalD_Atriculation:
    """The AnymalD quadruped"""

    def __init__(
        self,
        prim_path: str,
        name: str = "anymalD",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize robot and load RL policy.

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

                asset_path = f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-D/anymal_d.usd"

                prim.GetReferences().AddReference(asset_path)

        self.robot = Articulation(
            prim_path=self._prim_path,
            name=name,
            position=position,
            orientation=orientation,
        )

        self._dof_control_modes: List[int] = list()

        # print("assets_root_path: ", assets_root_path)

        # Policy
        file_content = omni.client.read_file(
            "/home/jin/Documents/StrideSim/exts/StrideSim/StrideSim/network/policy.pt"
        )[2]
        file = io.BytesIO(memoryview(file_content).tobytes())

        self._policy = torch.jit.load(file)
        self._base_vel_lin_scale = 1
        self._base_vel_ang_scale = 1
        self._action_scale = 0.5
        self._default_joint_pos = np.array(
            # [0.0, 0.4, -0.8, 0.0, -0.4, 0.8, -0.0, 0.4, -0.8, -0.0, -0.4, 0.8]
            [0.0, 0.0, 0.0, 0.0, 0.4, -0.4, 0.4, -0.4, -0.8, 0.8, -0.8, 0.8]
        )
        self._previous_action = np.zeros(12)
        self._policy_counter = 0
        self._decimation = 4

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
        """
        Compute the observation vector for the policy

        Argument:
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()

        print("lin_vel_I: ", lin_vel_I)
        print("ang_vel_I: ", ang_vel_I)
        print("pos_IB: ", pos_IB)
        print("q_IB: ", q_IB)
        print("R_IB: ", R_IB)
        print("R_BI: ", R_BI)

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
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[12:24] = current_joint_pos - self._default_joint_pos
        obs[24:36] = current_joint_vel

        # Previous Action
        obs[36:48] = self._previous_action

        # height_scanner
        rpy = -quat_to_euler_angles(q_IB)
        rpy[:2] = 0.0
        yaw_rot = np.transpose(euler_to_rot_matrix(rpy))

        world_scan_points = np.matmul(yaw_rot, self._scan_points.T).T + pos_IB

        for i in range(world_scan_points.shape[0]):
            self._query_info.clear()
            self.physx_query_interface.raycast_all(
                tuple(world_scan_points[i]),
                (0.0, 0.0, -1.0),
                100,
                self._hit_report_callback,
            )
            if self._query_info:
                distance = min(self._query_info)
                obs[48 + i] = np.clip(distance - 0.5, -1.0, 1.0)
            else:
                pass
                # print("No hit")
        return obs

    def advance(self, dt, command):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt {float} -- Timestep update in the world.
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            with torch.no_grad():
                obs = torch.from_numpy(obs).view(1, -1).float()
                self.action = self._policy(obs).detach().view(-1).numpy()
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self._default_joint_pos + (self.action * self._action_scale))

        print("network action: ", self.action)

        self.robot.apply_action(action)

        self._policy_counter += 1

    def initialize(self, physics_sim_view=None) -> None:
        """
        Initialize robot the articulation interface, set up drive mode
        """
        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes("force")
        self.robot.get_articulation_controller().switch_control_mode("position")
        self.robot._articulation_view.set_gains(np.zeros(12) + 200, np.zeros(12) + 5)

    def post_reset(self) -> None:
        """
        Post reset articulation
        """
        self.robot.post_reset()
