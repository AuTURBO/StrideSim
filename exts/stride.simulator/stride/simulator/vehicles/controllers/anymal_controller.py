import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path

from stride.simulator.vehicles.controllers.controller import Controller
from stride.simulator.vehicles.controllers.networks.actuator_network import (
    LstmSeaNetwork,
)

import io
import numpy as np
import torch

# TODO: state를 공유해야한다. 아니면 input으로 받아야함 state는 로봇 Vehicle에서 관리하고 있음 e.g) quadrupedrobot.state


class AnyamlController(Controller):
    """
    AnyamlController class - It defines a base interface for creating a AnyamlController

    Args:
        Controller: The base class for all controllers.
    """

    def __init__(self):
        super().__init__()

        assets_root_path = get_assets_root_path()

        # Policy
        file_content = omni.client.read_file(
            assets_root_path + "/Isaac/Samples/Quadruped/Anymal_Policies/policy_1.pt"
        )[2]
        file = io.BytesIO(memoryview(file_content).tobytes())

        self._policy = torch.jit.load(file)

        self._action_scale = 0.5
        self.base_vel_lin_scale = 2.0
        self.base_vel_ang_scale = 0.25
        self.joint_pos_scale = 1.0
        self.joint_vel_scale = 0.05
        self.default_joint_pos = np.array(
            [0.0, 0.4, -0.8, 0.0, -0.4, 0.8, -0.0, 0.4, -0.8, -0.0, -0.4, 0.8]
        )
        self.previous_action = np.zeros(12)
        self._policy_counter = 0

        # Actuator network
        file_content = omni.client.read_file(
            assets_root_path
            + "/Isaac/Samples/Quadruped/Anymal_Policies/sea_net_jit2.pt"
        )[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self._actuator_network = LstmSeaNetwork()
        self._actuator_network.setup(file, self.default_joint_pos)
        self._actuator_network.reset()
        self._state = {}  # FIXME: change this variable to state class..

    @property
    def state(self):
        return self._state

    def update_state(self, state):
        self._state = state
        pass

    def advance(self, dt, obs, command):
        """[summary]

        compute the desired torques and apply them to the articulation

        Argument:
        dt {float} -- Timestep update in the world.
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        """
        if self._policy_counter % 4 == 0:
            used_obs = obs
            with torch.no_grad():
                used_obs = torch.from_numpy(used_obs).view(1, -1).float()
                self.action = self._policy(used_obs).detach().view(-1).numpy()
            self.previous_action = self.action.copy()

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

        current_joint_pos = self.state.joint_angles
        current_joint_vel = self.state.joint_velocities
        current_joint_pos = np.array(current_joint_pos.reshape([3, 4]).T.flat)
        current_joint_vel = np.array(current_joint_vel.reshape([3, 4]).T.flat)
        joint_torques, _ = self._actuator_network.compute_torques(
            current_joint_pos, current_joint_vel, self._action_scale * self.action
        )

        self._policy_counter += 1

        return joint_torques
