
import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path

from stride.simulator.vehicles.controllers.controller import Controller
from stride.simulator.vehicles.controllers.networks.actuator_network import LstmSeaNetwork

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

        self._dc_interface.wake_up_articulation(self._handle)

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
            current_joint_pos, current_joint_vel, self._action_scale * self.action)

        # finally convert controller order to DC order for command torque
        torque_reorder = np.array(joint_torques.reshape([4, 3]).T.flat)
        self._dc_interface.set_articulation_dof_efforts(self._handle, torque_reorder)

        self._policy_counter += 1
