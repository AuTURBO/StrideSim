# python
from typing import Union, Tuple
import numpy as np
from numpy import genfromtxt
import torch


class LstmSeaNetwork:
    """Implements an SEA network with LSTM hidden layers."""

    def __init__(self):
        # define the network
        self._network = None
        self._hidden_state = torch.zeros((2, 12, 8), requires_grad=False)
        self._cell_state = torch.zeros((2, 12, 8), requires_grad=False)
        # default joint position
        self._default_joint_pos = None

    """
    Properties
    """

    def get_hidden_state(self) -> np.ndarray:
        if self._hidden_state is None:
            return np.zeros((12, 8))
        else:
            return self._hidden_state[1].detach().numpy()

    """
    Operations
    """

    def setup(self, path_or_buffer, default_joint_pos: Union[list, np.ndarray]):
        # load the network from JIT file
        self._network = torch.jit.load(path_or_buffer)
        # set the default joint position
        self._default_joint_pos = np.asarray(default_joint_pos)

    def reset(self):
        # reset the hidden state of LSTM
        with torch.no_grad():
            self._hidden_state[:, :, :] = 0.0
            self._cell_state[:, :, :] = 0.0

    @torch.no_grad()
    def compute_torques(self, joint_pos, joint_vel, actions) -> Tuple[np.ndarray, np.ndarray]:
        # create sea network input obs
        actions = actions.copy()
        actuator_net_input = torch.zeros((12, 1, 2))
        actuator_net_input[:, 0, 0] = torch.from_numpy(actions + self._default_joint_pos - joint_pos)
        actuator_net_input[:, 0, 1] = torch.from_numpy(np.clip(joint_vel, -20.0, 20))
        # call the network
        torques, (self._hidden_state, self._cell_state) = self._network(
            actuator_net_input, (self._hidden_state, self._cell_state)
        )
        # return the torque to apply with clipping along with hidden state
        return torques.detach().clip(-80.0, 80.0).numpy(), self._hidden_state[1].numpy()


class SeaNetwork(torch.nn.Module):
    """Implements a SEA network with MLP hidden layers."""

    def __init__(self):
        super().__init__()
        # define layer architecture
        self._sea_network = torch.nn.Sequential(
            torch.nn.Linear(6, 32),
            torch.nn.Softsign(),
            torch.nn.Linear(32, 32),
            torch.nn.Softsign(),
            torch.nn.Linear(32, 1),
        )
        # define the delays
        self._num_delays = 2
        self._delays = [8, 3]
        # define joint histories
        self._history_size = self._delays[0]
        self._joint_pos_history = np.zeros((12, self._history_size + 1))
        self._joint_vel_history = np.zeros((12, self._history_size + 1))
        # define scaling for the actuator network
        self._sea_vel_scale = 0.4
        self._sea_pos_scale = 3.0
        self._sea_output_scale = 20.0
        self._action_scale = 0.5
        # default joint position
        self._default_joint_pos = None

    """
    Operations
    """

    def setup(self, weights_path: str, default_joint_pos: Union[list, np.ndarray]):
        # load the weights into network
        self._load_weights(weights_path)
        # set the default joint position
        self._default_joint_pos = np.asarray(default_joint_pos)

    def reset(self):
        self._joint_pos_history.fill(0.0)
        self._joint_vel_history.fill(0.0)

    def compute_torques(self, joint_pos, joint_vel, actions) -> np.ndarray:
        self._update_joint_history(joint_pos, joint_vel, actions)
        return self._compute_sea_torque()

    """
    Internal helpers.
    """

    def _load_weights(self, weights_path: str):
        # load the data
        data = genfromtxt(weights_path, delimiter=",")
        # manually defines the number of neurons in MLP
        expected_num_params = 6 * 32 + 32 + 32 * 32 + 32 + 32 * 1 + 1
        assert data.size == expected_num_params
        # assign neuron weights to each linear layer
        idx = 0
        for layer in self._sea_network:
            if not isinstance(layer, torch.nn.Softsign):
                # layer weights
                weight = np.reshape(
                    data[idx : idx + layer.in_features * layer.out_features],
                    newshape=(layer.in_features, layer.out_features),
                ).T
                layer.weight = torch.nn.Parameter(torch.from_numpy(weight.astype(np.float32)))
                idx += layer.out_features * layer.in_features
                # layer biases
                bias = data[idx : idx + layer.out_features]
                layer.bias = torch.nn.Parameter(torch.from_numpy(bias.astype(np.float32)))
                idx += layer.out_features
        # set the module in eval mode
        self.eval()

    def _update_joint_history(self, joint_pos, joint_vel, actions):
        # convert to numpy (sanity)
        joint_pos = np.asarray(joint_pos)
        joint_vel = np.asarray(joint_vel)
        # compute error in position
        joint_pos_error = self._action_scale * actions + self._default_joint_pos - joint_pos
        # store into history
        self._joint_pos_history[:, : self._history_size] = self._joint_pos_history[:, 1:]
        self._joint_vel_history[:, : self._history_size] = self._joint_vel_history[:, 1:]
        self._joint_pos_history[:, self._history_size] = joint_pos_error
        self._joint_vel_history[:, self._history_size] = joint_vel

    def _compute_sea_torque(self):
        inp = torch.zeros((12, 6))
        for dof in range(12):
            inp[dof, 0] = self._sea_vel_scale * self._joint_vel_history[dof, self._history_size - self._delays[0]]
            inp[dof, 1] = self._sea_vel_scale * self._joint_vel_history[dof, self._history_size - self._delays[1]]
            inp[dof, 2] = self._sea_vel_scale * self._joint_vel_history[dof, self._history_size]
            inp[dof, 3] = self._sea_pos_scale * self._joint_pos_history[dof, self._history_size - self._delays[0]]
            inp[dof, 4] = self._sea_pos_scale * self._joint_pos_history[dof, self._history_size - self._delays[1]]
            inp[dof, 5] = self._sea_pos_scale * self._joint_pos_history[dof, self._history_size]
        return self._sea_output_scale * self._sea_network(inp)


# EOF
