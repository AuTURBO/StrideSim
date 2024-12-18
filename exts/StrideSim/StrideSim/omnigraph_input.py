import numpy as np
import threading
import time

import carb  # noqa: F401
import omni.graph.core as og


class ROS2OmniInput:
    def __init__(self, prim_paths: dict):
        super().__init__()
        self._prim_paths = self.initialize_prim_paths(prim_paths)
        (self._omni_graph, _, _, _) = og.Controller.edit(
            {"graph_path": "/action_graph_input", "evaluator_name": "execution"}, {}
        )
        self._omni_controller = og.Controller(self._omni_graph)

        self._keys = og.Controller.Keys
        self.initialize_omnigraph()

        self._angular_velocity_path = "/action_graph_input/Ros_Twist_sub.outputs:angularVelocity"
        self._linear_velocity_path = "/action_graph_input/Ros_Twist_sub.outputs:linearVelocity"
        self._linear_vel = np.zeros(3)
        self._angular_vel = np.zeros(3)

        self._graph_monitor_thread = threading.Thread(target=self.setup_graph_monitoring)
        self._graph_monitor_thread.start()

    def initialize_prim_paths(self, prim_paths: dict):
        default_paths = {
            "namespace": "AnymalD",
        }
        for key, default in default_paths.items():
            if key not in prim_paths:
                prim_paths[key] = default
        return prim_paths

    def initialize_omnigraph(self):
        self.create_general_nodes()
        self.create_ros_nodes()
        self.set_general_node_values()
        self.set_ros_node_values()
        self.connect_general_nodes()
        self.connect_ros_nodes()

    def create_general_nodes(self):
        nodes = [
            ("PTick", "omni.graph.action.OnPlaybackTick"),
            ("Namespace_string", "omni.graph.nodes.ConstantString"),
        ]
        self.create_nodes(nodes)

    def create_ros_nodes(self):
        nodes = [
            ("Ros_Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("Ros_Twist_sub", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
        ]
        self.create_nodes(nodes)

    def create_nodes(self, nodes):
        self._omni_controller.edit(
            self._omni_graph,
            {self._keys.CREATE_NODES: nodes},
        )

    def set_general_node_values(self):
        values = [
            ("Namespace_string.inputs:value", self._prim_paths["namespace"]),
        ]
        self.set_node_values(values)

    def set_ros_node_values(self):
        values = [
            ("Ros_Context.inputs:useDomainIDEnvVar", True),
            ("Namespace_string.inputs:value", self._prim_paths["namespace"]),
        ]
        self.set_node_values(values)

    def set_node_values(self, values):
        self._omni_controller.edit(
            self._omni_graph,
            {self._keys.SET_VALUES: values},
        )

    def connect_general_nodes(self):
        connections = [
            ("PTick.outputs:tick", "Ros_Twist_sub.inputs:execIn"),
        ]
        self.connect_nodes(connections)

    def connect_ros_nodes(self):
        connections = [
            ("Ros_Context.outputs:context", "Ros_Twist_sub.inputs:context"),
            ("Namespace_string.inputs:value", "Ros_Twist_sub.inputs:nodeNamespace"),
        ]
        self.connect_nodes(connections)

    def connect_nodes(self, connections):
        self._omni_controller.edit(
            self._omni_graph,
            {self._keys.CONNECT: connections},
        )

    def get_linear_velocity(self):
        return self._linear_vel

    def get_angular_velocity(self):
        return self._angular_vel

    def setup_graph_monitoring(self):

        while True:
            self._linear_vel = og.Controller.get(self._linear_velocity_path)
            self._angular_vel = og.Controller.get(self._angular_velocity_path)

            time.sleep(0.02)  # 50Hz = 0.02s period
