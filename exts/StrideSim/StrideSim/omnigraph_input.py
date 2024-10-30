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


# TODO: 여기서부터 다시 작업~
# def setup_graph_monitoring():
#     # Get a handle to the graph
#     keys = og.Controller.Keys
#     (graph_handle, _, _, _) = og.Controller.edit({"graph_path": "/action_graph_input"})

#     # Access the output attribute
#     output_attr = og.Controller.attribute("/action_graph_input/Ros_Twist_sub.outputs:angularVelocity")

#     # Define callback
#     def on_value_change(attr):
#         new_value = attr.get()
#         carb.log_info(f"Output value updated: {new_value}")

#     # Register callback
#     output_attr.add_value_changed_fn(on_value_change)

#     # Get the initial value
#     initial_value = output_attr.get()
#     carb.log_info(f"Initial value: {initial_value}")
