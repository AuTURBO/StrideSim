import omni.graph.core as og


class ROS2OmniOutput():
    def __init__(self, prim_paths: dict):
        super().__init__()
        self._prim_paths = self.initialize_prim_paths(prim_paths)
        (self._omni_graph, _, _, _) = og.Controller.edit(
            {"graph_path": "/action_graph_output", "evaluator_name": "execution"}, {}
        )
        self._omni_controller = og.Controller(self._omni_graph)

        self._keys = og.Controller.Keys
        self.initialize_omnigraph()

    def initialize_prim_paths(self, prim_paths: dict):
        default_paths = {
            "Lidar_path": "/World/AnymalD/AnymalD/base/lidar_parent/Lidar",
            "Imu_path": "/World/AnymalD/AnymalD/base/imu_link/Imu_Sensor",
            "TF_path": "/World/AnymalD/AnymalD/base",
            "Joint_states_path": "/World/AnymalD/AnymalD/base",
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
            ("Sim_Time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            ("Namespace_string", "omni.graph.nodes.ConstantString"),
        ]
        self.create_nodes(nodes)

    def create_ros_nodes(self):
        nodes = [
            ("Ros_Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("Imu_Read", "omni.isaac.sensor.IsaacReadIMU"),
            ("Ros2_Imu_Pub", "omni.isaac.ros2_bridge.ROS2PublishImu"),
            ("Lidar_Read", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
            ("Ros2_Lidar_Pub", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
            ("Ros2_tf_pub", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ("Ros2_Joint_states_pub", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
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
            ("Imu_Read.inputs:imuPrim", self._prim_paths["Imu_path"]),
            ("Lidar_Read.inputs:lidarPrim", self._prim_paths["Lidar_path"]),
            ("Imu_Read.inputs:readGravity", True),
            ("Ros2_Imu_Pub.inputs:frameId", "imu_link"),
            ("Ros2_Imu_Pub.inputs:topicName", "imu"),
            ("Ros2_Lidar_Pub.inputs:frameId", "lidar_parent"),
            ("Ros2_Lidar_Pub.inputs:topicName", "point_cloud"),
            ("Ros2_tf_pub.inputs:targetPrims", self._prim_paths["TF_path"]),
            (
                "Ros2_Joint_states_pub.inputs:targetPrim",
                self._prim_paths["Joint_states_path"],
            ),
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
            ("PTick.outputs:tick", "Imu_Read.inputs:execIn"),
            ("PTick.outputs:tick", "Lidar_Read.inputs:execIn"),
            ("PTick.outputs:tick", "Ros2_tf_pub.inputs:execIn"),
            ("PTick.outputs:tick", "Ros2_Joint_states_pub.inputs:execIn"),
            ("Sim_Time.outputs:simulationTime", "Ros2_Imu_Pub.inputs:timeStamp"),
            ("Sim_Time.outputs:simulationTime", "Ros2_Lidar_Pub.inputs:timeStamp"),
            ("Sim_Time.outputs:simulationTime", "Ros2_tf_pub.inputs:timeStamp"),
            ("Sim_Time.outputs:simulationTime", "Ros2_Joint_states_pub.inputs:timeStamp"),
        ]
        self.connect_nodes(connections)

    def connect_ros_nodes(self):
        connections = [
            ("Ros_Context.outputs:context", "Ros2_Imu_Pub.inputs:context"),
            ("Imu_Read.outputs:execOut", "Ros2_Imu_Pub.inputs:execIn"),
            ("Imu_Read.outputs:angVel", "Ros2_Imu_Pub.inputs:angularVelocity"),
            ("Imu_Read.outputs:linAcc", "Ros2_Imu_Pub.inputs:linearAcceleration"),
            ("Imu_Read.outputs:orientation", "Ros2_Imu_Pub.inputs:orientation"),
            ("Ros_Context.outputs:context", "Ros2_Lidar_Pub.inputs:context"),
            ("Lidar_Read.outputs:execOut", "Ros2_Lidar_Pub.inputs:execIn"),
            ("Lidar_Read.outputs:data", "Ros2_Lidar_Pub.inputs:data"),
            ("Ros_Context.outputs:context", "Ros2_tf_pub.inputs:context"),
            ("Namespace_string.inputs:value", "Ros2_Imu_Pub.inputs:nodeNamespace"),
            ("Namespace_string.inputs:value", "Ros2_Lidar_Pub.inputs:nodeNamespace"),
            ("Ros_Context.outputs:context", "Ros2_Joint_states_pub.inputs:context"),
        ]
        self.connect_nodes(connections)

    def connect_nodes(self, connections):
        self._omni_controller.edit(
            self._omni_graph,
            {self._keys.CONNECT: connections},
        )
