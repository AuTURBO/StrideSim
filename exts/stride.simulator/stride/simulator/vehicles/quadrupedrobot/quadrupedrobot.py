# The vehicle interface
from stride.simulator.vehicles.vehicle import Vehicle
from stride.simulator.interfaces.stride_sim_interface import StrideInterface

import omni
from omni.isaac.core.utils.rotations import quat_to_rot_matrix, quat_to_euler_angles, euler_to_rot_matrix
from pxr import Gf
import numpy as np
import asyncio


class QuadrupedRobotConfig:
    """
    A data class that is used for configuring a quadrupedrobot
    """

    def __init__(self):
        """
        Initialization of the QuadrupedRobotConfig class
        """

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "quadrupedrobot"

        # The USD file that describes the visual aspect of the vehicle
        # (and some properties such as mass and moments of inertia)
        self.usd_file = ""

        # The default sensors for a quadrotor
        self.sensors = []

        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle].
        # It can also be a ROS2 backend or your own custom Backend implementation!
        self.backends = []

        self.controller = None


class QuadrupedRobot(Vehicle):
    """QuadrupedRobot class - It defines a base interface for creating a QuadrupedRobot
    """

    def __init__(  # pylint: disable=dangerous-default-value FIXME
            self,
            # Simulation specific configurations
            stage_prefix: str = "quadrupedrobot",
            usd_file: str = "",
            vehicle_id: int = 0,
            # Spawning pose of the vehicle
            init_pos=[0.0, 0.0, 0.07],
            init_orientation=[0.0, 0.0, 0.0, 1.0],
            config=QuadrupedRobotConfig(),
    ):
        """Initializes the quadrupedrobot object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned.
                                Defaults to "quadrupedrobot".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention).
                                Defaults to [0.0, 0.0, 0.07].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw].
                                    Defaults to [0.0, 0.0, 0.0, 1.0].
            config (_type_, optional): _description_. Defaults to QuadrupedRobotConfig().
        """

        # 1. Initiate the vehicle object itself
        super().__init__(stage_prefix, usd_file, init_pos, init_orientation)

        # 2. Initialize all the vehicle sensors
        self._sensors = config.sensors
        for sensor in self._sensors:
            sensor.set_spherical_coordinate(StrideInterface().latitude,
                                            StrideInterface().longitude,
                                            StrideInterface().altitude)
            pass

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # 4. Save the backend interface (if given in the configuration of the vehicle)
        # and initialize them
        self._backends = config.backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callback to the physics engine to update the state of the vehicle at every timestep.
        self._world.add_physics_callback(self._stage_prefix + "/sim_state", self.update_sim_state)

        # Height scaner
        y = np.arange(-0.5, 0.6, 0.1)
        x = np.arange(-0.8, 0.9, 0.1)
        grid_x, grid_y = np.meshgrid(x, y)
        self._scan_points = np.zeros((grid_x.size, 3))
        self._scan_points[:, 0] = grid_x.transpose().flatten()
        self._scan_points[:, 1] = grid_y.transpose().flatten()
        self.physx_query_interface = omni.physx.get_physx_scene_query_interface()
        self._query_info = []

    def update_sensors(self, dt: float):
        """Callback that is called at every physics steps and will call the sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_sensor method will also be called for
        every backend. For example, if new data is generated for an IMU and we have a MavlinkBackend,
        then the update_sensor method will be called for that backend
        so that this data can latter be sent thorugh mavlink.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)

            # If some data was updated and we have a mavlink backend or ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float):
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle.
        This callback is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        for backend in self._backends:
            backend.update_state(self._state)

    def start(self):
        """
        Initializes the communication with all the backends.
        This method is invoked automatically when the simulation starts
        """
        for backend in self._backends:
            backend.start()

    def stop(self):
        """
        Signal all the backends that the simulation has stopped.
        This method is invoked automatically when the simulation stops
        """
        for backend in self._backends:
            backend.stop()

        async def reset_async():
            await self._world.reset_async()
            await self._world.pause_async()

        asyncio.ensure_future(reset_async())

    def update(self, dt: float):
        """
        Method that computes and applies the torques to the vehicle in simulation based on the base body speed.
        This method must be implemented by a class that inherits this type. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the desired base velocity for robot from the first backend (can be mavlink or other) expressed in m/s
        if len(self._backends) != 0:
            command = self._backends[0].input_reference()
        else:
            command = [0.0 for i in range(3)]

        command = np.array([1.0, 0.0, 0.0])

        obs = self._compute_observation(command)

        self.controller.update_state(self.state)

        torque = self.controller.advance(dt, obs, command)

        self.apply_torque(torque)

        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)

    def apply_torque(self, torque):

        self._dc_interface.wake_up_articulation(self._handle)

        torque_reorder = torque.reshape([4, 3]).T.flat

        self._dc_interface.set_articulation_dof_efforts(self._handle, torque_reorder)

    def _compute_observation(self, command):
        """[summary]
        
        compute the observation vector for the policy
        
        Argument:
        command {np.ndarray} -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.state.linear_velocity  #pylint: disable=invalid-name
        ang_vel_I = self.state.angular_velocity  #pylint: disable=invalid-name
        pos_IB = self.state.position  #pylint: disable=invalid-name
        # Convert quaternion from XYZW to WXYZ format
        q_IB = np.array(  #pylint: disable=invalid-name
            [self.state.attitude[-1], self.state.attitude[0], self.state.attitude[1], self.state.attitude[2]])

        R_IB = quat_to_rot_matrix(q_IB)  #pylint: disable=invalid-name
        R_BI = R_IB.transpose()  #pylint: disable=invalid-name
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(235)
        # Base lin vel
        obs[:3] = self.controller.base_vel_lin_scale * lin_vel_b
        # Base ang vel
        obs[3:6] = self.controller.base_vel_ang_scale * ang_vel_b
        # Gravity
        obs[6:9] = gravity_b
        # Command
        obs[9] = self.controller.base_vel_lin_scale * command[0]
        obs[10] = self.controller.base_vel_lin_scale * command[1]
        obs[11] = self.controller.base_vel_ang_scale * command[2]
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
        current_joint_pos = self.state.joint_angles
        current_joint_vel = self.state.joint_velocities
        current_joint_pos = np.array(current_joint_pos.reshape([3, 4]).T.flat)
        current_joint_vel = np.array(current_joint_vel.reshape([3, 4]).T.flat)
        obs[12:24] = self.controller.joint_pos_scale * (current_joint_pos - self.controller.default_joint_pos)
        obs[24:36] = self.controller.joint_vel_scale * current_joint_vel

        obs[36:48] = self.controller.previous_action

        # height_scanner
        rpy = -quat_to_euler_angles(q_IB)
        rpy[:2] = 0.0
        yaw_rot = np.array(Gf.Matrix3f(euler_to_rot_matrix(rpy)))

        world_scan_points = np.matmul(yaw_rot, self._scan_points.T).T + pos_IB

        for i in range(world_scan_points.shape[0]):
            self._query_info.clear()
            self.physx_query_interface.raycast_all(tuple(world_scan_points[i]), (0.0, 0.0, -1.0), 100,
                                                   self._hit_report_callback)
            if self._query_info:
                distance = min(self._query_info)
                obs[48 + i] = np.clip(distance - 0.5, -1.0, 1.0)
            else:
                print("No hit")

        return obs

    def _hit_report_callback(self, hit):
        current_hit_body = hit.rigid_body
        if "/World/layout/GroundPlane" in current_hit_body:
            self._query_info.append(hit.distance)
        return True
