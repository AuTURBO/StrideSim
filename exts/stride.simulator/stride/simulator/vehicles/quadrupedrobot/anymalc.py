from stride.simulator.vehicles.quadrupedrobot.quadrupedrobot import QuadrupedRobot, QuadrupedRobotConfig

# Mavlink interface
# from stride.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Get the location of the asset
from stride.simulator.backends import LoggerBackend
from stride.simulator.params import ROBOTS_ENVIRONMNETS
from stride.simulator.vehicles.sensors.imu import Imu

from stride.simulator.vehicles.controllers.anymal_controller import AnyamlController


class AnymalCConfig(QuadrupedRobotConfig):
    """
    AnymalC configuration class
    """

    def __init__(self):

        super().__init__()

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "/World/AnymalC"

        # The USD file that describes the visual aspect of the vehicle
        self.usd_file = ROBOTS_ENVIRONMNETS["Anymal C"]

        # The default sensors for a Anymal C
        self.sensors = [Imu()]  # pylint: disable=use-list-literal FIXME

        # The backends for actually sending commands to the vehicle.
        # It can also be a ROS2 backend or your own custom Backend implementation!
        self.backends = [LoggerBackend()]  # pylint: disable=use-list-literal FIXME


class AnymalC(QuadrupedRobot):
    """AnymalC class - It is a child class of QuadrupedRobot class to implement a AnymalC robot in the simulator.
    """

    def __init__(self, id: int, init_pos, init_orientation, config=AnymalCConfig()):

        if init_pos is None:
            init_pos = [0.0, 0.0, 0.5]
        if init_orientation is None:
            init_orientation = [0.0, 0.0, 0.0, 1.0]

        super().__init__(config.stage_prefix, config.usd_file, id, init_pos, init_orientation, config=config)

        self.controller = AnyamlController()

    def update_sensors(self, dt: float):
        """Callback that is called at every physics steps and will call the sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_sensor method will also be called for
        every backend. For example, if new data is generated for an IMU and we have a MavlinkBackend, then the
        update_sensor method will be called for that backend so that this data can latter be sent thorugh mavlink.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)

            if sensor_data is not None:
                print("TODO: Implement backend code.")

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
        yaw_rot = np.array(Gf.Matrix3f(euler_to_rot_matrix(rpy)))

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