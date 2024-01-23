from stride.simulator.vehicles.quadrupedrobot.quadrupedrobot import QuadrupedRobot, QuadrupedRobotConfig

# Mavlink interface
# from stride.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Get the location of the asset
from stride.simulator.params import ROBOTS_ENVIRONMNETS
from stride.simulator.vehicles.sensors import Imu
from stride.simulator.interfaces import StrideInterface


class AnymalCConfig(QuadrupedRobotConfig):
    """
    AnymalC configuration class
    """

    def __init__(self):

        super().__init__()

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "AnymalC"

        # The USD file that describes the visual aspect of the vehicle
        # (and some properties such as mass and moments of inertia)
        self.usd_file = ROBOTS_ENVIRONMNETS["Anymal C"]

        # The default sensors for a quadrotor
        self.sensors = [Imu()]  # pylint: disable=use-list-literal FIXME

        # The backends for actually sending commands to the vehicle.
        # By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle].
        # It can also be a ROS2 backend or your own custom Backend implementation!
        self.backends = []  # pylint: disable=use-list-literal FIXME


class AnymalC(QuadrupedRobot):
    """AnymalC class - It is a child class of QuadrupedRobot class to implement a AnymalC robot in the simulator.
    """

    def __init__(self, id: int, init_pos, init_orientation, config=AnymalCConfig()):

        if init_pos is None:
            init_pos = [0.0, 0.0, 0.5]
        if init_orientation is None:
            init_orientation = [0.0, 0.0, 0.0, 1.0]

        super().__init__(config.stage_prefix, config.usd_file, id, init_pos, init_orientation, config=config)


        self._sensors = config.sensors
        for sensor in self._sensors:
            sensor.set_spherical_coordinate(StrideInterface().latitude, StrideInterface().longitude,
                                            StrideInterface().altitude)

        # Add callbacks to the physics engine to update each sensor at every timestep and let the sensor decide
        # depending on its internal update rate whether to generate new data.
        # TODO: Uncomment this when the physics engine is implemented.
        # self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

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
