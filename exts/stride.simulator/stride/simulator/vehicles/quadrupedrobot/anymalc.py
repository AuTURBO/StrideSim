from stride.simulator.vehicles.quadrupedrobot.quadrupedrobot import (
    QuadrupedRobot,
    QuadrupedRobotConfig,
)

# Mavlink interface
# from stride.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Get the location of the asset
# from stride.simulator.backends import LoggerBackend
from stride.simulator.params import ROBOTS
from stride.simulator.vehicles.sensors.imu import Imu
from stride.simulator.vehicles.sensors.lidar import Lidar

from stride.simulator.vehicles.controllers.anymal_controller import AnyamlController

import yaml
import os


class AnymalCConfig(QuadrupedRobotConfig):
    """
    AnymalC configuration class
    """

    def __init__(self):

        super().__init__()

        self.vehicle_name = "anymal_c"

        # Get the path to the "" directory
        stridesim_dir = os.path.abspath(__file__)
        for _ in range(5):
            stridesim_dir = os.path.dirname(stridesim_dir)

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "/World/AnymalC"

        # The USD file that describes the visual aspect of the vehicle
        self.usd_file = ROBOTS["Anymal C"]

        # read config file
        with open(
            stridesim_dir + "/config/anymalc_cfg.yaml", "r", encoding="utf-8"
        ) as file:
            self.config = yaml.safe_load(file)

        # The default sensors for a Anymal C
        self.sensors = [
            Imu(self.config["sensor"]["imu"]),
            Lidar(self.config["sensor"]["lidar"]),
        ]  # pylint: disable=use-list-literal FIXME

        # The backends for actually sending commands to the vehicle.
        # It can also be a ROS2 backend or your own custom Backend implementation!
        self.backends = []  # pylint: disable=use-list-literal


class AnymalC(QuadrupedRobot):
    """AnymalC class - It is a child class of QuadrupedRobot class to implement a AnymalC robot in the simulator."""

    def __init__(self, id: int, init_pos, init_orientation, config=AnymalCConfig()):

        if init_pos is None:
            init_pos = [0.0, 0.0, 0.5]
        if init_orientation is None:
            init_orientation = [0.0, 0.0, 0.0, 1.0]

        super().__init__(
            config.stage_prefix,
            config.usd_file,
            id,
            init_pos,
            init_orientation,
            config=config,
        )

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
            try:
                sensor_data = sensor.update(self._state, dt)
            except Exception as e:  # pylint: disable=broad-except
                print(f"Error updating sensor: {e}")
                continue

            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize the dc interface, set up drive mode
        """
        super().initialize(physics_sim_view=physics_sim_view)
        self.get_articulation_controller().set_effort_modes("force")
        self.get_articulation_controller().switch_control_mode("effort")
