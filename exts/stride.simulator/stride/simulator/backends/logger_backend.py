__all__ = ["LoggerBackend", "LoggerBackendConfig"]

import carb

from stride.simulator.backends.backend import Backend


class LoggerBackendConfig:
    """
    An auxiliary data class used to store all the configurations for the LoggerBackend communications.
    """

    def __init__(self, config=None):
        """
        Initialize the LoggerBackendConfig class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the LoggerBackend interface
                           - it can be empty or only have some of the parameters used by this backend.
        
        Examples:
            The dictionary default parameters are

            >>> {"vehicle_id": 0,           
            >>>  "update_rate": 250.0
            >>> }
        """
        if config is None:
            config = {}
        else:
            assert isinstance(config, dict), "The config parameter must be a dictionary."

        self.vehicle_id = config.get("vehicle_id", 0)
        self.update_rate: float = config.get("update_rate", 250.0)  # [Hz]


class LoggerBackend(Backend):
    """
    Logger Backend that just prints the state of the vehicle to the console.
    """

    def __init__(self, config=LoggerBackendConfig()):
        """Initialize the Logger

        Args:
            config (LoggerBackendConfig): The configuration class for the LoggerBackend. Defaults to
                                          LoggerBackendConfig().
        """

        super().__init__()
        self._id = config.vehicle_id

    def update_state(self, state):
        """
        Method that handles the receival of the state of the vehicle. It just prints the state to the console.
        """
        carb.log_info(f"Received state update for vehicle {self._id}")
        carb.log_info(f"Position: {state.position}")
        carb.log_info(f"Attitude: {state.attitude}")
        carb.log_info(f"Linear velocity: {state.linear_velocity}")
        carb.log_info(f"Angular velocity: {state.angular_velocity}")
        carb.log_info(f"Linear acceleration: {state.linear_acceleration}")

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """

        carb.log_info(f"sensor type: {sensor_type}")

        if sensor_type == "Imu":
            self.update_imu_data(data)
        else:
            carb.log_warn(f"Sensor type {sensor_type} is not supported by the ROS2 backend.")
            pass
        # TODO: Add support for other sensors

    def update_imu_data(self, data):
        """
        Method that handles the receival of IMU data. It just prints the data to the console.
        """
        carb.log_info(f"Received IMU data for vehicle {self._id}")
        carb.log_info(f"Angular velocity: {data['angular_velocity']}")
        carb.log_info(f"Linear acceleration: {data['linear_acceleration']}")

    def update(self, dt: float):
        """
        Method that update the state of the backend and the information being sent/received from the communication
        interface. This method will be called by the simulation on every physics steps.
        """
        carb.log_info(f"Updating logger backend for vehicle {self._id}")
        carb.log_info(f"Time elapsed since last update: {dt}")
