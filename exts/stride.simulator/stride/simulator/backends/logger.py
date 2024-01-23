import carb

import omni.kit.app
from stride.simulator.backends.backend import Backend

class Logger(Backend):

    def __init__(self, vehicle_id: int):
        self._id = vehicle_id


    def update_state(self, state):
        """
        Method that handles the receival of the state of the vehicle. It just prints the state to the console.
        """

        carb.log_info("Received state update for vehicle {}".format(self._id))
        carb.log_info("Position: {}".format(state.position))
        carb.log_info("Attitude: {}".format(state.attitude))
        carb.log_info("Linear velocity: {}".format(state.linear_velocity))
        carb.log_info("Angular velocity: {}".format(state.angular_velocity))
        carb.log_info("Linear acceleration: {}".format(state.linear_acceleration))


    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """

        if sensor_type == "IMU":
            self.update_imu_data(data)

        # TODO: Add support for other sensors


    def update_imu_data(self, data):
        """
        Method that handles the receival of IMU data. It just prints the data to the console.
        """
        carb.log_info("Received IMU data for vehicle {}".format(self._id))
        carb.log_info("Angular velocity: {}".format(data["angular_velocity"]))
        carb.log_info("Linear acceleration: {}".format(data["linear_acceleration"]))


    def update(self, dt: float):
        """
        Method that update the state of the backend and the information being sent/received from the communication
        interface. This method will be called by the simulation on every physics steps.
        """

        carb.log_info("Updating logger backend for vehicle {}".format(self._id))
        carb.log_info("Time elapsed since last update: {}".format(dt))