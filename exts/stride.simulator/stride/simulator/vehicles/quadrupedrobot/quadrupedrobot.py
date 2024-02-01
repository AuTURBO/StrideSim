# The vehicle interface
from stride.simulator.vehicles.vehicle import Vehicle
from stride.simulator.interfaces.stride_sim_interface import StrideInterface
# import carb

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
            sensor.set_spherical_coordinate(StrideInterface().latitude, StrideInterface().longitude,
                                            StrideInterface().altitude)
            pass

        # debug tool
        import ipdb # pylint: disable=import-outside-toplevel
        ipdb.set_trace()

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # 4. Save the backend interface (if given in the configuration of the multirotor)
        # and initialize them
        self._backends = config.backends
        for backend in self._backends:
            backend.initialize(self)

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
            command = [0.0 for i in range(3)] # FIXME: change 3 to base command size

        torque = self.controller.advance(dt, command)

        self.apply_torque(torque)

        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)

    def apply_torque(self, torque):

        self._dc_interface.wake_up_articulation(self._handle)

        torque_reorder = torque.reshape([4, 3]).T.flat

        self._dc_interface.set_articulation_dof_efforts(self._handle, torque_reorder)
