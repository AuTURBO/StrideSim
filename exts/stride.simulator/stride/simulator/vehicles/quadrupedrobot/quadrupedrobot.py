# The vehicle interface
from stride.simulator.vehicles.vehicle import Vehicle
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
                                Defaults to "quadrotor".
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


# TODO: check other functions from stride simulator
