from stride.simulator.vehicles.quadrupedrobot.quadrupedrobot import QuadrupedRobot, QuadrupedRobotConfig

# Mavlink interface
# from stride.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Get the location of the asset
from stride.simulator.params import ROBOTS_ENVIRONMNETS


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
        self.sensors = []  # pylint: disable=use-list-literal FIXME

        # The backends for actually sending commands to the vehicle.
        # By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle].
        # It can also be a ROS2 backend or your own custom Backend implementation!
        self.backends = []  # pylint: disable=use-list-literal FIXME


class AnymalC(QuadrupedRobot):

    def __init__(self, id: int, init_pos, init_orientation, config=AnymalCConfig()):

        if init_pos is None:
            init_pos = [0.0, 0.0, 0.5]
        if init_orientation is None:
            init_orientation = [0.0, 0.0, 0.0, 1.0]

        super().__init__(config.stage_prefix, config.usd_file, id, init_pos, init_orientation, config=config)
