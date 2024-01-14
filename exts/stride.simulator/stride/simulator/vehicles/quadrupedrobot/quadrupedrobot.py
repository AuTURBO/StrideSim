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

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in simulation based on the motor speed. 
        This method must be implemented by a class that inherits this type. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the articulation root of the vehicle
        articulation = self._world.dc_interface.get_articulation(self._stage_prefix)

        # Get the desired angular velocities for each rotor from the first backend (can be mavlink or other) expressed in rad/s
        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0 for i in range(self._thrusters._num_rotors)]

        # Input the desired rotor velocities in the thruster model
        self._thrusters.set_input_reference(desired_rotor_velocities)

        # Get the desired forces to apply to the vehicle
        forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)

        # Apply force to each rotor
        for i in range(4):

            # Apply the force in Z on the rotor frame
            self.apply_force([0.0, 0.0, forces_z[i]], body_part="/rotor" + str(i))

            # Generate the rotating propeller visual effect
            self.handle_propeller_visual(i, forces_z[i], articulation)

        # Apply the torque to the body frame of the vehicle that corresponds to the rolling moment
        self.apply_torque([0.0, 0.0, rolling_moment], "/body")

        # Compute the total linear drag force to apply to the vehicle's body frame
        drag = self._drag.update(self._state, dt)
        self.apply_force(drag, body_part="/body")

        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)
        
    def apply_force(self, force, pos=[0.0, 0.0, 0.0], body_part="/body"):
        """
        Method that will apply a force on the rigidbody, on the part specified in the 'body_part' at its relative position
        given by 'pos' (following a FLU) convention. 

        Args:
            force (list): A 3-dimensional vector of floats with the force [Fx, Fy, Fz] on the body axis of the vehicle according to a FLU convention.
            pos (list): _description_. Defaults to [0.0, 0.0, 0.0].
            body_part (str): . Defaults to "/body".
        """

        # Get the handle of the rigidbody that we will apply the force to
        rb = self._world.dc_interface.get_rigid_body(self._stage_prefix + body_part)

        # Apply the force to the rigidbody. The force should be expressed in the rigidbody frame
        self._world.dc_interface.apply_body_force(rb, carb._carb.Float3(force), carb._carb.Float3(pos), False)

# TODO: check other functions from stride simulator
