__all__ = ["Sensor"]

from stride.simulator.vehicles import State

class Sensor:
    """The base class for implementing a sensor.

    Attributes:
        update_period (float): The period for each sensor update: update_period = 1 / update_frequency (in seconds).
        origin_latitude (float): The latitude of the origin of the world in degrees.
        origin_longitude (float): The longitude of the origin of the world in degrees.
        origin_altitude (float): The altitude of the origin of the world relative to sea water level in meters.
    """
    def __init__(self, sensor_type: str, update_frequency: float):
        """Initialize the Sensor class

        Args:
            sensor_type (str): A name that describes the type of sensor.
            update_frequency (float): The rate at which the data in the sensor should be refreshed (in Hz).
        """

        # Set the sensor type and update rate.
        self._sensor_type = sensor_type
        self._update_frequency = update_frequency
        self._update_period = 1.0 / self._update_frequency

        # Auxiliary variables used to control whether to update the sensor or not, given the time elapsed.
        self._first_update = True
        self._total_time = 0.0

        # Set the spherical coordinate of the world - some sensors might need it.
        self._origin_latitude = -999
        self._origin_longitude = -999
        self._origin_altitude = 0.0

    def set_spherical_coordinate(self, origin_latitude, origin_longitude, origin_altitude):
        """Method that initializes the sensor shperical coordinate attributes.

        Note:
            Given that some sensors require the knowledge of the latitude, longitude and altitude of the [0, 0, 0]
            coordinate of the world, then we might as well just save this information for whatever sensor that comes.

        Args:
            origin_latitude (float): The latitude of the origin of the world in degrees.
            origin_longitude (float): The longitude of the origin of the world in degrees.
            origin_altitude (float): The altitude of the origin of the world relative to sea water level in meters.
        """
        self._origin_latitude = origin_latitude
        self._origin_longitude = origin_longitude
        self._origin_altitude = origin_altitude

    def set_update_frequency(self, frequency: float):
        """Method that changes the update frequency and period of the sensor.

        Args:
            frequency (float): The new frequency at which the data in the sensor should be refreshed (in Hz).
        """
        self._update_frequency = frequency
        self._update_period = 1.0 / self._update_frequency

    def update_at_frequency(fnc):
        """Decorator function used to check if the time elapsed between the last sensor update call and the current
        sensor update call is higher than the defined 'update_frequency' of the sensor. If so, we need to actually
        compute new values to simulate a measurement of the sensor at a given frequency.

        Args:
            fnc (function): The function that we want to enforce a specific update rate.

        Examples:
            >>> class GPS(Sensor):
            >>>    @Sensor.update_at_frequency
            >>>    def update(self):
            >>>        (do some logic here)

        Returns:
            [None, Dict]: This decorator function returns None if there was no data to be produced by the sensor at the
            specified timestamp or a Dict with the current state of the sensor otherwise.
        """

        # Define a wrapper function so that the "self" of the object can be passed to the function as well.

        def wrapper(self, state: State, dt: float):
            # Add the total time passed between the last time the sensor was updated and the current call.
            self.total_time += dt

            # If it is time to update the sensor data, then just call the update function of the sensor.
            if self.total_time >= self.update_period or self.first_update:

                # Result of the update function for the sensor.
                result = fnc(self, state, self.total_time) # pylint: disable=not-callable, TODO: enable this.

                # Reset the auxiliary counter variables.
                self.first_update = False
                self.total_time = 0.0

                return result
            return None
        return wrapper

    @property
    def sensor_type(self):
        """
        (str) A name that describes the type of sensor.
        """
        return self._sensor_type

    @property
    def update_frequency(self):
        """
        (float) The rate at which the data in the sensor should be refreshed (in Hz).
        """
        return self._update_frequency

    @property
    def update_period(self):
        """
        (float) The period for each sensor update: update_period = 1 / update_frequency (in seconds).
        """
        return self._update_period

    @property
    def first_update(self):
        """
        (bool) A flag that indicates whether this is the first time the sensor is being updated.
        """
        return self._first_update

    @property
    def total_time(self):
        """
        (float) The total time elapsed since the last sensor update.
        """
        return self._total_time

    @property
    def origin_latitude(self):
        """
        (float) The latitude of the origin of the world in degrees.
        """
        return self._origin_latitude

    @property
    def origin_longitude(self):
        """
        (float) The longitude of the origin of the world in degrees.
        """
        return self._origin_longitude

    @property
    def origin_altitude(self):
        """
        (float) The altitude of the origin of the world relative to sea water level in meters.
        """
        return self._origin_altitude

    def update(self, state: State, dt: float):
        """Method that should be implemented by the class that inherits Sensor. This is where the actual implementation
        of the sensor should be performed.

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """
        pass

    def config_from_dict(self, config_dict):
        """Method that should be implemented by the class that inherits Sensor. This is where the configuration of the
        sensor based on a dictionary input should be performed.

        Args:
            config_dict (dict): A dictionary containing the configurations of the sensor
        """
        pass
