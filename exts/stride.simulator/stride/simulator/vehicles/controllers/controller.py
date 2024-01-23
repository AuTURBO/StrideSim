__all__ = ["Controller"]

# import numpy as np

class Controller:
    """
    Controller.
    Note:
        TODO
    """

    def __init__(self):
        self._torque = None

    def advance(self, state):
        """
        Add your algorithm to make torque

        Args:
            state (class): robot states

        Returns:
            torque: robot torques
        """

        return self._torque
