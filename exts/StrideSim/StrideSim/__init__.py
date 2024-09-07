"""
Python module serving as a project/extension template.
"""

from .base_sample import *

# Register Gym environments.
from .tasks import *

# Register UI extensions.
from .quadruped import *
from .quadruped_extension import *
