"""Package containing task implementations for various robotic environments."""

import os
import toml

from omni.isaac.lab_tasks.utils import import_packages

# FIXME: 무작정 주석처리해버림, 무슨 문제가 생길지 모름


##
# Register Gym environments.
##


# The blacklist is used to prevent importing configs from sub-packages
_BLACKLIST_PKGS = ["utils"]
# Import all configs in this package
import_packages(__name__, _BLACKLIST_PKGS)
