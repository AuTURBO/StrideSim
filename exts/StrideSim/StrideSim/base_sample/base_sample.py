# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gc
from abc import abstractmethod

from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async


class BaseSample:
    def __init__(self) -> None:
        self._world = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        # self._logging_info = ""
        return

    def get_world(self):
        return self._world

    def set_world_settings(self, physics_dt=None, stage_units_in_meters=None, rendering_dt=None):
        if physics_dt is not None:
            self._world_settings["physics_dt"] = physics_dt
        if stage_units_in_meters is not None:
            self._world_settings["stage_units_in_meters"] = stage_units_in_meters
        if rendering_dt is not None:
            self._world_settings["rendering_dt"] = rendering_dt
        return

    async def load_world_async(self):
        """Function called when clicking load button"""
        if World.instance() is None:
            await create_new_stage_async()
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
        self._current_tasks = self._world.get_current_tasks()
        await self._world.reset_async()
        await self._world.pause_async()
        await self.setup_post_load()
        if len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        return

    async def reset_async(self):
        """Function called when clicking reset button"""
        if self._world.is_tasks_scene_built() and len(self._current_tasks) > 0:
            self._world.remove_physics_callback("tasks_step")
        await self._world.play_async()
        await update_stage_async()
        await self.setup_pre_reset()
        await self._world.reset_async()
        await self._world.pause_async()
        await self.setup_post_reset()
        if self._world.is_tasks_scene_built() and len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        return

    @abstractmethod
    def setup_scene(self, scene: Scene) -> None:
        """used to setup anything in the world, adding tasks happen here for instance.

        Args:
            scene (Scene): [description]
        """
        return

    @abstractmethod
    async def setup_post_load(self):
        """called after first reset of the world when pressing load,
        initializing private variables happen here.
        """
        return

    @abstractmethod
    async def setup_pre_reset(self):
        """called in reset button before resetting the world
        to remove a physics callback for instance or a controller reset
        """
        return

    @abstractmethod
    async def setup_post_reset(self):
        """called in reset button after resetting the world which includes one step with rendering"""
        return

    @abstractmethod
    async def setup_post_clear(self):
        """called after clicking clear button
        or after creating a new stage and clearing the instance of the world with its callbacks
        """
        return

    # def log_info(self, info):
    #     self._logging_info += str(info) + "\n"
    #     return

    def _world_cleanup(self):
        self._world.stop()
        self._world.clear_all_callbacks()
        self._current_tasks = None
        self.world_cleanup()
        return

    def world_cleanup(self):
        """Function called when extension shutdowns and starts again, (hot reloading feature)"""
        return

    async def clear_async(self):
        """Function called when clicking clear button"""
        await create_new_stage_async()
        if self._world is not None:
            self._world_cleanup()
            self._world.clear_instance()
            self._world = None
            gc.collect()
        await self.setup_post_clear()
        return
