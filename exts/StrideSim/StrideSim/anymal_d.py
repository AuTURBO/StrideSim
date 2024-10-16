# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np

import carb
import omni
import omni.appwindow  # Contains handle to keyboard

from StrideSim.anymal_articulation import AnymalD_Atriculation
from StrideSim.base_sample import BaseSample


class AnymalD(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 500.0
        self._world_settings["rendering_dt"] = 10.0 / 500.0
        self._base_command = [0.0, 0.0, 0.0]

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [2.0, 0.0, 0.0],
            "UP": [2.0, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-2.0, 0.0, 0.0],
            "DOWN": [-2.0, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -2.0, 0.0],
            "RIGHT": [0.0, -2.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 2.0, 0.0],
            "LEFT": [0.0, 2.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 2.0],
            "N": [0.0, 0.0, 2.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -2.0],
            "M": [0.0, 0.0, -2.0],
        }

    def setup_scene(self) -> None:
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )
        self.AnymalD = AnymalD_Atriculation(
            prim_path="/World/AnymalD",
            name="AnymalD",
            position=np.array([0, 0, 0.8]),
        )
        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP),
            self._timeline_timer_callback_fn,
        )
        self.AnymalD.robot.set_joints_default_state(self.AnymalD._default_joint_pos)

    async def setup_post_load(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._world.add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        self._physics_ready = False
        await self._world.play_async()
        self.AnymalD.initialize()

    async def setup_post_reset(self) -> None:
        await self._world.play_async()
        self._physics_ready = False
        self.AnymalD.initialize()

    def on_physics_step(self, step_size) -> None:
        if self._physics_ready:
            self.AnymalD.advance(step_size, self._base_command)
        else:
            self._physics_ready = True

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """Subscriber callback to when kit is updated."""

        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True

    def _timeline_timer_callback_fn(self, event) -> None:
        if self.AnymalD:
            self.AnymalD.post_reset()
            self._physics_ready = False

    def world_cleanup(self):
        self._event_timer_callback = None
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")
