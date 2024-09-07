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
from omni.isaac.quadruped.robots import Unitree

from .base_sample import BaseSample


class QuadrupedExample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 400.0
        self._world_settings["rendering_dt"] = 5.0 / 400.0
        self._enter_toggled = 0
        self._base_command = [0.0, 0.0, 0.0, 0]
        self._event_flag = False

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [5, 0.0, 0.0],
            "UP": [5, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-5, 0.0, 0.0],
            "DOWN": [-5, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -4.0, 0.0],
            "RIGHT": [0.0, -4.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 4.0, 0.0],
            "LEFT": [0.0, 4.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.0],
            "N": [0.0, 0.0, 1.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.0],
            "M": [0.0, 0.0, -1.0],
        }
        return

    def setup_scene(self) -> None:
        world = self.get_world()
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )
        self._a1 = world.scene.add(
            Unitree(
                prim_path="/World/A1",
                name="A1",
                position=np.array([0, 0, 0.400]),
                physics_dt=self._world_settings["physics_dt"],
            )
        )
        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self._timeline_timer_callback_fn
        )
        return

    async def setup_post_load(self) -> None:
        self._world = self.get_world()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._world.add_physics_callback("sending_actions", callback_fn=self.on_physics_step)
        await self._world.play_async()
        return

    async def setup_post_reset(self) -> None:
        self._event_flag = False
        await self._world.play_async()
        self._a1.set_state(self._a1._default_a1_state)
        self._a1.post_reset()
        return

    def on_physics_step(self, step_size) -> None:
        if self._event_flag:
            self._a1._qp_controller.switch_mode()
            self._event_flag = False
        self._a1.advance(step_size, self._base_command)

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """Subscriber callback to when kit is updated."""
        # reset event
        self._event_flag = False
        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] += np.array(self._input_keyboard_mapping[event.input.name])
                self._event_flag = True

            # enter, toggle the last command
            if event.input.name == "ENTER" and self._enter_toggled is False:
                self._enter_toggled = True
                if self._base_command[3] == 0:
                    self._base_command[3] = 1
                else:
                    self._base_command[3] = 0
                self._event_flag = True

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] -= np.array(self._input_keyboard_mapping[event.input.name])
                self._event_flag = True
            # enter, toggle the last command
            if event.input.name == "ENTER":
                self._enter_toggled = False
        # since no error, we are fine :)
        return True

    def _timeline_timer_callback_fn(self, event) -> None:
        if self._a1:
            self._a1.post_reset()
        return

    def world_cleanup(self):
        self._event_timer_callback = None
        if self._world.physics_callback_exists("sending_actions"):
            self._world.remove_physics_callback("sending_actions")
        return
