import os
import subprocess

import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder, get_style, setup_ui_headers  # scrolling_frame_builder

from StrideSim.anymal_d import AnymalD
from StrideSim.base_sample import BaseSampleExtension
from StrideSim.parameters import RL_DIR


class AnyamlDExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)

        overview = "This Example shows quadruped simulation in Isaac Sim. Currently there is a performance issue with "
        overview += (
            "the quadruped gait controller; it's being investigated and will be improved in an upcoming release."
        )
        overview += "\n\tKeybord Input:"
        overview += "\n\t\tup arrow / numpad 8: Move Forward"
        overview += "\n\t\tdown arrow/ numpad 2: Move Reverse"
        overview += "\n\t\tleft arrow/ numpad 4: Move Left"
        overview += "\n\t\tright arrow / numpad 6: Move Right"
        overview += "\n\t\tN / numpad 7: Spin Counterclockwise"
        overview += "\n\t\tM / numpad 9: Spin Clockwise"

        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        super().start_extension(
            menu_name="",
            submenu_name="",
            name="StrideSim_AnymalD",
            title="Auturbo quadruped robot simulation",
            doc_link="https://github.com/AuTURBO/StrideSim",
            overview=overview,
            file_path=os.path.abspath(__file__),
            sample=AnymalD(),
        )
        return

    def _build_ui(self, title, doc_link, overview, file_path, number_of_extra_frames):
        with self._window.frame:
            self._main_stack = ui.VStack(spacing=5, height=0)
            with self._main_stack:
                setup_ui_headers(self._ext_id, file_path, title, doc_link, overview)
                self._controls_frame = ui.CollapsableFrame(
                    title="World Controls",
                    width=ui.Fraction(1),
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    for i in range(number_of_extra_frames):
                        self._extra_frames.append(
                            ui.CollapsableFrame(
                                title="",
                                width=ui.Fraction(0.33),
                                height=0,
                                visible=False,
                                collapsed=False,
                                style=get_style(),
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            )
                        )
                with self._controls_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        dict = {
                            "label": "Load World",
                            "type": "button",
                            "text": "Load",
                            "tooltip": "Load World and Task",
                            "on_clicked_fn": self._on_load_world,
                        }
                        self._buttons["Load World"] = btn_builder(**dict)
                        self._buttons["Load World"].enabled = True
                        dict = {
                            "label": "Reset",
                            "type": "button",
                            "text": "Reset",
                            "tooltip": "Reset robot and environment",
                            "on_clicked_fn": self._on_reset,
                        }
                        self._buttons["Reset"] = btn_builder(**dict)
                        self._buttons["Reset"].enabled = False

                # New panel for Reinforcement Learning
                self._rl_frame = ui.CollapsableFrame(
                    title="Reinforcement Learning Panel",
                    width=ui.Fraction(1),
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with self._rl_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        ui.Label("Task Name:", width=ui.Fraction(0.3))
                        self._rl_task_name_field = ui.StringField(height=0, width=ui.Fraction(0.7))
                        self._rl_task_name_field.model.set_value("Template-Isaac-Velocity-Rough-Anymal-D-v0")
                        ui.Label("Window popup:", width=ui.Fraction(0.3))

                        self._headless_dropdown = ui.ComboBox(0, "on", "off")

                        dict_train = {
                            "label": "Train",
                            "type": "button",
                            "text": "TRAIN",
                            "tooltip": "Start training",
                            "on_clicked_fn": self._on_train,
                        }
                        self._buttons["Train"] = btn_builder(**dict_train)

                        dict_play = {
                            "label": "Play",
                            "type": "button",
                            "text": "PLAY",
                            "tooltip": "Start playing",
                            "on_clicked_fn": self._on_play,
                        }
                        self._buttons["Play"] = btn_builder(**dict_play)
        return

    def _on_train(self):
        task_name = self._rl_task_name_field.model.get_value_as_string()
        training_window = self._headless_dropdown.model.get_item_value_model().get_value_as_string()

        command = f"python {RL_DIR}/train.py --task {task_name}"
        if training_window == "1":
            command += " --headless"

        try:
            subprocess.Popen(command, shell=True)
            print(f"Started training process: {command}")
        except Exception as e:
            print(f"Error starting training process: {e}")
        return

    def _on_play(self):
        task_name = self._rl_task_name_field.model.get_value_as_string()

        command = f"python {RL_DIR}/play.py --task {task_name}"

        command += " --num_envs 10"

        try:
            subprocess.Popen(command, shell=True)
            print(f"Started training process: {command}")
        except Exception as e:
            print(f"Error starting training process: {e}")
        return
