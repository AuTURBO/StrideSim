# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gc
import math
import weakref

import carb
import omni.appwindow
import omni.ext
import omni.ui as ui
from omni.isaac.ui.dpad import Dpad
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.style import VERTICAL_SPACING
from omni.isaac.ui.ui_utils import *
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

EXTENSION_NAME = "Example UI"

PRINT_DEBUG = True


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        self._ext_id = ext_id
        self._settings = carb.settings.get_settings()

        # Keep a Reference to the Usd Context, Stage, & Viewport
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._app_event_sub = None

        # Initialize the UI Window
        self._window = None

        # Keep a Reference to interactive GUI elements
        self._models = {}

        # Extension-specific Parameters
        self._plot_data = [0.0 for i in range(360)]
        self._plot_data_xyz = []
        for j in range(3):
            data = []
            for i in range(360):
                data.append(math.cos(math.radians(i + j * 100)))
            self._plot_data_xyz.append(data)

        self._tick = 0
        self._folder_picker = None

        # Add EXTENSION_NAME to a Drop Down Menu
        # The UI for EXTENSION_NAME is created once selected from the Menu.
        menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [
            MenuItemDescription(name="Misc", sub_menu=[MenuItemDescription(name="Templates", sub_menu=menu_items)])
        ]
        add_menu_items(self._menu_items, "Isaac Examples")

        # Add Dpad Controllers
        self.dpads = []
        for i in range(4):
            self.dpads.append(Dpad(name=f"Dpad Controller {i}"))

        self._build_ui()

    def on_shutdown(self):
        """Cleanup objects on extension shutdown"""
        self._app_event_subscription = None
        remove_menu_items(self._menu_items, "Isaac Examples")
        self._window = None
        self._app_event_sub = None
        self._search_bar.destroy()
        if self._folder_picker:
            self._folder_picker.hide()
            self._folder_picker.destroy()
            self._folder_picker = None
        self._models = {}
        for dpad in self.dpads:
            dpad.shutdown()
        gc.collect()

    def _menu_callback(self):
        """Call the UI builder once selected from the drop down menu"""
        self._build_ui()

        # Add Dpads on Top
        self.dpads = []
        for i in range(4):
            self.dpads.append(Dpad(name=f"Dpad Controller {i}"))

    def _build_ui(self):
        """Builds the UI for EXTENSION_NAME"""
        if not self._window:
            self._window = ui.Window(
                title=EXTENSION_NAME, width=700, height=0, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM
            )

            with self._window.frame:
                with ui.VStack(spacing=5, height=0):
                    title = "Isaac Sim Example UI"
                    doc_link = "https://docs.omniverse.nvidia.com/isaacsim/latest/index.html"

                    overview = (
                        "The Example UI shows how to use Isaac Sim's robotics-centric UI tools for your own extensions."
                    )
                    overview += "\n\nUse this as a reference or template when creating a UI for a new project."
                    overview += "\n\nPress the 'Open in IDE' button to view the source code."

                    setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

                    self.build_example_gui_grid()
                    self.build_plot_frame()
                    self.build_search_frame()
                    self.build_folder_picker_frame()

                    self.build_comms_frame()

                    self.build_progress_bar_frame()

                    # Shows how to Group UI elements
                    self.build_custom_ui()

    def build_example_gui_grid(self):
        test_gui = {
            "Test_0": {
                "label": "CB_0",
                "type": "checkbox",
                "default_val": False,
                "tooltip": "Add Tooltip Here",
                "on_clicked_fn": self._on_dummy_callable_0,
            },
            "Test_1": {
                "label": "CB_1",
                "type": "checkbox",
                "default_val": True,
                "tooltip": "Add Tooltip Here",
                "on_clicked_fn": self._on_dummy_callable_1,
            },
            "Test_2": {
                "label": "CB_2",
                "type": "checkbox",
                "default_val": True,
                "tooltip": "Add Tooltip Here",
                "on_clicked_fn": self._on_dummy_callable_2,
            },
            "Test_4": {
                "label": "BTN_0",
                "type": "button",
                "text": "PRESS ME",
                "tooltip": "Add Tooltip Here",
                "on_clicked_fn": self._on_dummy_callable_1,
            },
            "Test_5": {
                "label": "BTN_GROUP_0",
                "count": 2,
                "type": "multi_button",
                "text": ["PRESS ME", "no...PReSs mE"],
                "tooltip": ["This is the Label Tooltip", "Tooltip 0", "Tooltip 1"],
                "on_clicked_fn": [self._on_dummy_callable_0, self._on_dummy_callable_2],
            },
            "Test_6": {
                "label": "BTN_GROUP_1",
                "count": 3,
                "type": "multi_button",
                "text": ["PRESS ME", "NO, PRESS ME", "NO...PRESs ME!"],
                "tooltip": ["This group has button tooltips", "Tooltip 0", "Tooltip 1", "Tooltip 2"],
                "on_clicked_fn": [self._on_dummy_callable_0, self._on_dummy_callable_2, self._on_dummy_callable_1],
            },
            "Test_6.5": {
                "label": "BTN_GROUP_2",
                "count": 3,
                "type": "multi_button",
                "text": ["PRESS ME", "NO, PRESS ME", "NO, PRESS ME!"],
                "tooltip": ["This group doesn't have button tooltips", "", "", ""],
                "on_clicked_fn": [self._on_dummy_callable_0, self._on_dummy_callable_2, self._on_dummy_callable_1],
            },
            "Test_7": {
                "label": "CB_GROUP_0",
                "count": 3,
                "type": "multi_checkbox",
                "default_val": [False, True, False, True],
                "text": ["Label 0", "Label 1", "Label 2"],
                "tooltip": ["This is the Label Tooltip", "Tooltip 0", "Tooltip 1", "Tooltip 2"],
                "on_clicked_fn": [self._on_dummy_callable_0, self._on_dummy_callable_2, self._on_dummy_callable_1],
            },
            "Test_8": {
                "label": "CB_GROUP_1",
                "count": 4,
                "type": "multi_checkbox",
                "default_val": [False, True, False, True],
                "text": ["Label 0", "Label 1", "Label 2", "Label 3"],
                "tooltip": ["This is the Label Tooltip", "Tooltip 0", "Tooltip 1", "Tooltip 2", "Tooltip 3"],
                "on_clicked_fn": [
                    self._on_dummy_callable_0,
                    self._on_dummy_callable_2,
                    self._on_dummy_callable_1,
                    self._on_dummy_callable_1,
                ],
            },
            "Test_15": {
                "label": "INT_DRAG",
                "type": "intfield",
                "default_val": 0,
                "min": -100,
                "max": 100,
                "tooltip": "This is the Label Tooltip",
            },
            "Test_16": {
                "label": "FLT_FIELD",
                "type": "floatfield",
                "default_val": 0,
                "min": -1.0,
                "max": 1.0,
                "tooltip": "This is the Label Tooltip",
            },
            "Test_17": {
                "label": "INT_FIELD_COMBO_0",
                "type": "combo_intfield_slider",
                "default_val": 0,
                "min": -100,
                "max": 100,
                "tooltip": ["This is the Label Tooltip", "INT FIELD Tooltip"],
            },
            "Test_9": {
                "label": "FF_COMBO_0",
                "type": "combo_floatfield_slider",
                "default_val": 0,
                "min": -1,
                "max": 1,
                "step": 0.001,
                "tooltip": ["This is the Label Tooltip", "FF Tooltip"],
            },
            "Test_10": {
                "label": "FF_COMBO_1",
                "type": "combo_floatfield_slider",
                "default_val": 0,
                "min": 0,
                "max": 20,
                "step": 2,
                "tooltip": ["This is the Label Tooltip", "FF Tooltip"],
            },
            "Test_11": {
                "label": "DROPDOWN",
                "type": "dropdown",
                "default_val": 2,
                "tooltip": "This is the Label Tooltip",
                "items": ["Config 1", "Config 2", "Config 3"],
                "on_clicked_fn": self._on_dummy_callable_3,
            },
            "Test_13": {
                "label": "DROPDOWN_GROUP",
                "type": "multi_dropdown",
                "count": 3,
                "default_val": [0, 1, 0],
                "tooltip": "This is the Label Tooltip",
                "items": [
                    ["Option 1", "Option 2", "Option 3"],
                    ["Option A", "Option B", "Option C"],
                    ["Option X", "Option Y"],
                ],
                "on_clicked_fn": [self._on_dummy_callable_3, self._on_dummy_callable_3, self._on_dummy_callable_3],
            },
            "Test_12": {
                "label": "ENABLE_DROPDN",
                "type": "checkbox_dropdown",
                "default_val": [False, 1],
                "tooltip": "This is the Label Tooltip",
                "items": ["Config 1", "Config 2", "Config 3"],
                "on_clicked_fn": [self._on_dummy_callable_0, self._on_dummy_callable_3],
            },
            "Test_3": {
                "label": "ENABLE_STR",
                "type": "checkbox_stringfield",
                "default_val": [False, "default"],
                "tooltip": "This is the Label Tooltip",
                "on_clicked_fn": self._on_dummy_callable_0,
            },
            "Test_14": {
                "label": "ENABLE_STREAM",
                "type": "checkbox_scrolling_frame",
                "default_val": [False, "No Data"],
                "tooltip": "This is the Label Tooltip",
                "on_clicked_fn": self._on_dummy_callable_0,
            },
        }

        self._grid = ui.CollapsableFrame(
            title="Example: GUI Grid",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._grid:
            with ui.VStack(spacing=5, height=0):

                # You can also build a UI with a dictionary
                for key, value in test_gui.items():
                    # Button
                    if value["type"] == "button":
                        self._models["btn_" + value["label"]] = btn_builder(**value)
                    # Checkbox
                    elif value["type"] == "checkbox":
                        self._models["cb_" + value["label"]] = cb_builder(**value)
                    # Multiple Buttons
                    elif value["type"] == "multi_button":
                        elems = multi_btn_builder(**value)
                        for i in range(len(elems)):
                            self._models["btn_" + value["label"]] = elems[i]
                    # Multiple Checkboxes
                    elif value["type"] == "multi_checkbox":
                        elems = multi_cb_builder(**value)
                        for i in range(len(elems)):
                            self._models["cb_" + value["label"]] = elems[i]
                    # Int Field
                    elif value["type"] == "intfield":
                        int_field = int_builder(**value)
                    # Float Field
                    elif value["type"] == "floatfield":
                        flt_field = float_builder(**value)
                    # Int Field + Slider
                    elif value["type"] == "combo_intfield_slider":
                        int_field, slider = combo_intfield_slider_builder(**value)
                    # Float Field + Slider
                    elif value["type"] == "combo_floatfield_slider":
                        flt_field, slider = combo_floatfield_slider_builder(**value)
                    # Dropdown ComboBox
                    elif value["type"] == "dropdown":
                        self._models["dropdown_" + value["label"]] = dropdown_builder(**value)
                    # Multiple Dropdown ComboBoxes
                    elif value["type"] == "multi_dropdown":
                        elems = multi_dropdown_builder(**value)
                        for i in range(len(elems)):
                            self._models["dropdown_" + value["label"]] = elems[i].model
                    # Checkbox + Stringfield
                    elif value["type"] == "checkbox_stringfield":
                        cb, str_field = combo_cb_str_builder(**value)
                    # Checkbox + Dropdown ComboBox
                    elif value["type"] == "checkbox_dropdown":
                        cb, combo_box = combo_cb_dropdown_builder(**value)
                    elif value["type"] == "checkbox_scrolling_frame":
                        cb, text = combo_cb_scrolling_frame_builder(**value)

                kwargs = {"label": "Translate", "axis_count": 3, "min": -1000, "max": 1000, "step": 1}
                self._models["multi_float_translate"] = xyz_builder(**kwargs)
                kwargs = {"label": "Rotate", "axis_count": 4, "tooltip": "Orientation in Quaternions"}
                self._models["multi_float_rotate"] = xyz_builder(**kwargs)

                kwargs = {"label": "Color Picker", "default_val": [0.353, 0.637, 0.269, 1]}
                self._models["color_picker"] = color_picker_builder(**kwargs)

                # Test building with default values
                ui.Spacer(height=LABEL_HEIGHT)
                ui.Label("Testing Default UI Elements")
                btn_builder()
                cb_builder()
                str_builder()
                float_builder()
                multi_btn_builder()
                multi_cb_builder()
                dropdown_builder()
                scrolling_frame_builder()
                multi_dropdown_builder()
                combo_cb_dropdown_builder()
                combo_cb_str_builder()
                btn_builder()
                combo_cb_scrolling_frame_builder()
                xyz_builder()
                color_picker_builder()
                ui.Spacer()

    def toggle_app_step(self, val=None):
        print("You've cliked time_series_plot_data:", val)
        if val:
            if not self._app_event_sub:
                self._app_event_sub = (
                    omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_app_step)
                )
            else:
                self._app_event_sub = None
        else:
            self._app_event_sub = None

    def _on_app_step(self, e: carb.events.IEvent):
        self._tick += 1
        val = math.sin(math.radians(self._tick))
        self._models["timeseries_plot_val"].set_value(val)
        self._plot_data.append(val)
        if len(self._plot_data) > 360:
            self._plot_data.pop(0)
        self._models["timeseries_plot"].set_data(*self._plot_data)

    def toggle_app_step_1(self, val=None):
        print("You've cliked time_series_plot_data:", val)
        if val:
            if not self._app_event_sub:
                self._app_event_sub = (
                    omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_app_step_1)
                )
            else:
                self._app_event_sub = None
        else:
            self._app_event_sub = None

    def _on_app_step_1(self, e: carb.events.IEvent):
        self._tick += 5
        val = math.sin(math.radians(self._tick))
        self._models["timeseries_plot_hist_val"].set_value(val)
        self._plot_data.append(val)
        if len(self._plot_data) > 360:
            self._plot_data.pop(0)
        self._models["timeseries_plot_hist"].set_data(*self._plot_data)

    def toggle_app_step_2(self, val=None):
        print("You've cliked time_series_plot_data:", val)
        if val:
            if not self._app_event_sub:
                self._app_event_sub = (
                    omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_app_step_2)
                )
            else:
                self._app_event_sub = None
        else:
            self._app_event_sub = None

    def _on_app_step_2(self, e: carb.events.IEvent):
        self._tick += 1
        for i in range(3):
            val = math.sin(math.radians(self._tick + i * 100))
            self._plot_data_xyz[i].append(val)
            if len(self._plot_data_xyz[i]) > 360:
                self._plot_data_xyz[i].pop(0)
            self._models["timeseries_plot_xyz"][i].set_data(*self._plot_data_xyz[i])
            self._models["timeseries_plot_xyz_vals"][i].set_value(val)

    def build_plot_frame(self):
        self._plot_frame = ui.CollapsableFrame(
            title="Example: Plotting",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._plot_frame:
            with ui.VStack(spacing=5, height=0):
                data = []
                for i in range(360):
                    data.append(math.cos(math.radians(i)))
                kwargs = {"label": "Static Plot", "color": 0xFF1A58FF, "data": data}  # ORANGE
                plot_builder(**kwargs)

                plots = []
                for j in range(3):
                    data = []
                    for i in range(360):
                        data.append(math.cos(math.radians(i + j * 100)))
                    plots.append(data)
                xyz_plot_builder("Static XYZ Plot", plots)

                kwargs = {"label": "Time Series Plot", "on_clicked_fn": self.toggle_app_step, "data": self._plot_data}
                self._models["timeseries_plot"], self._models["timeseries_plot_val"] = combo_cb_plot_builder(**kwargs)

                kwargs = {
                    "label": "Time Series Bar Plot",
                    "on_clicked_fn": self.toggle_app_step_1,
                    "type": ui.Type.HISTOGRAM,
                    "value_stride": 15,
                    "min": -5,
                    "data": self._plot_data,
                }
                self._models["timeseries_plot_hist"], self._models["timeseries_plot_hist_val"] = combo_cb_plot_builder(
                    **kwargs
                )

                kwargs = {
                    "label": "Time Series XYZ Plot",
                    "on_clicked_fn": self.toggle_app_step_2,
                    "data": self._plot_data_xyz,
                }
                (
                    self._models["timeseries_plot_xyz"],
                    self._models["timeseries_plot_xyz_vals"],
                ) = combo_cb_xyz_plot_builder(**kwargs)

    def build_search_frame(self):
        self._search_frame = ui.CollapsableFrame(
            title="Example: Search",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        with self._search_frame:
            list = ["the", "quick", "brown", "fox", "Hello", "World"]
            self._list_item_model = SearchListItemModel(*list)
            self._list_item_delegate = SearchListItemDelegate(self._on_dummy_callable_4)
            kwargs = {
                "label": "Simple Search",
                "type": "search",
                "model": self._list_item_model,
                "delegate": self._list_item_delegate,
                "tooltip": "Search & Double Click to Select",
            }
            self._search_bar, self._search_treeview = build_simple_search(**kwargs)

    def build_folder_picker_frame(self):
        self._folder_picker_frame = ui.CollapsableFrame(
            title="Example: Folder Picker",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        with self._folder_picker_frame:
            with ui.VStack(spacing=5):
                kwargs = {
                    "label": "Output Directory",
                    "type": "stringfield",
                    "default_val": "/home/",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "on_clicked_fn": self._on_dummy_callable_0,
                    "use_folder_picker": True,
                }
                str_builder(**kwargs)

    def handle_connect(self, val=False):
        if val:
            # do connect
            print("connecting")
        else:
            # do disconnect
            print("disconnecting")

    def build_comms_frame(self):
        self._comms_frame = ui.CollapsableFrame(
            title="Example: Communications",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._comms_frame:

            with ui.VStack(spacing=5):
                self._models["hostname"] = str_builder("Hostname", "stringfield", "127.0.0.1", "Host Address")
                self._models["port"] = str_builder("Port", "stringfield", "12345", "Port")
                self._models["connect_btn"] = state_btn_builder(
                    "", "button", "CONNECT", "DISCONNECT", "", self.handle_connect
                )

    def build_progress_bar_frame(self):
        self._progress_bar_frame = ui.CollapsableFrame(
            title="Example: Progress Bar",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._progress_bar_frame:

            def trigger_progress_bar():
                model.set_value(model.get_value_as_float() + 0.03)
                if model.get_value_as_float() > 1:
                    model.set_value(0)

            with ui.VStack(spacing=5):
                model = progress_bar_builder("Progress Bar")
                kwargs = {"label": "", "text": "GO", "on_clicked_fn": trigger_progress_bar}
                btn_builder(**kwargs)

    def build_custom_ui(self):
        """
        This is where the User creates their main GUI.
        Use a Group Frame to help visually differente user-generated vs core Isaac UI elements
        """
        self._my_ui = ui.CollapsableFrame(
            title="My Custom UI",
            height=0,
            collapsed=False,
            style=get_style(),
            name="groupFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._my_ui:
            with ui.VStack(spacing=VERTICAL_SPACING):
                ui.CollapsableFrame(
                    title="Parameter Group 1",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    name="subFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                ui.CollapsableFrame(
                    title="Parameter Group 2",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    name="subFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                ui.CollapsableFrame(
                    title="Parameter Group 3",
                    height=0,
                    collapsed=False,
                    style=get_style(),
                    name="subFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

    def _on_dummy_callable_0(self, val=None, val2=None):
        """Dummy Callable for testing the GUI"""
        if PRINT_DEBUG:
            print("You've cliked DUMMY CALLABLE 0:", val)

    def _on_dummy_callable_1(self, val=None):
        """Dummy Callable for testing the GUI"""
        if PRINT_DEBUG:
            print("You've cliked DUMMY CALLABLE 1:", val)

    def _on_dummy_callable_2(self, val=None):
        """Dummy Callable for testing the GUI"""
        if PRINT_DEBUG:
            print("You've cliked DUMMY CALLABLE 2:", val)

    def _on_dummy_callable_3(self, val=None):
        """Dummy Callable for testing the GUI"""
        if PRINT_DEBUG:
            print("You've cliked DUMMY CALLABLE 3. Item Selected: ", val)

    def _on_dummy_callable_4(self, model, button, val=None):
        """Dummy Callable for testing the GUI"""
        if PRINT_DEBUG:
            print("You've cliked DUMMY CALLABLE 4. Item Selected: ", val.text)
