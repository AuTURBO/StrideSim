# TODO: 나중에 UI코드 부분을 빼서 ui folder에 정리해야 함

# Python garbage collenction and asyncronous API
from threading import Timer

# Omniverse general API
import pxr
import omni.ext
import omni.usd
import omni.kit.ui
import omni.kit.app
from omni import ui

from omni.kit.viewport.utility import get_active_viewport

from stride.simulator.interfaces.stride_sim_interface import StrideInterface
from stride.simulator.vehicles.quadrupedrobot.anymalc import AnymalC, AnymalCConfig
from stride.simulator.params import SIMULATION_ENVIRONMENTS

import asyncio
import carb

# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[stride.simulator] some_public_function was called with x: ", x)
    return x**x


class StrideSimulatorExtension(omni.ext.IExt):
    """Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will
    be instantiated when a extension gets enabled and `on_startup(ext_id)` will be called. Later when the extension gets
    disabled on_shutdown() is called."""

    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[stride.simulator] stride simulator startup")

        self._window = ui.Window("Stride Simulator", width=300, height=300)

        # Start the extension backend
        self._stride_sim = StrideInterface()

        with self._window.frame:
            with ui.VStack():
                label = ui.Label("")

                def on_click():
                    # Check if we already have a stage loaded (when using autoload feature, it might not be ready yet)
                    # This is a limitation of the simulator, and we are doing this to make sure that the
                    # extension does no crash when using the GUI with autoload feature
                    # If autoload was not enabled, and we are enabling the extension from the Extension widget, then
                    # we will always have a state open, and the auxiliary timer will never run

                    if omni.usd.get_context().get_stage_state() != omni.usd.StageState.CLOSED:
                        self._stride_sim.initialize_world()
                    else:
                        # We need to create a timer to check until the window is properly open and the stage created.
                        # This is a limitation of the current Isaac Sim simulator and the way it loads extensions :(
                        self.autoload_helper()

                    label.text = "Initialize world"
                    
                    asyncio.ensure_future(self._stride_sim.load_environment_async(
                        SIMULATION_ENVIRONMENTS["Default Environment"], force_clear=True))

                def on_spawn():
                    
                    async def async_load_vehicle():

                        # Check if we already have a physics environment activated. If not, then activate it
                        # and only after spawn the vehicle. This is to avoid trying to spawn a vehicle without a physics
                        # environment setup. This way we can even spawn a vehicle in an empty world and it won't care
                        if hasattr(self._stride_sim.world, "_physics_context") == False:
                                await self._stride_sim.world.initialize_simulation_context_async()

                        self._anymal_config = AnymalCConfig()

                        self._anymal = AnymalC(id=ext_id, init_pos=[0.0, 0.0, 0.5],init_orientation=[0.0, 0.0, 0.0, 1.0],
                                        config=self._anymal_config)

                    # Run the actual vehicle spawn async so that the UI does not freeze
                    asyncio.ensure_future(async_load_vehicle())

                    label.text = "Load vehicle"

                with ui.HStack():
                    ui.Button("Init", clicked_fn=on_click)
                    ui.Button("Env", clicked_fn=on_spawn)

    def autoload_helper(self):
        # Check if we already have a viewport and a camera of interest
        if get_active_viewport() is not None and isinstance(get_active_viewport().stage) == pxr.Usd.Stage and str(
                get_active_viewport().stage.GetPrimAtPath("/OmniverseKit_Persp")) != "invalid null prim":
            self._stride_sim.initialize_world()
        else:
            Timer(0.1, self.autoload_helper).start()

    def on_shutdown(self):
        print("[stride.simulator] stride simulator shutdown")
