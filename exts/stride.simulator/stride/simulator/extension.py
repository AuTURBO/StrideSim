# TODO: 나중에 UI코드 부분을 빼서 ui folder에 정리해야 함

# Omniverse general API
import omni
import omni.ext
import omni.usd
import omni.kit.ui
import omni.kit.app
from omni import ui

from stride.simulator.interfaces.stride_sim_interface import StrideInterface
from stride.simulator.vehicles.quadrupedrobot.anymalc import AnymalC, AnymalCConfig
from stride.simulator.params import SIMULATION_ENVIRONMENTS

import asyncio


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

        self._window.deferred_dock_in(
            "Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE
        )

        # Start the extension backend
        self._stride_sim = StrideInterface()

        with self._window.frame:
            with ui.VStack():

                def on_world():

                    self._stride_sim.initialize_world()

                    label.text = "Initialize world"

                def on_environment():

                    self._stride_sim.load_asset(
                        SIMULATION_ENVIRONMENTS["Default Environment"], "/World/layout"
                    )

                    label.text = "Load environment"

                def on_simulation():
                    async def respawn():

                        self._anymal_config = AnymalCConfig()

                        self._anymal = AnymalC(
                            id=0,
                            init_pos=[0.0, 0.0, 0.7],
                            init_orientation=[0.0, 0.0, 0.0, 1.0],
                            config=self._anymal_config,
                        )

                        self._current_tasks = self._stride_sim.world.get_current_tasks()
                        await self._stride_sim.world.reset_async()
                        await self._stride_sim.world.pause_async()

                        if len(self._current_tasks) > 0:
                            self._stride_sim.world.add_physics_callback(
                                "tasks_step", self._world.step_async
                            )

                    asyncio.ensure_future(respawn())

                    label.text = "Load simulation"

                with ui.HStack():
                    ui.Button("Init World", clicked_fn=on_world)
                    ui.Button("Environment", clicked_fn=on_environment)
                    ui.Button("Simulation", clicked_fn=on_simulation)

                label = ui.Label("")

    def on_shutdown(self):
        print("[stride.simulator] stride simulator shutdown")
