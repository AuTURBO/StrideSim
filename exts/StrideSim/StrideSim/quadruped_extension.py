import os

from .base_sample import BaseSampleExtension
from .quadruped import QuadrupedExample


class StrideSimExtension(BaseSampleExtension):
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
            name="StrideSim",
            title="Auturbo quadruped robot simulation",
            doc_link="https://github.com/AuTURBO/StrideSim",
            overview=overview,
            file_path=os.path.abspath(__file__),
            sample=QuadrupedExample(),
        )
        return


# import subprocess

# import omni.ext


# # Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
# def some_public_function(x: int):
#     print("[StrideSim] some_public_function was called with x: ", x)
#     return x**x


# # Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# # instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# # on_shutdown() is called.
# class ExampleExtension(omni.ext.IExt):
#     # ext_id is current extension id. It can be used with extension manager to query additional information, like where
#     # this extension is located on filesystem.
#     def on_startup(self, ext_id):
#         print("[StrideSim] startup")

#         self._window = omni.ui.Window("My Window", width=300, height=300)
#         with self._window.frame:
#             with omni.ui.VStack():
#                 label = omni.ui.Label("")

#                 def on_train():
#                     label.text = "start training"

#                     # Execute the specified Python command
#                     subprocess.run(
#                         [
#                             "python",
#                             "scripts/rsl_rl/train.py",
#                             "--task",
#                             "Isaac-Velocity-Rough-Anymal-D-v0",
#                             "--headless",
#                         ]
#                     )

#                 def on_play():
#                     label.text = "empty"

#                 with omni.ui.HStack():
#                     omni.ui.Button("Train", clicked_fn=on_train)
#                     omni.ui.Button("Reset", clicked_fn=on_play)

#     def on_shutdown(self):
#         print("[StrideSim] shutdown")
