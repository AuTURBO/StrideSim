import omni.ext
import subprocess


# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[StrideSim] some_public_function was called with x: ", x)
    return x**x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class ExampleExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[StrideSim] startup")

        self._window = omni.ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            with omni.ui.VStack():
                label = omni.ui.Label("")

                def on_train():
                    label.text = f"start training"
                    
                    # Execute the specified Python command
                    subprocess.run(["python", "scripts/rsl_rl/train.py", "--task", "Isaac-Velocity-Rough-Anymal-D-v0", "--headless"])
                    
                def on_play():
                    label.text = "empty"

                with omni.ui.HStack():
                    omni.ui.Button("Train", clicked_fn=on_train)
                    omni.ui.Button("Reset", clicked_fn=on_play)


    def on_shutdown(self):
        print("[StrideSim] shutdown")
