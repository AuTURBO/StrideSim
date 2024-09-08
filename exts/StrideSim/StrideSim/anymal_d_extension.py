import os

from StrideSim.base_sample import BaseSampleExtension
from StrideSim.anymal_d import AnymalD


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