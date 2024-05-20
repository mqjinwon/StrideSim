"""
This is omni graph backend for ros2
"""

from stride.simulator.backends.backend import Backend
import omni.graph.core as og


class ROS2OmniBackend(Backend):
    def __init__(self, prim_path: dict):
        super().__init__()

        self._prim_path = prim_path
        self.keys = og.Controller.Keys
        self.initialize_omnigraph()

    def initialize_omnigraph(self):

        # setup graph
        og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                self.keys.CREATE_NODES: [
                    ("PTick", "omni.graph.action.OnPlaybackTick"),
                    ("Sim_Time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ],
                self.keys.SET_VALUES: [
                    # ("A.inputs:key", "A"),
                    # ("D.inputs:key", "D"),
                    # ("OnTick.inputs:onlyPlayback", True),  # only tick when simulator is playing
                    # ("NegOne.inputs:value", -1),
                    # ("CubeWrite.inputs:name", "size"),
                    # ("CubeWrite.inputs:primPath", "/Cube"),
                    # ("CubeWrite.inputs:usePath", True),
                    # ("CubeRead.inputs:name", "size"),
                    # ("CubeRead.inputs:primPath", "/Cube"),
                    # ("CubeRead.inputs:usePath", True),
                ],
                self.keys.CONNECT: [
                    # ("OnTick.outputs:tick", "CubeWrite.inputs:execIn"),
                    # ("A.outputs:isPressed", "ToDouble1.inputs:value"),
                    # ("D.outputs:isPressed", "ToDouble2.inputs:value"),
                    # ("ToDouble2.outputs:converted", "Negate.inputs:a"),
                    # ("NegOne.inputs:value", "Negate.inputs:b"),
                    # ("ToDouble1.outputs:converted", "DeltaAdd.inputs:a"),
                    # ("Negate.outputs:product", "DeltaAdd.inputs:b"),
                    # ("DeltaAdd.outputs:sum", "SizeAdd.inputs:a"),
                    # ("CubeRead.outputs:value", "SizeAdd.inputs:b"),
                    # ("SizeAdd.outputs:sum", "CubeWrite.inputs:value"),
                ],
            },
        )
