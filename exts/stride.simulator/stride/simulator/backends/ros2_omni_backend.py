"""
This is omni graph backend for ros2
"""

from stride.simulator.backends.backend import Backend
import omni.graph.core as og


class ROS2OmniBackend(Backend):
    """
    A class representing the ROS2 Omni Backend.

    This backend is responsible for initializing and configuring the ROS2 integration
    with the Omni platform.

    Args:
        prim_path (dict): A dictionary containing the paths for the lidar, IMU, and controller.

    Attributes:
        _prim_path (dict): The initialized primary path dictionary.
        keys (og.Controller.Keys): The keys for the OmniGraph controller.

    Methods:
        prim_path_init: Initializes the primary path dictionary with default values.
        initialize_omnigraph: Sets up the OmniGraph controller and configures the graph.

    """

    def __init__(self, prim_path: dict):
        super().__init__()

        self._prim_path = self.prim_path_init(prim_path)
        self.keys = og.Controller.Keys
        self.initialize_omnigraph()

    def prim_path_init(self, prim_path: dict):
        """
        Initializes the primary path dictionary with default values.

        Args:
            prim_path (dict): The primary path dictionary.

        Returns:
            dict: The initialized primary path dictionary.

        """
        if not "Lidar_path" in prim_path.keys():
            prim_path["Lidar_path"] = "/World/Go1/lidar/lidar_PhysX"

        if not "Imu_path" in prim_path.keys():
            prim_path["Imu_path"] = "/World/Go1/imu_link/Imu_Sensor"

        if not "TF_path" in prim_path.keys():
            prim_path["TF_path"] = "/World/Go1"

        return prim_path

    def initialize_omnigraph(self):
        """
        Sets up the OmniGraph controller and configures the graph.

        """
        # setup graph
        og.Controller.edit(
            {"graph_path": "/action_graph", "evaluator_name": "execution"},
            {
                self.keys.CREATE_NODES: [
                    ("PTick", "omni.graph.action.OnPlaybackTick"),
                    ("Sim_Time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Ros_Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("Imu_Read", "omni.isaac.sensor.IsaacReadIMU"),
                    ("Ros2_Imu_Pub", "omni.isaac.ros2_bridge.ROS2PublishImu"),
                    ("Lidar_Read", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                    ("Ros2_Lidar_Pub", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
                    ("Ros2_tf_pub", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                self.keys.SET_VALUES: [
                    ("Ros_Context.inputs:useDomainIDEnvVar", False),
                    ("Imu_Read.inputs:imuPrim", self._prim_path["Imu_path"]),
                    ("Lidar_Read.inputs:lidarPrim", self._prim_path["Lidar_path"]),
                    ("Imu_Read.inputs:readGravity", True),
                    ("Ros2_Imu_Pub.inputs:frameId", "sim_imu"),
                    ("Ros2_Imu_Pub.inputs:topicName", "imu"),
                    ("Ros2_Lidar_Pub.inputs:frameId", "sim_lidar"),
                    ("Ros2_Lidar_Pub.inputs:topicName", "point_cloud"),
                    ("Ros2_tf_pub.inputs:targetPrims", self._prim_path["TF_path"]),
                ],
                self.keys.CONNECT: [
                    ("PTick.outputs:tick", "Imu_Read.inputs:execIn"),
                    ("Ros_Context.outputs:context", "Ros2_Imu_Pub.inputs:context"),
                    ("Imu_Read.outputs:execOut", "Ros2_Imu_Pub.inputs:execIn"),
                    (
                        "Imu_Read.outputs:angVel",
                        "Ros2_Imu_Pub.inputs:angularVelocity",
                    ),
                    (
                        "Imu_Read.outputs:linAcc",
                        "Ros2_Imu_Pub.inputs:linearAcceleration",
                    ),
                    (
                        "Imu_Read.outputs:orientation",
                        "Ros2_Imu_Pub.inputs:orientation",
                    ),
                    (
                        "Sim_Time.outputs:simulationTime",
                        "Ros2_Imu_Pub.inputs:timeStamp",
                    ),
                    ("PTick.outputs:tick", "Lidar_Read.inputs:execIn"),
                    ("Ros_Context.outputs:context", "Ros2_Lidar_Pub.inputs:context"),
                    ("Lidar_Read.outputs:execOut", "Ros2_Lidar_Pub.inputs:execIn"),
                    (
                        "Lidar_Read.outputs:data",
                        "Ros2_Lidar_Pub.inputs:data",
                    ),
                    (
                        "Sim_Time.outputs:simulationTime",
                        "Ros2_Lidar_Pub.inputs:timeStamp",
                    ),
                    ("PTick.outputs:tick", "Ros2_tf_pub.inputs:execIn"),
                    ("Ros_Context.outputs:context", "Ros2_tf_pub.inputs:context"),
                    (
                        "Sim_Time.outputs:simulationTime",
                        "Ros2_tf_pub.inputs:timeStamp",
                    ),
                ],
            },
        )
