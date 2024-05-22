__all__ = ["Imu2"]

from stride.simulator.vehicles import State
from stride.simulator.vehicles.sensors.sensor import Sensor

from omni.isaac.core import World
from omni.isaac.sensor import IMUSensor
from omni.isaac.sensor import _sensor

import numpy as np


class Imu2(Sensor):
    """
    The class that implements the Imu sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config=None):
        """
        Initialize the imu class

        Args:
        """
        if config is None:
            config = {}
        else:
            assert isinstance(
                config, dict
            ), "The config parameter must be a dictionary."

        self.config = config

        super().__init__(
            sensor_type="Imu2",
            update_frequency=self.config.get("update_frequency", 20.0),
        )

        # Save the state of the sensor
        self._state = {
            "imu": [],
        }

        self.generate_imu_sensor()

        self._imu_sensor_interface = _sensor.acquire_imu_sensor_interface()

    def generate_imu_sensor(self):

        my_world = World.instance()

        self._imu = my_world.scene.add(
            IMUSensor(
                prim_path=self.config.get("prim_path"),
                name="imu",
                frequency=50,
                translation=np.array([0, 0, 0]),
                orientation=np.array([1, 0, 0, 0]),
                linear_acceleration_filter_size=10,
                angular_velocity_filter_size=10,
                orientation_filter_size=10,
            )
        )

    @property
    def state(self):
        return self._state

    @Sensor.update_at_frequency
    def update(self, state: State, dt: float):

        imu_data = self._imu_sensor_interface.get_sensor_reading(
            self.config.get("prim_path"), use_latest_data=True, read_gravity=True
        )
        self._state["imu"] = imu_data

        return self._state
