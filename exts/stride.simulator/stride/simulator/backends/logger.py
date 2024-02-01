import carb

from stride.simulator.backends.backend import Backend


class Logger(Backend):
    """
    Logger backend that just prints the state of the vehicle to the console.
    """

    def __init__(self, vehicle_id: int):
        super().__init__()
        self._id = vehicle_id

    def update_state(self, state):
        """
        Method that handles the receival of the state of the vehicle. It just prints the state to the console.
        """
        carb.log_info(f"Received state update for vehicle {self._id}")
        carb.log_info(f"Position: {state.position}")
        carb.log_info(f"Attitude: {state.attitude}")
        carb.log_info(f"Linear velocity: {state.linear_velocity}")
        carb.log_info(f"Angular velocity: {state.angular_velocity}")
        carb.log_info(f"Linear acceleration: {state.linear_acceleration}")

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data
        """

        if sensor_type == "IMU":
            self.update_imu_data(data)

        # TODO: Add support for other sensors

    def update_imu_data(self, data):
        """
        Method that handles the receival of IMU data. It just prints the data to the console.
        """
        carb.log_info(f"Received IMU data for vehicle {self._id}")
        carb.log_info(f"Angular velocity: {data['angular_velocity']}")
        carb.log_info(f"Linear acceleration: {data['linear_acceleration']}")

    def update(self, dt: float):
        """
        Method that update the state of the backend and the information being sent/received from the communication
        interface. This method will be called by the simulation on every physics steps.
        """
        carb.log_info(f"Updating logger backend for vehicle {self._id}")
        carb.log_info(f"Time elapsed since last update: {dt}")