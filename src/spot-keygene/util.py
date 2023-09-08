import logging

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot_state import RobotStateClient

LOGGER = logging.getLogger()


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client: RobotStateClient):
        super().__init__("robot_state", robot_state_client, LOGGER, period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()
