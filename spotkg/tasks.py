#  Copyright (c) Romir Kulshrestha 2023.

import logging
import time

from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.robot_state import RobotStateClient

UPDATE_INTERVAL = 0.1  # s


def update_tasks(async_tasks: AsyncTasks):
    """Update all async tasks."""
    while True:
        async_tasks.update()
        time.sleep(UPDATE_INTERVAL)


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client: RobotStateClient, logger=logging.getLogger(__name__)):
        super().__init__("robot_state", robot_state_client, logger, period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


class AsyncImage(AsyncPeriodicQuery):
    """Grab image."""

    def __init__(self, image_client, image_sources, logger=logging.getLogger(__name__)):
        # Period is set to be about 15 FPS
        super(AsyncImage, self).__init__('images', image_client, logger, period_sec=0.067)
        self.image_sources = image_sources

    def _start_query(self):
        return self._client.get_image_from_sources_async(self.image_sources)
