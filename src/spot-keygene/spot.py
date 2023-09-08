import os

import bosdyn
import bosdyn.client
import bosdyn.client.docking
import bosdyn.client.estop
import bosdyn.client.graph_nav
import bosdyn.client.lease
import bosdyn.client.map_processing
import bosdyn.client.power
import bosdyn.client.recording
import bosdyn.client.robot_command
import bosdyn.client.robot_state
import bosdyn.client.util
import bosdyn.client.world_object

from .nav import GraphNavInterface
from .recording import RecordingInterface


class Spot:
    """
    A class to represent a Boston Dynamics Spot robot.

    This class is a wrapper around the Boston Dynamics SDK.
    """

    robot: bosdyn.client.Robot
    sdk: bosdyn.client.Sdk

    command_client: bosdyn.client.robot_command.RobotCommandClient
    state_client: bosdyn.client.robot_state.RobotStateClient
    lease_client: bosdyn.client.lease.LeaseClient

    lease: bosdyn.client.lease.Lease

    def __init__(self, addr, name, download_path=os.getcwd(), upload_path=os.getcwd()):
        self.powered_on = False
        self.addr = addr
        self.name = name

        self.state: str = "startup"

        bosdyn.client.util.setup_logging()
        self.sdk = bosdyn.client.create_standard_sdk("keygene-client")
        self.robot = self.sdk.create_robot(self.addr, self.name)
        self.robot.logger.info("Starting up")

        self.robot.logger.debug("Authenticating")
        bosdyn.client.util.authenticate(self.robot)
        self.robot.logger.debug("Authentication OK")

        self.robot.start_time_sync()

        self.command_client: bosdyn.client.robot_command.RobotCommandClient = (
            self.robot.ensure_client(
                bosdyn.client.robot_command.RobotCommandClient.default_service_name
            )
        )
        self.state_client: bosdyn.client.robot_state.RobotStateClient = (
            self.robot.ensure_client(
                bosdyn.client.robot_state.RobotStateClient.default_service_name
            )
        )
        self.lease_client: bosdyn.client.lease.LeaseClient = self.robot.ensure_client(
            bosdyn.client.lease.LeaseClient.default_service_name
        )
        self.estop_client: bosdyn.client.estop.EstopClient = self.robot.ensure_client(
            bosdyn.client.estop.EstopClient.default_service_name
        )
        self.graph_nav_client: bosdyn.client.graph_nav.GraphNavClient = (
            self.robot.ensure_client(
                bosdyn.client.graph_nav.GraphNavClient.default_service_name
            )
        )
        self.recording_client: bosdyn.client.recording.GraphNavRecordingServiceClient = self.robot.ensure_client(
            bosdyn.client.recording.GraphNavRecordingServiceClient.default_service_name
        )
        self.world_object_client: bosdyn.client.world_object.WorldObjectClient = (
            self.robot.ensure_client(
                bosdyn.client.world_object.WorldObjectClient.default_service_name
            )
        )
        self.docking_client: bosdyn.client.docking.DockingClient = (
            self.robot.ensure_client(
                bosdyn.client.docking.DockingClient.default_service_name
            )
        )
        self.map_processing_client: bosdyn.client.map_processing.MapProcessingServiceClient = self.robot.ensure_client(
            bosdyn.client.map_processing.MapProcessingServiceClient.default_service_name
        )
        self.power_client: bosdyn.client.power.PowerClient = self.robot.ensure_client(
            bosdyn.client.power.PowerClient.default_service_name
        )

        # self._estop = EStop(self.estop_client, 15, f"estop-{self.name}")

        client_metadata = (
            bosdyn.client.recording.GraphNavRecordingServiceClient.make_client_metadata(
                session_name=os.path.basename(os.getcwd()),
                client_username=self.robot._current_user,
                client_id="spot-keygene",
                client_type="Python SDK",
            )
        )
        self.recording_interface = RecordingInterface(
            self.robot, download_path, self.recording_client, self.graph_nav_client, self.map_processing_client,
            client_metadata
        )
        self.graph_nav_interface = GraphNavInterface(self.robot, upload_path, self.command_client, self.state_client,
                                                     self.graph_nav_client, self.power_client)

        # self.robot_state_task = AsyncRobotState(self.state_client)
        # self.async_tasks = AsyncTasks([self.robot_state_task])
        # self.async_tasks.update()

        self.robot.logger.info("Spot initialized")
        self.acquire()
        self.robot.logger.info("Startup complete")

    def __del__(self):
        if self.state == "ready":
            self.shutdown()

    def acquire(self):
        self.robot.logger.debug("Waiting for time sync...")
        self.robot.time_sync.wait_for_sync()
        self.robot.logger.debug("Time sync OK")

        self.robot.logger.debug("Acquiring lease...")
        self.lease = self.lease_client.acquire()
        self.robot.logger.debug(f"Lease acquired.")

        if not hasattr(self, "_estop"):
            self.robot.logger.warn(
                "EStop not configured -- please use an external EStop client."
            )

    def shutdown(self):
        self.robot.logger.warn("Shutting down...")

        if self.robot.is_powered_on():
            self.power_off()

        if self.lease:
            self.robot.logger.debug("Releasing lease...")
            self.lease_client.return_lease(self.lease)
            self.robot.logger.debug("Lease released")

        self.robot.logger.debug("Stopping time sync...")
        self.robot.time_sync.stop()
        self.robot.logger.debug("Time sync stopped")

        self.state = "shutdown"
        self.robot.logger.warn("Shutdown complete")

    def power_on(self):
        if self.robot.is_powered_on():
            return
        self.robot.logger.info("Powering on...")
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), "Failed to power on"
        self.powered_on = True
        self.robot.logger.info("Power on complete")

    def power_off(self):
        if not self.robot.is_powered_on():
            return
        self.robot.logger.info("Powering off...")
        self.robot.power_off(timeout_sec=20)
        assert not self.robot.is_powered_on(), "Failed to power off"
        self.powered_on = False
        self.robot.logger.info("Power off complete")

    def estop(self):
        self.robot.logger.warn("ESTOP!")
        self._estop.stop()

    def stand(self):
        self.robot.logger.info("Stand requested")
        bosdyn.client.robot_command.blocking_stand(self.command_client, timeout_sec=5)
        self.robot.logger.info("Standing")

    def sit(self):
        self.robot.logger.info("Sit requested")
        bosdyn.client.robot_command.blocking_sit(self.command_client, timeout_sec=5)
        self.robot.logger.info("Sitting")
