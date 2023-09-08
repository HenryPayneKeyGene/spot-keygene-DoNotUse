import bosdyn
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.robot_command
import bosdyn.client.robot_state
import bosdyn.client.util

from .estop import EStop


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

    def __init__(self, addr, name):
        self.addr = addr
        self.name = name

        self.state: str = 'startup'

        bosdyn.client.util.setup_logging()
        self.sdk = bosdyn.client.create_standard_sdk('keygene-client')
        self.robot = self.sdk.create_robot(self.addr, self.name)
        self.robot.logger.info("Starting up")

        self.robot.logger.debug("Authenticating")
        bosdyn.client.util.authenticate(self.robot)
        self.robot.logger.debug("Authentication OK")

        self.robot.start_time_sync()

        self.command_client = self.robot.ensure_client(
            bosdyn.client.robot_command.RobotCommandClient.default_service_name)
        self.state_client = self.robot.ensure_client(
            bosdyn.client.robot_state.RobotStateClient.default_service_name)
        self.lease_client = self.robot.ensure_client(
            bosdyn.client.lease.LeaseClient.default_service_name)
        self.estop_client = self.robot.ensure_client(
            bosdyn.client.estop.EstopClient.default_service_name)

        self._estop = EStop(self.estop_client, 15, f"estop-{self.name}")

        self.robot.logger.info("Spot initialized")
        self.acquire()
        self.robot.logger.info("Startup complete")

    def __del__(self):
        if self.state == 'ready':
            self.shutdown()

    def acquire(self):

        self.robot.logger.debug("Waiting for time sync...")
        self.robot.time_sync.wait_for_sync()
        self.robot.logger.debug("Time sync OK")

        self.robot.logger.debug("Acquiring lease...")
        self.lease = self.lease_client.acquire()
        self.robot.logger.debug("Lease acquired.")


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

        self.state = 'shutdown'
        self.robot.logger.warn("Shutdown complete")

    def power_on(self):
        self.robot.logger.info("Powering on...")
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), "Failed to power on"
        self.robot.logger.info("Power on complete")

    def power_off(self):
        self.robot.logger.info("Powering off...")
        self.robot.power_off(timeout_sec=20)
        assert not self.robot.is_powered_on(), "Failed to power off"
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
