#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

"""
BLK ARC LiDAR Service

This service allows you to control the BLK ARC LiDAR from a remote machine.
It can also be used during autowalk missions to capture scans.
"""

import logging
import sys
import time
from pathlib import Path
from typing import Tuple

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import service_customization_pb2
from bosdyn.api.mission import remote_pb2, remote_service_pb2_grpc
from bosdyn.client.directory_registration import (DirectoryRegistrationClient, DirectoryRegistrationKeepAlive)
from bosdyn.client.server_util import GrpcServiceRunner, ResponseContext
from bosdyn.client.service_customization_helpers import (InvalidCustomParamSpecError, validate_dict_spec)
from bosdyn.client.util import setup_logging

import blk_arc_grpc.device_pb2 as device_message
from blk_arc_api.blk_arc import BLK_ARC
from blk_arc_api.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED
DIRECTORY_NAME = 'blk-arc'
AUTHORITY = 'remote-mission'
SERVICE_TYPE = 'bosdyn.api.mission.RemoteMissionService'
DOWNLOAD_PATH = Path.cwd() / "scans"

_LOGGER = logging.getLogger(__name__)
_ACTION_KEY = 'action'


class BlkArcServicer(remote_service_pb2_grpc.RemoteMissionServiceServicer):
    """
    Servicer to handle remote mission service calls for the LiDAR.
    """

    def __init__(self, logger=None):
        self.logger = logger or _LOGGER

        # Create the custom parameters.
        self.custom_params = service_customization_pb2.DictParam.Spec()
        action_param = service_customization_pb2.StringParam.Spec()
        action_param.default_value = "status"
        action_param.editable = True
        action_ui_info = service_customization_pb2.UserInterfaceInfo()
        action_ui_info.display_name = "Action"
        action_ui_info.description = "Command to send to the LiDAR"
        dict_spec = service_customization_pb2.DictParam.Spec()
        dict_spec.specs[_ACTION_KEY].spec.string_spec.CopyFrom(action_param)
        dict_spec.specs[_ACTION_KEY].ui_info.CopyFrom(action_ui_info)
        self.custom_params.CopyFrom(dict_spec)
        try:
            # Validate the custom parameters.
            validate_dict_spec(self.custom_params)
        except InvalidCustomParamSpecError as e:
            self.logger.info(e)
            # Clear the custom parameters if they are invalid.
            self.custom_params.Clear()

        self.blk_arc = BLK_ARC()
        # Connect:
        if not self.blk_arc.connect(CONNECTION_TYPE):
            self.logger.warning("Could not connect, make sure you selected the right connection-type.")
            return

            # Get Device Info:
        device_info = self.blk_arc.get_device_info()
        if device_info:
            self.logger.info("\nDevice info:\n"
                             f"  Device Name:   {device_info.device_name}\n"
                             f"  Serial Number: {device_info.serial_number}")
        else:
            self.logger.warning("Could not get device-info.")

        # Get Firmware Version:
        firmware_version = self.blk_arc.get_firmware_version()
        if firmware_version:
            self.logger.info(f"Firmware version: {firmware_version}")
        else:
            self.logger.warning("Could not get firmware-version.")

        # Get Device Status:
        device_status = self.blk_arc.get_device_status()
        if device_status:
            device_states = device_message.DeviceStateResponse.State
            self.logger.info(f"Device is in state '{device_states.Name(device_status.state)}'.")
        else:
            self.logger.warning("Could not get device-status.")

        # Get Disk Information
        free_disk_space = self.blk_arc.get_free_disk_space_percentage()
        if free_disk_space:
            self.logger.info(f"Free disk space: {free_disk_space:.2f}%")
        else:
            self.logger.warning("Could not get free disk space information.")

    def Tick(self, request, context):
        """Logs text, then provides a valid response."""
        response = remote_pb2.TickResponse()
        # This utility context manager will fill out some fields in the message headers.
        with ResponseContext(response, request):
            # Default to saying hello to 'World'.
            action = 'status'
            action_param = request.params.values.get(_ACTION_KEY)
            if action_param is not None:
                action = action_param.string_value.value
            self.logger.info(f"Action: {action}")

            # do action
            result = self._action(action)
            response.string_response = result[1]
            response.status = result[0]
        return response

    def _action(self, action) -> Tuple[remote_pb2.TickResponse, str]:
        # match action
        if action == "status":
            return self._status()
        elif action == "capture":
            return self._capture()
        elif action == "stop-capture":
            return self._stop_capture()
        elif action.startswith("download"):
            return self._download(action[9:])

        return remote_pb2.TickResponse.MISSING_INPUTS, f"Unknown action '{action}'"

    def _status(self):
        device_status = self.blk_arc.get_device_status()
        if device_status:
            device_states = device_message.DeviceStateResponse.State
            return remote_pb2.TickResponse.STATUS_SUCCESS, f"Device is in state '{device_states.Name(device_status.state)}'."

    def _capture(self):
        self.logger.info("Starting a scan. Do not move the LiDAR.")
        start_scan_info = self.blk_arc.start_capture()
        self.blk_arc.begin_static_pose()

        if start_scan_info:
            # Wait for the scan to be initialized, up to 10 seconds:
            attempts = 0
            while not self.blk_arc.is_scanning() and attempts < 20:
                time.sleep(0.5)
                attempts += 1
            if not self.blk_arc.is_scanning():
                return remote_pb2.TickResponse.STATUS_FAILURE, "Could not start a scan."

            self.logger.info(f"Initialized scan with ID: {start_scan_info.scan_id}.")
            return remote_pb2.TickResponse.STATUS_SUCCESS, str(start_scan_info.scan_id)

        return remote_pb2.TickResponse.STATUS_FAILURE, "Could not start a scan."

    def _stop_capture(self):
        # check if scanning
        if not self.blk_arc.is_scanning():
            return remote_pb2.TickResponse.STATUS_FAILURE, "Cannot stop capture while not scanning"

        # Stop the scan
        self.logger.info("Stopping capture.")
        self.blk_arc.end_static_pose()
        res = self.blk_arc.stop_capture()
        if res is None:
            return remote_pb2.TickResponse.STATUS_FAILURE, "Could not stop the scan"

        self.logger.info("Stopped capture.")
        return remote_pb2.TickResponse.STATUS_SUCCESS, res.duration

    def _download(self, scan_id: str):
        # check if scanning
        if self.blk_arc.is_scanning():
            return remote_pb2.TickResponse.STATUS_FAILURE, "Cannot download while scanning"

        # Download the scan
        self.logger.info("Downloading the scan.")
        if not self.blk_arc.download_scan(int(scan_id), DOWNLOAD_PATH):
            return remote_pb2.TickResponse.STATUS_FAILURE, "Could not download the scan"
        self.logger.info("Downloaded the scan.")
        return remote_pb2.TickResponse.STATUS_SUCCESS, str(DOWNLOAD_PATH / f"{scan_id}.blk")


def EstablishSession(self, request, _context):
    response = remote_pb2.EstablishSessionResponse()
    with ResponseContext(response, request):
        self.logger.info('EstablishSession unimplemented!')
        response.status = remote_pb2.EstablishSessionResponse.STATUS_OK
    return response


def Stop(self, request, _context):
    response = remote_pb2.StopResponse()
    with ResponseContext(response, request):
        self.logger.info('Stop unimplemented!')
        response.status = remote_pb2.StopResponse.STATUS_OK
    return response


def TeardownSession(self, request, _context):
    response = remote_pb2.TeardownSessionResponse()
    with ResponseContext(response, request):
        self.logger.info('TeardownSession unimplemented!')
        response.status = remote_pb2.TeardownSessionResponse.STATUS_OK
    return response


def GetRemoteMissionServiceInfo(self, request, _context):
    response = remote_pb2.GetRemoteMissionServiceInfoResponse()
    with ResponseContext(response, request):
        response.custom_params.CopyFrom(self.custom_params)
    return response


def run_service(port, logger=None):
    # Proto service specific function used to attach a servicer to a server.
    add_servicer_to_server_fn = remote_service_pb2_grpc.add_RemoteMissionServiceServicer_to_server

    # Instance of the servicer to be run.
    service_servicer = BlkArcServicer(logger=logger)
    return GrpcServiceRunner(service_servicer, add_servicer_to_server_fn, port, logger=logger)


if __name__ == '__main__':
    # Define all arguments used by this service.
    import argparse

    # Create the top-level parser.
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='Select how this service will be accessed.', dest='host_type')

    # Create the parser for the "local" command.
    local_parser = subparsers.add_parser('local', help='Run this example locally.')
    bosdyn.client.util.add_service_hosting_arguments(local_parser)

    # Create the parser for the "robot" command.
    robot_parser = subparsers.add_parser('robot', help='Run this example with a robot in the loop.')
    bosdyn.client.util.add_base_arguments(robot_parser)
    bosdyn.client.util.add_service_endpoint_arguments(robot_parser)

    options = parser.parse_args()

    # If using the example without a robot in the loop, start up the service, which can be
    # be accessed directly at localhost:options.port.
    if options.host_type == 'local':
        # Setup logging to use INFO level.
        setup_logging()
        service_runner = run_service(options.port, logger=_LOGGER)
        print(f'{DIRECTORY_NAME} service running.\nCtrl + C to shutdown.')
        service_runner.run_until_interrupt()
        sys.exit(f'Shutting down {DIRECTORY_NAME} service')

    # Else if a robot is available, register the service with the robot so that all clients can
    # access it through the robot directory without knowledge of the service IP or port.

    # Setup logging to use either INFO level or DEBUG level.
    setup_logging(options.verbose)

    # Create and authenticate a bosdyn robot object.
    sdk = bosdyn.client.create_standard_sdk('HelloWorldMissionServiceSDK')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    # Create a service runner to start and maintain the service on background thread.
    service_runner = run_service(options.port, logger=_LOGGER)

    # Use a keep alive to register the service with the robot directory.
    dir_reg_client = robot.ensure_client(DirectoryRegistrationClient.default_service_name)
    keep_alive = DirectoryRegistrationKeepAlive(dir_reg_client, logger=_LOGGER)
    keep_alive.start(DIRECTORY_NAME, SERVICE_TYPE, AUTHORITY, options.host_ip, service_runner.port)

    # Attach the keep alive to the service runner and run until a SIGINT is received.
    with keep_alive:
        service_runner.run_until_interrupt()
