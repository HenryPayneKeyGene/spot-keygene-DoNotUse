# TODO - This is a copy of the example service from the bosdyn SDK. We need to modify this to
#  implement the functionality we need.

"""Example of how to run a RemoteMissionService servicer."""

import logging
import sys

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import service_customization_pb2
from bosdyn.api.mission import remote_pb2, remote_service_pb2_grpc
from bosdyn.client.directory_registration import (DirectoryRegistrationClient,
                                                  DirectoryRegistrationKeepAlive)
from bosdyn.client.server_util import GrpcServiceRunner, ResponseContext
from bosdyn.client.service_customization_helpers import (InvalidCustomParamSpecError,
                                                         validate_dict_spec)
from bosdyn.client.util import setup_logging

DIRECTORY_NAME = 'hello-world-callback'
AUTHORITY = 'remote-mission'
SERVICE_TYPE = 'bosdyn.api.mission.RemoteMissionService'

_LOGGER = logging.getLogger(__name__)
_WHO_KEY = 'who'


class HelloWorldServicer(remote_service_pb2_grpc.RemoteMissionServiceServicer):
    """Every tick, logs 'Hello world!'

    Shows an example of these concepts:
     - Ticking.
     - Using inputs.
    """

    def __init__(self, logger=None):
        self.logger = logger or _LOGGER
        # Create the custom parameters.
        self.custom_params = service_customization_pb2.DictParam.Spec()
        who_param = service_customization_pb2.StringParam.Spec()
        who_param.default_value = "World"
        who_param.editable = True
        who_ui_info = service_customization_pb2.UserInterfaceInfo()
        who_ui_info.display_name = "Name"
        who_ui_info.description = "Who Spot will say hello to"
        dict_spec = service_customization_pb2.DictParam.Spec()
        dict_spec.specs[_WHO_KEY].spec.string_spec.CopyFrom(who_param)
        dict_spec.specs[_WHO_KEY].ui_info.CopyFrom(who_ui_info)
        self.custom_params.CopyFrom(dict_spec)
        try:
            # Validate the custom parameters.
            validate_dict_spec(self.custom_params)
        except InvalidCustomParamSpecError as e:
            self.logger.info(e)
            # Clear the custom parameters if they are invalid.
            self.custom_params.Clear()

    def Tick(self, request, context):
        """Logs text, then provides a valid response."""
        response = remote_pb2.TickResponse()
        # This utility context manager will fill out some fields in the message headers.
        with ResponseContext(response, request):
            # Default to saying hello to 'World'.
            name = 'World'
            who = request.params.values.get(_WHO_KEY)
            if who is not None:
                name = who.string_value.value
            self.logger.info('Hello %s!', name)
            response.status = remote_pb2.TickResponse.STATUS_SUCCESS
        return response

    def EstablishSession(self, request, context):
        response = remote_pb2.EstablishSessionResponse()
        with ResponseContext(response, request):
            self.logger.info('EstablishSession unimplemented!')
            response.status = remote_pb2.EstablishSessionResponse.STATUS_OK
        return response

    def Stop(self, request, context):
        response = remote_pb2.StopResponse()
        with ResponseContext(response, request):
            self.logger.info('Stop unimplemented!')
            response.status = remote_pb2.StopResponse.STATUS_OK
        return response

    def TeardownSession(self, request, context):
        response = remote_pb2.TeardownSessionResponse()
        with ResponseContext(response, request):
            self.logger.info('TeardownSession unimplemented!')
            response.status = remote_pb2.TeardownSessionResponse.STATUS_OK
        return response

    def GetRemoteMissionServiceInfo(self, request, context):
        response = remote_pb2.GetRemoteMissionServiceInfoResponse()
        with ResponseContext(response, request):
            response.custom_params.CopyFrom(self.custom_params)
        return response


def run_service(port, logger=None):
    # Proto service specific function used to attach a servicer to a server.
    add_servicer_to_server_fn = remote_service_pb2_grpc.add_RemoteMissionServiceServicer_to_server

    # Instance of the servicer to be run.
    service_servicer = HelloWorldServicer(logger=logger)
    return GrpcServiceRunner(service_servicer, add_servicer_to_server_fn, port, logger=logger)


if __name__ == '__main__':
    # Define all arguments used by this service.
    import argparse

    # Create the top-level parser.
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='Select how this service will be accessed.',
                                       dest='host_type')

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