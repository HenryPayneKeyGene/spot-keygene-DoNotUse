#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import threading
import time

import bosdyn
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client
import bosdyn.mission.client
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.docking import DockingClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.map_processing import MapProcessingServiceClient as MapClient
from bosdyn.client.power import PowerClient
from bosdyn.client.recording import GraphNavRecordingServiceClient as RecordingClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_sit, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.util import setup_logging
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.util import duration_str, secs_to_hms

from .globals import VELOCITY_BASE_ANGULAR, VELOCITY_BASE_SPEED, VELOCITY_CMD_DURATION
from .nav import GraphNavInterface
from .recording import RecorderInterface
from .util import AsyncRobotState


class Spot:
    """
    A class to represent a Boston Dynamics Spot robot.

    This class is a wrapper around the Boston Dynamics SDK.
    """

    robot: bosdyn.client.Robot
    sdk: bosdyn.client.Sdk

    command_client: RobotCommandClient
    state_client: RobotStateClient
    lease_client: LeaseClient

    lease_keep_alive: LeaseKeepAlive

    def __init__(self, config: dict):
        self.powered_on = False
        self.addr = config["addr"]
        self.name = config["name"]

        self.state: str = "startup"

        setup_logging()
        self.sdk = bosdyn.client.create_standard_sdk("keygene-client")
        self.robot = self.sdk.create_robot(self.addr, self.name)
        self.logger = self.robot.logger
        self.logger.info("Starting up")

        bosdyn.client.util.authenticate(self.robot)
        self.logger.info("Authentication OK")

        self.robot_id = self.robot.get_id()
        self.robot.start_time_sync()

        # initialize clients
        self.command_client: RobotCommandClient = (self.robot.ensure_client(RobotCommandClient.default_service_name))
        self.state_client: RobotStateClient = (self.robot.ensure_client(RobotStateClient.default_service_name))
        self.lease_client: LeaseClient = self.robot.ensure_client(LeaseClient.default_service_name)
        self.estop_client: EstopClient = self.robot.ensure_client(EstopClient.default_service_name)
        self.graph_nav_client: GraphNavClient = (self.robot.ensure_client(GraphNavClient.default_service_name))
        self.recording_client: RecordingClient = self.robot.ensure_client(RecordingClient.default_service_name)
        self.world_object_client: WorldObjectClient = (self.robot.ensure_client(WorldObjectClient.default_service_name))
        self.docking_client: DockingClient = (self.robot.ensure_client(DockingClient.default_service_name))
        self.map_processing_client: MapClient = self.robot.ensure_client(MapClient.default_service_name)
        self.power_client: PowerClient = self.robot.ensure_client(PowerClient.default_service_name)
        # self.mission_client = self.robot.ensure_client(bosdyn.mission.client.MissionClient.default_service_name)

        # initialize interfaces

        self.recording_interface = RecorderInterface(self.robot, config["download_path"], self.lease_client,
                                                      self.map_processing_client, self.power_client,
                                                     self.state_client, self.command_client, self.world_object_client,
                                                     self.recording_client, self.graph_nav_client)
        # self.graph_nav_interface = GraphNavInterface(self.robot, config["upload_path"], self.command_client,
        #                                              self.state_client, self.graph_nav_client, self.power_client,
        #                                              self.mission_client)

        try:
            self.estop_endpoint = EstopEndpoint(self.estop_client, 'kg-estop', 9.0)
        except Exception as _:
            # Not the estop.
            self.estop_endpoint = None

        self.robot_state_task = AsyncRobotState(self.state_client)
        self._async_tasks = AsyncTasks([self.robot_state_task])
        self._lock = threading.Lock()

        if self.estop_endpoint is not None:
            self.estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.
        
        self.estop_endpoint.allow()

        self.estop_keep_alive = None
        self.exit_check = None
        self.lease_keep_alive = None

        self.logger.info("Spot initialized, startup complete.")

    def __del__(self):
        if self.state == "ready":
            self.shutdown()

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self.robot_state_task.proto

    def acquire(self):
        self.logger.debug("Waiting for time sync...")
        self.robot.time_sync.wait_for_sync()
        self.logger.debug("Time sync OK")

        if self.lease_keep_alive is not None and self.lease_keep_alive.is_alive():
            self.logger.warn("Lease already acquired.")
            return

        if not hasattr(self, "_estop"):
            self.logger.warn("EStop not configured -- please use an external EStop client.")

        self.lease_keep_alive = LeaseKeepAlive(self.lease_client)
        self.logger.info(f"Lease acquired.")

    def release(self):
        if self.lease_keep_alive is None or not self.lease_keep_alive.is_alive():
            return
        self.lease_keep_alive.shutdown()
        self.logger.warn("Lease released.")

    def shutdown(self):
        self.logger.warn("Shutting down...")

        self.power_off()
        self.release()

        if self.estop_keep_alive:
            self.estop_keep_alive.shutdown()

        self.robot.time_sync.stop()
        self.logger.debug("Time sync stopped")

        self.state = "shutdown"
        self.logger.warn("Shutdown complete")

    def power_on(self):
        if self.robot.is_powered_on():
            return
        self.logger.info("Powering on...")
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), "Failed to power on"
        self.powered_on = True
        self.logger.info("Power on complete")

    def power_off(self):
        if not self.robot.is_powered_on():
            return
        self.logger.info("Powering off...")
        self.robot.power_off(timeout_sec=20)
        assert not self.robot.is_powered_on(), "Failed to power off"
        self.powered_on = False
        self.logger.info("Power off complete")

    def release_estop(self):
        if self.estop_client is not None and self.estop_endpoint is not None:
            if self.estop_keep_alive:
                self._try_grpc("stopping estop", self.estop_keep_alive.stop)
                self.estop_keep_alive.shutdown()
                self.estop_keep_alive = None

    def estop(self):
        if self.estop_client is not None and self.estop_endpoint is not None:
            if not self.estop_keep_alive:
                self.estop_keep_alive = EstopKeepAlive(self.estop_endpoint)

    def _toggle_time_sync(self):
        if self.robot.time_sync.stopped:
            self.robot.time_sync.start()
        else:
            self.robot.time_sync.stop()

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError) as err:
            self.logger.error(f"Failed {desc}: {err}")
            return None

    def stand(self):
        self.logger.info("Stand requested")
        blocking_stand(self.command_client, timeout_sec=5)
        self.logger.info("Standing")

    def sit(self):
        self.logger.info("Sit requested")
        blocking_sit(self.command_client, timeout_sec=5)
        self.logger.info("Sitting")

    def _startrobot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self.command_client.robot_command(lease=None, command=command_proto,
                                              end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def _self_right(self):
        self._startrobot_command('self_right', RobotCommandBuilder.selfright_command())

    def _sit(self):
        self._startrobot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._startrobot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def _move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

    def _move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

    def _strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

    def _strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

    def _turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

    def _turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

    def _stop(self):
        self._startrobot_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._startrobot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _return_to_origin(self):
        self._startrobot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
            end_time_secs=time.time() + 20)

    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.logger.error('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc("powering-on", self._request_power_on)
        else:
            self._try_grpc("powering-off", self._safe_power_off)

    def _request_power_on(self):
        bosdyn.client.power.power_on(self.power_client)

    def _safe_power_off(self):
        self._startrobot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    @staticmethod
    def _lease_str(lease_keep_alive):
        alive = 'RUNNING' if lease_keep_alive.is_alive() else 'STOPPED'
        lease_proto = lease_keep_alive.lease_wallet.get_lease_state().lease_original.lease_proto
        lease = f'{lease_proto.resource}:{lease_proto.sequence}'
        return f'Lease {lease} THREAD:{alive}'

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return f'Power: {state_str[6:]}'  # get rid of STATE_ prefix

    def _estop_str(self):
        if not self.estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self.estop_keep_alive else 'STOPPED'
        estop_status = '??'
        state = self.robot_state
        if state:
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break
        return f'Estop {estop_status} (thread: {thread_status})'

    def _time_sync_str(self):
        if not self.robot.time_sync:
            return 'Time sync: (none)'
        if self.robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self.robot.time_sync.thread_exception
            if exception:
                status = f'{status} Exception: {exception}'
        else:
            status = 'RUNNING'
        try:
            skew = self.robot.time_sync.getrobot_clock_skew()
            if skew:
                skew_str = 'offset={}'.format(duration_str(skew))
            else:
                skew_str = "(Skew undetermined)"
        except (TimeSyncError, RpcError) as err:
            skew_str = '({})'.format(err)
        return 'Time sync: {} {}'.format(status, skew_str)

    def _waypoint_str(self):
        state = self.graph_nav_client.get_localization_state()
        try:
            self._waypoint_id = state.localization.waypoint_id
            if self._waypoint_id == '':
                self._waypoint_id = 'NONE'

        except:
            self._waypoint_id = 'ERROR'

        if self.recording_interface.recording and self._waypoint_id != 'NONE' and self._waypoint_id != 'ERROR':
            if self._waypoint_id not in self.recording_interface.desert_flag:
                self.recording_interface.desert_flag[self._waypoint_id] = self.recording_interface.desert_flag
            return f'Current waypoint: {self._waypoint_id} [ RECORDING ]'

        return f'Current waypoint: {self._waypoint_id}'

    def _fiducial_str(self):
        return f'Visible fiducials: {str(self.recording_interface.count_visible_fiducials())}'

    def _desert_str(self):
        if self.recording_interface.desert_mode:
            return '[ FEATURE DESERT MODE ]'
        else:
            return ''

    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        if battery_state.charge_percentage.value:
            bar_len = int(battery_state.charge_percentage.value) // 10
            bat_bar = f'|{"=" * bar_len}{" " * (10 - bar_len)}|'
        else:
            bat_bar = ''
        time_left = ''
        if battery_state.estimated_runtime:
            time_left = f' ({secs_to_hms(battery_state.estimated_runtime.seconds)})'
        return f'Battery: {status}{bat_bar}{time_left}'
