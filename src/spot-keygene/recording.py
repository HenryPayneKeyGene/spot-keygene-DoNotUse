#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import curses
import logging
import os
import threading
import time

import bosdyn.api.robot_state_pb2 as robot_state_proto
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.api.mission import nodes_pb2
from bosdyn.client import RpcError
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotLocalizedToEndError, NotReadyYetError
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, secs_to_hms
from google.protobuf import wrappers_pb2 as wrappers

from .globals import COMMAND_INPUT_RATE, NAV_VELOCITY_LIMITS
from .spot import Spot
from .util import ExitCheck


def write_bytes(filepath, filename, data):
    """Write data to a file."""
    os.makedirs(filepath, exist_ok=True)
    with open(os.path.join(filepath, filename), 'wb+') as f:
        f.write(data)
        f.close()


def write_mission(mission, filename):
    """ Write a mission to disk. """
    open(filename, 'wb').write(mission.SerializeToString())


class CursesHandler(logging.Handler):
    """logging handler which puts messages into the curses interface"""

    def __init__(self, wasd_interface):
        super(CursesHandler, self).__init__()
        self._wasd_interface = wasd_interface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._wasd_interface.add_message(f'{record.levelname:s} {msg:s}')


class RecorderInterface:
    def __init__(self, spot: Spot):
        self.spot = spot
        self.logger = logging.getLogger(__name__)

        # Flag indicating whether mission is currently being recorded
        self._recording = False

        # Flag indicating whether robot is in feature desert mode
        self._desert_mode = False

        # Filepath for the location to put the downloaded graph and snapshots.
        self._download_filepath = self.spot.download_path

        # List of waypoint commands
        self._waypoint_commands = []

        # Dictionary indicating which waypoints are deserts
        self._desert_flag = {}

        # Current waypoint id
        self._waypoint_id = 'NONE'

        # Local copy of the graph.
        self._graph = None
        self._all_graph_wps_in_order = []

        self._command_dictionary = {
            27: self.spot.stop,  # ESC key
            ord('\t'): self._quit_program,
            ord('T'): self.spot.toggle_time_sync,
            ord(' '): self.spot.toggle_estop,
            ord('r'): self.spot.self_right,
            ord('P'): self.spot.toggle_power,
            ord('v'): self.spot.sit,
            ord('f'): self.spot.stand,
            ord('w'): self.spot.move_forward,
            ord('s'): self.spot.move_backward,
            ord('a'): self.spot.strafe_left,
            ord('c'): self._auto_close_loops,
            ord('d'): self.spot.strafe_right,
            ord('q'): self.spot.turn_left,
            ord('e'): self.spot.turn_right,
            ord('m'): self._start_recording,
            ord('l'): self._relocalize,
            ord('z'): self._enter_desert,
            ord('x'): self._exit_desert,
            ord('g'): self._generate_mission,
            ord('j'): self._scan
        }
        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._exit_check = None
        self._lease_keep_alive = None
        self._lock = threading.Lock()

    def start(self, lease_keep_alive):
        """Begin communication with the robot."""
        self._lease_keep_alive = lease_keep_alive

        # Clear existing graph nav map
        self.spot.graph_nav_client.clear_graph()

    def shutdown(self):
        """Shutdown the interface."""
        self.spot.shutdown()

    def _quit_program(self):
        if self._recording:
            self._stop_recording()
        self.shutdown()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def __del__(self):
        self.shutdown()

    def flush_and_estop_buffer(self, screen):
        """Manually flush the curses input buffer but trigger any estop requests (space)"""
        key = ''
        while key != -1:
            key = screen.getch()
            if key == ord(' '):
                self.spot.toggle_estop()

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        with self._lock:
            self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx):
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    def drive(self, screen):
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            curses_handler = CursesHandler(self)
            curses_handler.setLevel(logging.INFO)
            self.logger.addHandler(curses_handler)

            screen.nodelay(True)  # Don't block for user input.
            screen.resize(26, 96)
            screen.refresh()

            # for debug
            curses.echo()

            try:
                while not self._exit_check.kill_now:
                    self.spot.async_tasks.update()
                    self._drive_draw(screen, self._lease_keep_alive)

                    try:
                        cmd = screen.getch()
                        # Do not queue up commands on client
                        self.flush_and_estop_buffer(screen)
                        self._drive_cmd(cmd)
                        time.sleep(COMMAND_INPUT_RATE)
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self.spot.safe_power_off()
                        time.sleep(2.0)
                        self.shutdown()
                        raise

            finally:
                self.logger.removeHandler(curses_handler)

    def _drive_draw(self, screen, lease_keep_alive):
        """Draw the interface screen at each update."""
        screen.clear()  # clear screen
        screen.resize(26, 96)
        screen.addstr(0, 0, f'{self.spot.robot_id.nickname:20s} {self.spot.robot_id.serial_number}')
        screen.addstr(1, 0, self._lease_str(lease_keep_alive))
        screen.addstr(2, 0, self._battery_str())
        screen.addstr(3, 0, self._estop_str())
        screen.addstr(4, 0, self._power_state_str())
        screen.addstr(5, 0, self._time_sync_str())
        screen.addstr(6, 0, self._waypoint_str())
        screen.addstr(7, 0, self._fiducial_str())
        screen.addstr(8, 0, self._desert_str())
        for i in range(3):
            screen.addstr(10 + i, 2, self.message(i))
        screen.addstr(14, 0, 'Commands: [TAB]: quit [j]: Scan                     ')
        screen.addstr(15, 0, '          [T]: Time-sync, [SPACE]: Estop, [P]: Power')
        screen.addstr(16, 0, '          [v]: Sit, [f]: Stand, [r]: Self-right     ')
        screen.addstr(17, 0, '          [wasd]: Directional strafing              ')
        screen.addstr(18, 0, '          [qe]: Turning, [ESC]: Stop                ')
        screen.addstr(19, 0, '          [m]: Start recording mission              ')
        screen.addstr(20, 0, '          [l]: Add fiducial localization to mission ')
        screen.addstr(21, 0, '          [z]: Enter desert mode                    ')
        screen.addstr(22, 0, '          [x]: Exit desert mode                     ')
        screen.addstr(23, 0, '          [g]: Stop recording and generate mission  ')

        screen.refresh()

    def _drive_cmd(self, key):
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f'Unrecognized keyboard command: \'{chr(key)}\'')

    @property
    def robot_state(self):
        """Return the current robot state."""
        return self.spot.robot_state

    @property
    def robot(self):
        return self.spot.robot

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
        if not self.spot.estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self.spot.estop_keep_alive else 'STOPPED'
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
            skew = self.robot.time_sync.get_robot_clock_skew()
            if skew:
                skew_str = f'offset={duration_str(skew)}'
            else:
                skew_str = '(Skew undetermined)'
        except (TimeSyncError, RpcError) as err:
            skew_str = f'({err})'
        return f'Time sync: {status} {skew_str}'

    def _waypoint_str(self):
        state = self.spot.graph_nav_client.get_localization_state()
        try:
            self._waypoint_id = state.localization.waypoint_id
            if self._waypoint_id == '':
                self._waypoint_id = 'NONE'

        except:
            self._waypoint_id = 'ERROR'

        if self._recording and self._waypoint_id != 'NONE' and self._waypoint_id != 'ERROR':
            if self._waypoint_id not in self._desert_flag:
                self._desert_flag[self._waypoint_id] = self._desert_mode
            return f'Current waypoint: {self._waypoint_id} [ RECORDING ]'

        return f'Current waypoint: {self._waypoint_id}'

    def _fiducial_str(self):
        return f'Visible fiducials: {str(self.spot.count_visible_fiducials())}'

    def _desert_str(self):
        if self._desert_mode:
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
            time_left = f'({secs_to_hms(battery_state.estimated_runtime.seconds)})'
        return f'Battery: {status}{bat_bar} {time_left}'

    def _start_recording(self):
        """Start recording a map."""
        if self.spot.count_visible_fiducials() == 0:
            self.add_message('ERROR: Can\'t start recording -- No fiducials in view.')
            return

        if self._waypoint_id is None:
            self.add_message('ERROR: Not localized to waypoint.')
            return
        session_name = os.path.basename(self._download_filepath)
        # Create metadata for the recording session.
        client_metadata = GraphNavRecordingServiceClient.make_client_metadata(
            session_name=session_name, client_username=self.robot._current_user,
            client_id='Mission Recorder Example', client_type='Python SDK')
        environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_metadata))
        # Tell graph nav to start recording map
        status = self.spot.recording_client.start_recording(recording_environment=environment)
        if status != recording_pb2.StartRecordingResponse.STATUS_OK:
            self.add_message('Start recording failed.')
            return

        self.add_message('Started recording map.')
        self._graph = None
        self._all_graph_wps_in_order = []
        state = self.spot.graph_nav_client.get_localization_state()
        if '' == state.localization.waypoint_id:
            self.add_message('No localization after start recording.')
            self.spot.recording_client.stop_recording()
            return
        self._waypoint_commands = []
        # Treat this sort of like RPN / postfix, where operators follow their operands.
        # waypoint_id LOCALIZE means "localize to waypoint_id".
        # INITIALIZE takes no argument.
        # Implicitly, we make routes between pairs of waypoint ids as they occur.
        # `w0 w1 LOCALIZE w2` will give a route w0->w1, whereupon we localize,
        # and then a route w1->w2.
        self._waypoint_commands.append('INITIALIZE')
        # Start with the first waypoint.
        self._waypoint_commands.append(state.localization.waypoint_id)
        self._recording = True

    def _stop_recording(self):
        """Stop or pause recording a map."""
        if not self._recording:
            return True
        self._recording = False

        if self._waypoint_id != self._waypoint_commands[-1]:
            self._waypoint_commands += [self._waypoint_id]

        try:
            finished_recording = False
            status = recording_pb2.StopRecordingResponse.STATUS_UNKNOWN
            while True:
                try:
                    status = self.spot.recording_client.stop_recording()
                except NotReadyYetError:
                    # The recording service always takes some time to complete. stop_recording
                    # must be called multiple times to ensure recording has finished.
                    self.add_message('Stopping...')
                    time.sleep(1.0)
                    continue
                break

            self.add_message('Successfully stopped recording a map.')
            return True

        except NotLocalizedToEndError:
            # This should never happen unless there's an internal error on the robot.
            self.add_message(
                'There was a problem while trying to stop recording. Please try again.')
            return False

    def _relocalize(self):
        """Insert localization node into mission."""
        if not self._recording:
            print('Not recording mission.')
            return False

        self.add_message('Adding fiducial localization to mission.')
        self._waypoint_commands += [self._waypoint_id]
        self._waypoint_commands += ['LOCALIZE']

        # Return to waypoint (in case localization shifted to another waypoint)
        self._waypoint_commands += [self._waypoint_id]
        return True

    def _scan(self):
        """Insert scan node into mission."""
        if not self._recording:
            print('Not recording mission.')
            return False

        self.add_message('Adding scan to mission.')
        self._waypoint_commands += [self._waypoint_id]
        self._waypoint_commands += ['SCAN']

        # Return to waypoint (in case localization shifted to another waypoint)
        self._waypoint_commands += [self._waypoint_id]
        return True

    def _enter_desert(self):
        self._desert_mode = True
        if self._recording:
            if self._waypoint_commands == [] or self._waypoint_commands[-1] != self._waypoint_id:
                self._waypoint_commands += [self._waypoint_id]

    def _exit_desert(self):
        self._desert_mode = False
        if self._recording:
            if self._waypoint_commands == [] or self._waypoint_commands[-1] != self._waypoint_id:
                self._waypoint_commands += [self._waypoint_id]

    def _generate_mission(self):
        """Save graph map and mission file."""

        # Check whether mission has been recorded
        if not self._recording:
            self.add_message('ERROR: No mission recorded.')
            return

        # Check for empty mission
        if len(self._waypoint_commands) == 0:
            self.add_message('ERROR: No waypoints in mission.')
            return

        # Stop recording mission
        if not self._stop_recording():
            self.add_message('ERROR: Error while stopping recording.')
            return

        # Save graph map
        os.mkdir(self._download_filepath)
        if not self._download_full_graph():
            self.add_message('ERROR: Error downloading graph.')
            return

        # Generate mission
        mission = self._make_mission()

        # Save mission file
        os.mkdir(os.path.join(self._download_filepath, 'missions'))
        mission_filepath = os.path.join(self._download_filepath, 'missions', 'autogenerated')
        write_mission(mission, mission_filepath)

        # Quit program
        self._quit_program()

    @staticmethod
    def _make_desert(waypoint):
        waypoint.annotations.scan_match_region.empty.CopyFrom(
            map_pb2.Waypoint.Annotations.LocalizeRegion.Empty())

    def _download_full_graph(self, overwrite_desert_flag=None):
        """Download the graph and snapshots from the robot."""
        graph = self.spot.graph_nav_client.download_graph()
        if graph is None:
            self.add_message('Failed to download the graph.')
            return False

        if overwrite_desert_flag is not None:
            for wp in graph.waypoints:
                if wp.id in overwrite_desert_flag:
                    # Overwrite anything that we passed in.
                    self._desert_flag[wp.id] = overwrite_desert_flag[wp.id]
                elif wp.id not in self._desert_flag:
                    self._desert_flag[wp.id] = False

        # Mark desert waypoints
        for waypoint in graph.waypoints:
            if self._desert_flag[waypoint.id]:
                self._make_desert(waypoint)

        # Write graph map
        self._write_full_graph(graph)
        self.add_message(
            f'Graph downloaded with {len(graph.waypoints)} waypoints and {len(graph.edges)} edges')

        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints)
        self._download_and_write_edge_snapshots(graph.edges)

        # Cache a list of all graph waypoints, ordered by creation time.
        # Because we only record one (non-branching) chain, all possible routes are contained
        # in ranges in this list.
        self._graph = graph
        wp_to_time = []
        for wp in graph.waypoints:
            _time = wp.annotations.creation_time.seconds + wp.annotations.creation_time.nanos / 1e9
            wp_to_time.append((wp.id, _time))
        # Switch inner and outer grouping and grab only the waypoint names after sorting by time.
        self._all_graph_wps_in_order = list(zip(*sorted(wp_to_time, key=lambda x: x[1])))[0]

        return True

    def _write_full_graph(self, graph):
        """Download the graph from robot to the specified, local filepath location."""
        graph_bytes = graph.SerializeToString()
        write_bytes(self._download_filepath, 'graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints):
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            try:
                waypoint_snapshot = self.spot.graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                self.add_message(f'Failed to download waypoint snapshot: {waypoint.snapshot_id}')
                continue
            write_bytes(os.path.join(self._download_filepath, 'waypoint_snapshots'),
                        waypoint.snapshot_id, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            self.add_message(
                f'Downloaded {num_waypoint_snapshots_downloaded} of the total {len(waypoints)} waypoint snapshots.'
            )

    def _download_and_write_edge_snapshots(self, edges):
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self.spot.graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                self.add_message(f'Failed to download edge snapshot: {edge.snapshot_id}')
                continue
            write_bytes(os.path.join(self._download_filepath, 'edge_snapshots'), edge.snapshot_id,
                        edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            self.add_message(
                f'Downloaded {num_edge_snapshots_downloaded} of the total {num_to_download} edge snapshots.'
            )

    def _auto_close_loops(self):
        """Automatically find and close all loops in the graph."""
        close_fiducial_loops = True
        close_odometry_loops = True
        response = self.spot.map_processing_client.process_topology(
            params=map_processing_pb2.ProcessTopologyRequest.Params(
                do_fiducial_loop_closure=wrappers.BoolValue(value=close_fiducial_loops),
                do_odometry_loop_closure=wrappers.BoolValue(value=close_odometry_loops)),
            modify_map_on_server=True)
        self.add_message(f'Created {len(response.new_subgraph.edges)} new edge(s).')

    def _make_mission(self):
        """ Create a mission that visits each waypoint on stored path."""

        self.add_message(f'Mission: {self._waypoint_commands}')

        # Create a Sequence that visits all the waypoints.
        sequence = nodes_pb2.Sequence()

        prev_waypoint_command = None
        first_waypoint = True
        for waypoint_command in self._waypoint_commands:
            if waypoint_command == 'LOCALIZE':
                if prev_waypoint_command is None:
                    raise RuntimeError('No prev waypoint; LOCALIZE')
                sequence.children.add().CopyFrom(self._make_localize_node(prev_waypoint_command))
            elif waypoint_command == 'SCAN':
                if prev_waypoint_command is None:
                    raise RuntimeError('No prev waypoint; SCAN')
                sequence.children.add().CopyFrom(self._make_scan_node(prev_waypoint_command))
            elif waypoint_command == 'INITIALIZE':
                # Initialize to any fiducial.
                sequence.children.add().CopyFrom(self._make_initialize_node())
            else:
                if first_waypoint:
                    # Go to the beginning of the mission using any path. May be a no-op.
                    sequence.children.add().CopyFrom(self._make_goto_node(waypoint_command))
                else:
                    if prev_waypoint_command is None:
                        raise RuntimeError(f'No prev waypoint; route to {waypoint_command}')
                    sequence.children.add().CopyFrom(
                        self._make_go_route_node(prev_waypoint_command, waypoint_command))
                prev_waypoint_command = waypoint_command
                first_waypoint = False

        # Return a Node with the Sequence.
        ret = nodes_pb2.Node()
        ret.name = f'Visit {len(self._waypoint_commands):d} goals'
        ret.impl.Pack(sequence)
        return ret

    @staticmethod
    def _make_goto_node(waypoint_id):
        """ Create a leaf node that will go to the waypoint. """
        ret = nodes_pb2.Node()
        ret.name = f'goto {waypoint_id}'
        vel_limit = NAV_VELOCITY_LIMITS

        # We don't actually know which path we will plan. It could have many feature deserts.
        # So, let's just allow feature deserts.
        tolerance = graph_nav_pb2.TravelParams.TOLERANCE_IGNORE_POOR_FEATURE_QUALITY
        travel_params = graph_nav_pb2.TravelParams(velocity_limit=vel_limit,
                                                   feature_quality_tolerance=tolerance)

        impl = nodes_pb2.BosdynNavigateTo(travel_params=travel_params)
        impl.destination_waypoint_id = waypoint_id
        ret.impl.Pack(impl)
        return ret

    def _make_go_route_node(self, prev_waypoint_id, waypoint_id):
        """ Create a leaf node that will go to the waypoint along a route. """
        ret = nodes_pb2.Node()
        ret.name = f'go route {prev_waypoint_id} -> {waypoint_id}'
        vel_limit = NAV_VELOCITY_LIMITS

        if prev_waypoint_id not in self._all_graph_wps_in_order:
            raise RuntimeError(f'{prev_waypoint_id} (FROM) not in {self._all_graph_wps_in_order}')
        if waypoint_id not in self._all_graph_wps_in_order:
            raise RuntimeError(f'{waypoint_id} (TO) not in {self._all_graph_wps_in_order}')
        prev_wp_idx = self._all_graph_wps_in_order.index(prev_waypoint_id)
        wp_idx = self._all_graph_wps_in_order.index(waypoint_id)

        impl = nodes_pb2.BosdynNavigateRoute()
        backwards = False
        if wp_idx >= prev_wp_idx:
            impl.route.waypoint_id[:] = self._all_graph_wps_in_order[prev_wp_idx:wp_idx + 1]
        else:
            backwards = True
            # Switch the indices and reverse the output order.
            impl.route.waypoint_id[:] = reversed(
                self._all_graph_wps_in_order[wp_idx:prev_wp_idx + 1])
        edge_ids = [e.id for e in self._graph.edges]
        for i in range(len(impl.route.waypoint_id) - 1):
            eid = map_pb2.Edge.Id()
            # Because we only record one (non-branching) chain, and we only create routes from
            # older to newer waypoints, we can just follow the time-sorted list of all waypoints.
            if backwards:
                from_i = i + 1
                to_i = i
            else:
                from_i = i
                to_i = i + 1
            eid.from_waypoint = impl.route.waypoint_id[from_i]
            eid.to_waypoint = impl.route.waypoint_id[to_i]
            impl.route.edge_id.extend([eid])
            if eid not in edge_ids:
                raise RuntimeError(f'Was map recorded in linear chain? {eid} not in graph')

        if any(self._desert_flag[wp_id] for wp_id in impl.route.waypoint_id):
            tolerance = graph_nav_pb2.TravelParams.TOLERANCE_IGNORE_POOR_FEATURE_QUALITY
        else:
            tolerance = graph_nav_pb2.TravelParams.TOLERANCE_DEFAULT

        travel_params = graph_nav_pb2.TravelParams(velocity_limit=vel_limit,
                                                   feature_quality_tolerance=tolerance)
        impl.travel_params.CopyFrom(travel_params)
        ret.impl.Pack(impl)
        return ret

    @staticmethod
    def _make_localize_node(waypoint_id):
        """Make localization node."""
        loc = nodes_pb2.Node()
        loc.name = 'localize robot'

        impl = nodes_pb2.BosdynGraphNavLocalize()
        impl.localization_request.fiducial_init = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST_AT_TARGET
        impl.localization_request.initial_guess.waypoint_id = waypoint_id

        loc.impl.Pack(impl)
        return loc

    @staticmethod
    def _make_scan_node(waypoint_id):
        """Make scan node."""
        # this is actually a sleep node. scan is implemented in the mission.
        loc = nodes_pb2.Node()
        loc.name = 'scan'

        impl = nodes_pb2.Sleep(seconds=10)

        loc.impl.Pack(impl)
        return loc

    @staticmethod
    def _make_initialize_node():
        """Make initialization node."""
        loc = nodes_pb2.Node()
        loc.name = 'initialize robot'

        impl = nodes_pb2.BosdynGraphNavLocalize()
        impl.localization_request.fiducial_init = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST

        loc.impl.Pack(impl)
        return loc
