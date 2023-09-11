#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

from __future__ import annotations

import logging
import os
import time

from bosdyn.api import world_object_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.api.mission import nodes_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.power import PowerClient
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotLocalizedToEndError, NotReadyYetError
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient
from google.protobuf import wrappers_pb2 as wrappers

from .globals import NAV_VELOCITY_LIMITS


def write_mission(mission, filename):
    """ Write a mission to disk. """
    open(filename, 'wb').write(mission.SerializeToString())


def write_bytes(filepath, filename, data):
    """Write data to a file."""
    os.makedirs(filepath, exist_ok=True)
    with open(os.path.join(filepath, filename), 'wb+') as f:
        f.write(data)
        f.close()


class RecorderInterface(object):
    """A curses interface for recording robot missions."""

    def __init__(self, robot, download_filepath, lease_client: LeaseClient, estop_client: EstopClient,
                 map_client: MapProcessingServiceClient, power_client: PowerClient, state_client: RobotStateClient,
                 cmd_client: RobotCommandClient, world_object_client: WorldObjectClient,
                 rec_client: GraphNavRecordingServiceClient, graph_nav_client: GraphNavClient):
        self.logger = logging.getLogger(__name__)
        self._robot = robot

        # Flag indicating whether mission is currently being recorded
        self._recording = False

        # Flag indicating whether robot is in feature desert mode
        self._desert_mode = False

        # Filepath for the location to put the downloaded graph and snapshots.
        self._download_filepath = download_filepath

        # List of waypoint commands
        self._waypoint_commands = []

        # Dictionary indicating which waypoints are deserts
        self._desert_flag = {}

        # Current waypoint id
        self._waypoint_id = 'NONE'

        # Create clients -- do not use the for communication yet.
        self._lease_client = lease_client

        self._map_processing_client = map_client
        self._power_client = power_client
        self._robot_state_client = state_client
        self._robot_command_client = cmd_client
        self._world_object_client = world_object_client

        # Set up the recording service client.
        self._recording_client = rec_client

        # Set up the graph nav service client.
        self._graph_nav_client = graph_nav_client

        # Local copy of the graph.
        self._graph = None
        self._all_graph_wps_in_order = []

        self._command_dictionary = {
            27: self._stop,  # ESC key
            ord('\t'): self._quit_program,
            ord('T'): self._toggle_time_sync,
            ord(' '): self._toggle_estop,
            ord('r'): self._self_right,
            ord('P'): self._toggle_power,
            ord('v'): self._sit,
            ord('f'): self._stand,
            ord('w'): self._move_forward,
            ord('s'): self._move_backward,
            ord('a'): self._strafe_left,
            ord('c'): self._auto_close_loops,
            ord('d'): self._strafe_right,
            ord('q'): self._turn_left,
            ord('e'): self._turn_right,
            ord('m'): self.start_recording,
            ord('l'): self._relocalize,
            ord('z'): self._enter_desert,
            ord('x'): self._exit_desert,
            ord('g'): self.generate_mission
        }
        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None

    def start(self, lease_keep_alive):
        """Begin communication with the robot."""
        self._lease_keep_alive = lease_keep_alive
        self._robot_id = self._robot.get_id()

        # Clear existing graph nav map
        self._graph_nav_client.clear_graph()

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        self.logger.info("Shutting down WasdInterface.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()

    def __del__(self):
        self.shutdown()

    # def flush_and_estop_buffer(self, stdscr):
    #     """Manually flush the curses input buffer but trigger any estop requests (space)"""
    #     key = ''
    #     while key != -1:
    #         key = stdscr.getch()
    #         if key == ord(' '):
    #             self._toggle_estop()
    #
    # def add_message(self, msg_text):
    #     """Display the given message string to the user in the curses interface."""
    #     with self._lock:
    #         self._locked_messages = [msg_text] + self._locked_messages[:-1]
    #
    # def message(self, idx):
    #     """Grab one of the 3 last messages added."""
    #     with self._lock:
    #         return self._locked_messages[idx]
    #
    #
    #
    # def drive(self, stdscr):
    #     """User interface to control the robot via the passed-in curses screen interface object."""
    #     with ExitCheck() as self._exit_check:
    #         curses_handler = CursesHandler(self)
    #         curses_handler.setLevel(logging.INFO)
    #         self.logger.addHandler(curses_handler)
    #
    #         stdscr.nodelay(True)  # Don't block for user input.
    #         stdscr.resize(26, 96)
    #         stdscr.refresh()
    #
    #         # for debug
    #         curses.echo()
    #
    #         try:
    #             while not self._exit_check.kill_now:
    #                 self._async_tasks.update()
    #                 self._drive_draw(stdscr, self._lease_keep_alive)
    #
    #                 try:
    #                     cmd = stdscr.getch()
    #                     # Do not queue up commands on client
    #                     self.flush_and_estop_buffer(stdscr)
    #                     self._drive_cmd(cmd)
    #                     time.sleep(COMMAND_INPUT_RATE)
    #                 except Exception:
    #                     # On robot command fault, sit down safely before killing the program.
    #                     self._safe_power_off()
    #                     time.sleep(2.0)
    #                     self.shutdown()
    #                     raise
    #
    #         finally:
    #             self.logger.removeHandler(curses_handler)

    # def _drive_draw(self, stdscr, lease_keep_alive):
    #     """Draw the interface screen at each update."""
    #     stdscr.clear()  # clear screen
    #     stdscr.resize(26, 96)
    #     stdscr.addstr(0, 0, '{:20s} {}'.format(self._robot_id.nickname,
    #                                            self._robot_id.serial_number))
    #     stdscr.addstr(1, 0, self._lease_str(lease_keep_alive))
    #     stdscr.addstr(2, 0, self._battery_str())
    #     stdscr.addstr(3, 0, self._estop_str())
    #     stdscr.addstr(4, 0, self._power_state_str())
    #     stdscr.addstr(5, 0, self._time_sync_str())
    #     stdscr.addstr(6, 0, self._waypoint_str())
    #     stdscr.addstr(7, 0, self._fiducial_str())
    #     stdscr.addstr(8, 0, self._desert_str())
    #     for i in range(3):
    #         stdscr.addstr(10 + i, 2, self.message(i))
    #     stdscr.addstr(14, 0, "Commands: [TAB]: quit                               ")
    #     stdscr.addstr(15, 0, "          [T]: Time-sync, [SPACE]: Estop, [P]: Power")
    #     stdscr.addstr(16, 0, "          [v]: Sit, [f]: Stand, [r]: Self-right     ")
    #     stdscr.addstr(17, 0, "          [wasd]: Directional strafing              ")
    #     stdscr.addstr(18, 0, "          [qe]: Turning, [ESC]: Stop                ")
    #     stdscr.addstr(19, 0, "          [m]: Start recording mission              ")
    #     stdscr.addstr(20, 0, "          [l]: Add fiducial localization to mission ")
    #     stdscr.addstr(21, 0, "          [z]: Enter desert mode                    ")
    #     stdscr.addstr(22, 0, "          [x]: Exit desert mode                     ")
    #     stdscr.addstr(23, 0, "          [g]: Stop recording and generate mission  ")
    #
    #     stdscr.refresh()

    # def _drive_cmd(self, key):
    #     """Run user commands at each update."""
    #     try:
    #         cmd_function = self._command_dictionary[key]
    #         cmd_function()
    #
    #     except KeyError:
    #         if key and key != -1 and key < 256:
    #             self.add_message("Unrecognized keyboard command: '{}'".format(chr(key)))

    def _quit_program(self):
        if self._recording:
            self._stop_recording()
        self._robot.power_off()
        self.shutdown()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def get_fiducial_objects(self):
        """Get all fiducials that Spot detects with its perception system."""
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            # Return the first detected fiducials.
            return fiducial_objects
        # Return none if no fiducials are found.
        return None

    def fiducial_is_visible(self, fiducial_id):
        """Check if a specific fiducial is visible to Spot."""
        fiducial_objects = self.get_fiducial_objects()
        if fiducial_objects is not None:
            for fiducial_object in fiducial_objects:
                if fiducial_object.id == fiducial_id:
                    return True
        return False

    def wait_for_fiducial(self, fiducial_id=None):
        """Wait for a specific fiducial to be visible to Spot."""
        if fiducial_id is None:
            while not self.fiducial_visible():
                time.sleep(0.1)
        else:
            while not self.fiducial_is_visible(fiducial_id):
                time.sleep(0.1)

    def fiducial_visible(self):
        """Return True if robot can see fiducial."""
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(
            object_type=request_fiducials).world_objects
        self.logger.info("Fiducial objects: " + str(fiducial_objects))

        if len(fiducial_objects) > 0:
            return True

        return False

    def count_visible_fiducials(self):
        """Return number of fiducials visible to robot."""
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(
            object_type=request_fiducials).world_objects
        return len(fiducial_objects)

    def start_recording(self):
        """Start recording a map."""
        if self.count_visible_fiducials() == 0:
            self.logger.error("ERROR: Can't start recording -- No fiducials in view.")
            return

        if self._waypoint_id is None:
            self.logger.error("ERROR: Not localized to waypoint.")
            return
        session_name = os.path.basename(self._download_filepath)
        # Create metadata for the recording session.
        client_metadata = GraphNavRecordingServiceClient.make_client_metadata(
            session_name=session_name, client_username=self._robot._current_user,
            client_id='Mission Recorder Example', client_type='Python SDK')
        environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_metadata))
        # Tell graph nav to start recording map
        status = self._recording_client.start_recording(recording_environment=environment)
        if status != recording_pb2.StartRecordingResponse.STATUS_OK:
            self.logger.error("Start recording failed.")
            return

        self.logger.info("Started recording map.")
        self._graph = None
        self._all_graph_wps_in_order = []
        state = self._graph_nav_client.get_localization_state()
        if '' == state.localization.waypoint_id:
            self.logger.warning("No localization after start recording.")
            self._recording_client.stop_recording()
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
                    status = self._recording_client.stop_recording()
                except NotReadyYetError:
                    # The recording service always takes some time to complete. stop_recording
                    # must be called multiple times to ensure recording has finished.
                    self.logger.warning("Stopping...")
                    time.sleep(1.0)
                    continue
                break

            self.logger.info("Successfully stopped recording a map.")
            return True

        except NotLocalizedToEndError:
            # This should never happen unless there's an internal error on the robot.
            self.logger.error(
                "There was a problem while trying to stop recording. Please try again.")
            return False

    def _relocalize(self):
        """Insert localization node into mission."""
        if not self._recording:
            print('Not recording mission.')
            return False

        self.logger.info("Adding fiducial localization to mission.")
        self._waypoint_commands += [self._waypoint_id]
        self._waypoint_commands += ['LOCALIZE']

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

    def generate_mission(self):
        """Save graph map and mission file."""

        # Check whether mission has been recorded
        if not self._recording:
            self.logger.error("ERROR: No mission recorded.")
            return

        # Check for empty mission
        if len(self._waypoint_commands) == 0:
            self.logger.error("ERROR: No waypoints in mission.")
            return

        # Stop recording mission
        if not self._stop_recording():
            self.logger.error("ERROR: Error while stopping recording.")
            return

        # Save graph map
        os.mkdir(self._download_filepath)
        if not self._download_full_graph():
            self.logger.error("ERROR: Error downloading graph.")
            return

        # Generate mission
        mission = self._make_mission()

        # Save mission file
        os.mkdir(os.path.join(self._download_filepath, "missions"))
        mission_filepath = os.path.join(self._download_filepath, "missions", "autogenerated")
        write_mission(mission, mission_filepath)

        # Quit program
        self._quit_program()

    @staticmethod
    def _make_desert(waypoint):
        waypoint.annotations.scan_match_region.empty.CopyFrom(
            map_pb2.Waypoint.Annotations.LocalizeRegion.Empty())

    def _download_full_graph(self, overwrite_desert_flag=None):
        """Download the graph and snapshots from the robot."""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self.logger.error("Failed to download the graph.")
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
        self.logger.info(f"Graph downloaded with {len(graph.waypoints)} waypoints and {len(graph.edges)} edges")

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
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                self.logger.error("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                continue
            write_bytes(os.path.join(self._download_filepath, 'waypoint_snapshots'),
                        waypoint.snapshot_id, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            self.logger.info(
                f"Downloaded {num_waypoint_snapshots_downloaded} of the total {len(waypoints)} waypoint snapshots.")

    def _download_and_write_edge_snapshots(self, edges):
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                self.logger.error("Failed to download edge snapshot: " + edge.snapshot_id)
                continue
            write_bytes(os.path.join(self._download_filepath, 'edge_snapshots'), edge.snapshot_id,
                        edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            self.logger.info(
                f"Downloaded {num_edge_snapshots_downloaded} of the total {num_to_download} edge snapshots.")

    def _auto_close_loops(self):
        """Automatically find and close all loops in the graph."""
        close_fiducial_loops = True
        close_odometry_loops = True
        response = self._map_processing_client.process_topology(
            params=map_processing_pb2.ProcessTopologyRequest.Params(
                do_fiducial_loop_closure=wrappers.BoolValue(value=close_fiducial_loops),
                do_odometry_loop_closure=wrappers.BoolValue(value=close_odometry_loops)),
            modify_map_on_server=True)
        self.logger.info(f"Created {len(response.new_subgraph.edges)} new edge(s).")

    def _make_mission(self):
        """ Create a mission that visits each waypoint on stored path."""

        self.logger.info("Mission: " + str(self._waypoint_commands))

        # Create a Sequence that visits all the waypoints.
        sequence = nodes_pb2.Sequence()

        prev_waypoint_command = None
        first_waypoint = True
        for waypoint_command in self._waypoint_commands:
            if waypoint_command == 'LOCALIZE':
                if prev_waypoint_command is None:
                    raise RuntimeError(f'No prev waypoint; LOCALIZE')
                sequence.children.add().CopyFrom(self._make_localize_node(prev_waypoint_command))
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
        ret.name = "Visit %d goals" % len(self._waypoint_commands)
        ret.impl.Pack(sequence)
        return ret

    @staticmethod
    def _make_goto_node(waypoint_id):
        """ Create a leaf node that will go to the waypoint. """
        ret = nodes_pb2.Node()
        ret.name = "goto %s" % waypoint_id
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
        ret.name = "go route %s -> %s" % (prev_waypoint_id, waypoint_id)
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
        loc.name = "localize robot"

        impl = nodes_pb2.BosdynGraphNavLocalize()
        impl.localization_request.fiducial_init = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST_AT_TARGET
        impl.localization_request.initial_guess.waypoint_id = waypoint_id

        loc.impl.Pack(impl)
        return loc

    @staticmethod
    def _make_initialize_node():
        """Make initialization node."""
        loc = nodes_pb2.Node()
        loc.name = "initialize robot"

        impl = nodes_pb2.BosdynGraphNavLocalize()
        impl.localization_request.fiducial_init = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST

        loc.impl.Pack(impl)
        return loc

    @property
    def recording(self):
        return self._recording

    @property
    def desert_flag(self):
        return self._desert_flag

    @property
    def desert_mode(self):
        return self._desert_mode
