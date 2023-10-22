# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""WASD driving of robot."""
import curses
import logging
import os
import signal
import threading
import time
from argparse import Namespace

import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
# import bosdyn.api.robot_command_pb2 as robot_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from PIL import Image, ImageEnhance
from bosdyn import geometry
from bosdyn.api.autowalk import walks_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, recording_pb2
from bosdyn.client import ResponseError, Robot, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.docking import DockingClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.graph_nav import CannotModifyMapDuringRecordingError, GraphNavClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Error as LeaseBaseError, LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.util import duration_str, secs_to_hms

LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

IMAGE_SOURCES = [
    "frontright_fisheye_image",
    "frontleft_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
    "left_fisheye_image",
]
ASYNC_CAPTURE_RATE = 40  # milliseconds, 25 Hz
LINEAR_VELOCITY_DEFAULT = 0.6  # m/s
ANGULAR_VELOCITY_DEFAULT = 0.8  # rad/sec
COMMAND_DURATION_DEFAULT = 0.6  # seconds
SLIDER_MIDDLE_VALUE = 50
TRAVEL_MAX_DIST = 0.2  # meters
BLOCKED_PATH_WAIT_TIME = 5  # seconds
RETRY_COUNT_DEFAULT = 1
PROMPT_DURATION_DEFAULT = 60  # seconds
PROMPT_DURATION_DOCK = 600  # seconds
BATTERY_THRESHOLDS = (60.0, 10.0)  # battery percentages
POSE_INDEX, ROBOT_CAMERA_INDEX, DOCK_INDEX, SCAN_INDEX = range(4)
INITIAL_PANEL, RECORDED_PANEL, ACTION_PANEL, FINAL_PANEL = range(4)


def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error('Failed %s: %s', desc, err)


def _image_to_ascii(image, new_width):
    """Convert an rgb image to an ASCII 'image' that can be displayed in a terminal."""

    ASCII_CHARS = '@#S%?*+;:,.'

    enhancer = ImageEnhance.Contrast(image)
    image = enhancer.enhance(0.8)

    # Scaling image before rotation by 90 deg.
    scaled_rot_height = new_width
    original_rot_width, original_rot_height = image.size
    scaled_rot_width = (original_rot_width * scaled_rot_height) // original_rot_height
    # Scaling rotated width (height, after rotation) by half because ASCII chars
    #  in terminal seem about 2x as tall as wide.
    image = image.resize((scaled_rot_width // 2, scaled_rot_height))

    # Rotate image 90 degrees, then convert to grayscale.
    image = image.transpose(Image.ROTATE_270)
    image = image.convert('L')

    def _pixel_char(pixel_val):
        return ASCII_CHARS[pixel_val * len(ASCII_CHARS) // 256]

    img = []
    row = [' '] * new_width
    last_col = new_width - 1
    for idx, pixel_char in enumerate(_pixel_char(val) for val in image.getdata()):
        idx_row = idx % new_width
        row[idx_row] = pixel_char
        if idx_row == last_col:
            img.append(''.join(row))
    return img


class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""

    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    def request_exit(self):
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self):
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now


class CursesHandler(logging.Handler):
    """logging handler which puts messages into the curses interface"""

    def __init__(self, wasd_interface):
        super(CursesHandler, self).__init__()
        self._wasd_interface = wasd_interface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._wasd_interface.add_message(f'{record.levelname:s} {msg:s}')


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


class AsyncImageCapture(AsyncGRPCTask):
    """Grab camera images from the robot."""

    def __init__(self, robot):
        super(AsyncImageCapture, self).__init__()
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._ascii_image = None
        self._video_mode = False
        self._should_take_image = False

    @property
    def ascii_image(self):
        """Return the latest captured image as ascii."""
        return self._ascii_image

    def toggle_video_mode(self):
        """Toggle whether doing continuous image capture."""
        self._video_mode = not self._video_mode

    def take_image(self):
        """Request a one-shot image."""
        self._should_take_image = True

    def _start_query(self):
        self._should_take_image = False
        source_name = 'frontright_fisheye_image'
        return self._image_client.get_image_from_sources_async([source_name])

    def _should_query(self, now_sec):  # pylint: disable=unused-argument
        return self._video_mode or self._should_take_image

    def _handle_result(self, result):
        import io
        image = Image.open(io.BytesIO(result[0].shot.image.data))
        self._ascii_image = _image_to_ascii(image, new_width=70)

    def _handle_error(self, exception):
        LOGGER.exception('Failure getting image: %s', exception)


class WasdInterface(object):
    """A curses interface for driving the robot."""

    def __init__(self, robot):

        self.robot: Robot = robot

        self.strings = Namespace(rec="Start recording")

        self._robot_id = self.robot.get_id()
        self._lease_keepalive = None
        self.walk = self._init_walk()
        self.directory = os.getcwd()

        # Initialize clients
        self._lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self._power_client = self.robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self._graph_nav_client = self.robot.ensure_client(GraphNavClient.default_service_name)
        self._recording_client: GraphNavRecordingServiceClient = (
            self.robot.ensure_client(
                GraphNavRecordingServiceClient.default_service_name
            )
        )

        # Clear graph to ensure only the data recorded using this example gets packaged into map
        try:
            self._graph_nav_client.clear_graph()
        except CannotModifyMapDuringRecordingError:
            self._toggle_record()

        self._world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)
        self._docking_client = self.robot.ensure_client(
            DockingClient.default_service_name
        )
        try:
            self._estop_client = self.robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None

        # Initialize async tasks
        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._async_tasks = AsyncTasks([self._robot_state_task])
        self._async_tasks.update()
        self._image_task = AsyncImageCapture(robot)
        self._async_tasks = AsyncTasks([self._robot_state_task, self._image_task])
        self._lock = threading.Lock()

        # Timer for grabbing robot states

        # Default starting speed values for the robot
        self.linear_velocity = LINEAR_VELOCITY_DEFAULT
        self.angular_velocity = ANGULAR_VELOCITY_DEFAULT
        self.command_duration = COMMAND_DURATION_DEFAULT

        # Experimentally determined default starting values for pitch, roll, yaw, and height
        self.euler_angles = geometry.EulerZXY()
        self.robot_height = 0

        self.resumed_recording = False
        self.elements = []
        self.fiducial_objects = []
        self.dock = None
        self.dock_and_end_recording = False

        self._command_dictionary = {
            27: self._stop,  # ESC key
            ord('\t'): self._quit_program,
            ord('T'): self._toggle_time_sync,
            ord(' '): self._toggle_estop,
            ord('r'): self._self_right,
            ord('P'): self._toggle_power,
            ord('p'): self._toggle_power,
            ord('v'): self._sit,
            ord('b'): self._battery_change_pose,
            ord('f'): self._stand,
            ord('w'): self._move_forward,
            ord('s'): self._move_backward,
            ord('a'): self._strafe_left,
            ord('d'): self._strafe_right,
            ord('q'): self._turn_left,
            ord('e'): self._turn_right,
            ord('I'): self._image_task.take_image,
            ord('O'): self._image_task.toggle_video_mode,
            ord('u'): self._unstow,
            ord('j'): self._stow,
            ord('l'): self._toggle_lease,

            # recording actions
            ord('1'): self._toggle_record,
            ord('2'): self._save_autowalk,
        }
        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None
        self._lease_keepalive: LeaseKeepAlive = None

    def __del__(self):
        if self._recording_client is not None:
            recording_status = self._recording_client.get_record_status()

            if recording_status.is_recording:
                self._toggle_record()
        if self._lease_keepalive.is_alive():
            self._lease_keepalive.shutdown()

    @staticmethod
    def _init_walk():
        """Initialize walk object"""

        walk = walks_pb2.Walk()
        walk.global_parameters.self_right_attempts = RETRY_COUNT_DEFAULT
        walk.playback_mode.once.SetInParent()
        return walk

    def _save_autowalk(self):
        """Save autowalk in directory"""

        walk_name = "scan"  # self.walk_name
        self.walk.mission_name = walk_name
        self.add_message(f"Saving to {walk_name}.walk...")

        if not self.dock_and_end_recording:
            # Creates end waypoint and action if the last action is not docking
            self._toggle_record()
            waypoint_response = self._recording_client.create_waypoint()
            end_element = self._create_element('End Recording', waypoint_response.created_waypoint,
                                               is_action=False)
            self.elements.append(end_element)
            self._toggle_record()

        # Creates walk directory and saves autowalk in format
        walk_directory = os.path.join(self.directory, f'{walk_name}.walk')
        directory_copy_number = 0
        while os.path.isdir(walk_directory):
            # Appends (number) to folder name if folder with existing name exists
            directory_copy_number += 1
            walk_directory = os.path.join(self.directory, f'{walk_name}({directory_copy_number}).walk')
        self._graph_nav_client.write_graph_and_snapshots(walk_directory)
        self.walk.elements.extend(self.elements)
        mission_directory = os.path.join(walk_directory, 'missions')
        os.mkdir(mission_directory)
        with open(os.path.join(mission_directory, 'autogenerated.walk'), 'wb') as autowalk_file:
            autowalk_file.write(self.walk.SerializeToString())

        self._reset_walk()
        self.add_message("Saved recording.")

    def _reset_walk(self):
        """Resets walk parameters and GUI"""
        self.walk = self._init_walk()
        self.strings.rec = 'Start Recording'
        self.resumed_recording = False
        self.dock_and_end_recording = False
        self._graph_nav_client.clear_graph()

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                               return_at_exit=True)

        self._robot_id = self.robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info('Shutting down WasdInterface.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    def flush_and_estop_buffer(self, stdscr):
        """Manually flush the curses input buffer but trigger any estop requests (space)"""
        key = ''
        while key != -1:
            key = stdscr.getch()
            if key == ord(' '):
                self._toggle_estop()

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        with self._lock:
            self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx):
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def drive(self, stdscr):
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            curses_handler = CursesHandler(self)
            curses_handler.setLevel(logging.INFO)
            LOGGER.addHandler(curses_handler)

            stdscr.nodelay(True)  # Don't block for user input.
            stdscr.resize(26, 140)
            stdscr.refresh()

            # for debug
            curses.echo()

            try:
                while not self._exit_check.kill_now:
                    self._async_tasks.update()
                    self._drive_draw(stdscr, self._lease_keepalive)

                    try:
                        cmd = stdscr.getch()
                        # Do not queue up commands on client
                        self.flush_and_estop_buffer(stdscr)
                        self._drive_cmd(cmd)
                        time.sleep(COMMAND_INPUT_RATE)
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self._safe_power_off()
                        time.sleep(2.0)
                        raise

            finally:
                LOGGER.removeHandler(curses_handler)

    def _drive_draw(self, stdscr, lease_keep_alive):
        """Draw the interface screen at each update."""
        stdscr.clear()  # clear screen
        stdscr.resize(26, 140)
        stdscr.addstr(0, 0, f'{self._robot_id.nickname:20s} {self._robot_id.serial_number}')
        stdscr.addstr(1, 0, self._lease_str(lease_keep_alive))
        stdscr.addstr(2, 0, self._battery_str())
        stdscr.addstr(3, 0, self._estop_str())
        stdscr.addstr(4, 0, self._power_state_str())
        stdscr.addstr(5, 0, self._time_sync_str())
        for i in range(3):
            stdscr.addstr(7 + i, 2, self.message(i))
        stdscr.addstr(10, 0, 'Commands: [TAB]: quit                               ')
        stdscr.addstr(11, 0, '          [T]: Time-sync, [SPACE]: Estop, [P]: Power')
        stdscr.addstr(12, 0, '          [I]: Take image, [O]: Video mode          ')
        stdscr.addstr(13, 0, '          [f]: Stand, [r]: Self-right               ')
        stdscr.addstr(14, 0, '          [v]: Sit, [b]: Battery-change             ')
        stdscr.addstr(15, 0, '          [wasd]: Directional strafing              ')
        stdscr.addstr(16, 0, '          [qe]: Turning, [ESC]: Stop                ')
        stdscr.addstr(17, 0, '          [l]: Return/Acquire lease                 ')
        stdscr.addstr(18, 0, '')
        stdscr.addstr(19, 0, f"          [1]: {self.strings.rec} [2]: Save")
        stdscr.addstr(20, 0, '')

        # print as many lines of the image as will fit on the curses screen
        if self._image_task.ascii_image is not None:
            max_y, _max_x = stdscr.getmaxyx()
            for y_i, img_line in enumerate(self._image_task.ascii_image):
                if y_i + 19 >= max_y:
                    break

                stdscr.addstr(y_i + 19, 0, img_line)

        stdscr.refresh()

    def _drive_cmd(self, key):
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f'Unrecognized keyboard command: \'{chr(key)}\'')

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message(f'Failed {desc}: {err}')
            return None

    def _try_grpc_async(self, desc, thunk):

        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                self.add_message(f'Failed {desc}: {err}')
                return None

        future = thunk()
        future.add_done_callback(on_future_done)

    def _quit_program(self):
        self.add_message("Exiting...")
        self._sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def _toggle_time_sync(self):
        if self.robot.time_sync.stopped:
            self.robot.start_time_sync()
        else:
            self.robot.time_sync.stop()

    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    @staticmethod
    def _create_element(name, waypoint, is_action=True) -> walks_pb2.Element:
        """Creates default autowalk Element object, populating everything except Action and ActionWrapper"""
        element: walks_pb2.Element = walks_pb2.Element()
        element.name = name
        element.target.navigate_to.destination_waypoint_id = waypoint.id
        element.target.navigate_to.travel_params.max_distance = TRAVEL_MAX_DIST
        element.target.navigate_to.travel_params.feature_quality_tolerance = (
            graph_nav_pb2.TravelParams.FeatureQualityTolerance.TOLERANCE_IGNORE_POOR_FEATURE_QUALITY)
        element.target.navigate_to.travel_params.blocked_path_wait_time.seconds = BLOCKED_PATH_WAIT_TIME
        element.target_failure_behavior.retry_count = RETRY_COUNT_DEFAULT
        element.target_failure_behavior.prompt_duration.seconds = PROMPT_DURATION_DEFAULT
        element.target_failure_behavior.proceed_if_able.SetInParent()
        if is_action:
            element.action_failure_behavior.retry_count = RETRY_COUNT_DEFAULT
            element.action_failure_behavior.prompt_duration.seconds = PROMPT_DURATION_DEFAULT
            element.action_failure_behavior.proceed_if_able.SetInParent()
        element.battery_monitor.battery_start_threshold = BATTERY_THRESHOLDS[0]
        element.battery_monitor.battery_stop_threshold = BATTERY_THRESHOLDS[1]
        return element

    def _toggle_record(self):
        """toggle recording on/off. Initial state is OFF"""
        if self._recording_client is not None:
            recording_status = self._recording_client.get_record_status()

            if not recording_status.is_recording:
                # Start recording map
                start_recording_response = self._recording_client.start_recording_full()
                if start_recording_response.status != recording_pb2.StartRecordingResponse.STATUS_OK:
                    self.add_message(f'Error starting recording (status = {start_recording_response.status}).')
                    return False
                else:
                    self.strings.rec = 'Stop Recording'
                    if not self.resumed_recording:
                        del self.walk.elements[:]
                        start_element = self._create_element(
                            'Start', start_recording_response.created_waypoint, is_action=False)
                        self.elements.append(start_element)

            else:
                # Stop recording map
                while True:
                    try:
                        # For some reason it doesn't work the first time, no matter what
                        stop_status = self._recording_client.stop_recording()
                        if stop_status != recording_pb2.StopRecordingResponse.STATUS_NOT_READY_YET:
                            break
                    except bosdyn.client.recording.NotReadyYetError:
                        time.sleep(0.1)

                if stop_status != recording_pb2.StopRecordingResponse.STATUS_OK:
                    self.add_message(f'Error stopping recording (status = {stop_status}).')
                    return False

                self.resumed_recording = True
                self.strings.rec = 'Resume Recording'

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto,
                                                     end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _battery_change_pose(self):
        # Default HINT_RIGHT, maybe add option to choose direction?
        self._start_robot_command(
            'battery_change_pose',
            RobotCommandBuilder.battery_change_pose_command(
                dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT))

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

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
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def _unstow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

    def _return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
            end_time_secs=time.time() + 20)

    def _toggle_ascii_video(self):
        if self._video_mode:
            self._video_mode = False
        else:
            self._video_mode = True

    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async('powering-on', self._request_power_on)
        else:
            self._try_grpc('powering-off', self._safe_power_off)

    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self, lease_keep_alive):
        if lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            try:
                _lease = lease_keep_alive.lease_wallet.get_lease()
                lease = f'{_lease.lease_proto.resource}:{_lease.lease_proto.sequence}'
            except bosdyn.client.lease.Error:
                lease = '...'
            if lease_keep_alive.is_alive():
                alive = 'RUNNING'
            else:
                alive = 'STOPPED'
        return f'Lease {lease} THREAD:{alive}'

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return f'Power: {state_str[6:]}'  # get rid of STATE_ prefix

    def _estop_str(self):
        if not self._estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
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


def _setup_logging(verbose):
    """Log to file at debug level, and log to console at INFO or DEBUG (if verbose).

    Returns the stream/console logger so that it can be removed when in curses mode.
    """
    LOGGER.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # Save log messages to file wasd.log for later debugging.
    file_handler = logging.FileHandler('wasd.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_formatter)
    LOGGER.addHandler(file_handler)

    # The stream handler is useful before and after the application is in curses-mode.
    if verbose:
        stream_level = logging.DEBUG
    else:
        stream_level = logging.INFO

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(stream_level)
    stream_handler.setFormatter(log_formatter)
    LOGGER.addHandler(stream_handler)
    return stream_handler


def main():
    """Command-line interface."""
    stream_handler = _setup_logging(False)

    # Create robot object.
    sdk = create_standard_sdk('WASDClient')
    robot = sdk.create_robot("192.168.80.3")
    try:
        bosdyn.client.util.authenticate(robot)
        robot.start_time_sync()
    except RpcError as err:
        LOGGER.error('Failed to communicate with robot: %s', err)
        return False

    wasd_interface = WasdInterface(robot)
    try:
        wasd_interface.start()
    except (ResponseError, RpcError) as err:
        LOGGER.error('Failed to initialize robot communication: %s', err)
        return False

    LOGGER.removeHandler(stream_handler)  # Don't use stream handler in curses mode.

    try:
        try:
            # Prevent curses from introducing a 1-second delay for ESC key
            os.environ.setdefault('ESCDELAY', '0')
            # Run wasd interface in curses mode, then restore terminal config.
            curses.wrapper(wasd_interface.drive)
        finally:
            # Restore stream handler to show any exceptions or final messages.
            LOGGER.addHandler(stream_handler)
    except Exception as e:
        LOGGER.error('WASD has thrown an error: [%r] %s', e, e)
    finally:
        # Do any final cleanup steps.
        wasd_interface.shutdown()

    return True


if __name__ == '__main__':
    if not main():
        os._exit(1)
    os._exit(0)
