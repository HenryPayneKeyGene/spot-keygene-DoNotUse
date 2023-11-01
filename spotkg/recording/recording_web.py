#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT
import logging
import os
import threading
import time

import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.client.lease
from bosdyn import geometry
from bosdyn.api.autowalk import walks_pb2
from bosdyn.api.graph_nav import recording_pb2, graph_nav_pb2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.graph_nav import CannotModifyMapDuringRecordingError
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotReadyYetError
from nicegui import ui, app
from nicegui.elements.slider import Slider
from nicegui.events import KeyEventArguments

from ..spot_client import SpotClient

LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

IMAGE_SOURCES = ["frontright_fisheye_image", "frontleft_fisheye_image", "right_fisheye_image", "back_fisheye_image",
                 "left_fisheye_image", ]
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


class RecorderGUI(object):
    """A web interface for driving the robot."""

    spot: SpotClient
    walk: walks_pb2.Walk
    directory: str
    walk_name: str

    _recording_client: GraphNavRecordingServiceClient
    _estop_client: EstopClient
    _estop_endpoint: EstopEndpoint

    linear_velocity: Slider
    angular_velocity: Slider

    _command_dictionary: dict

    def __init__(self, hostname: str):
        self.walk = self._init_walk()
        self.directory = os.path.join(os.getcwd(), "autowalks")
        self.hostname = hostname

        # Default starting speed values for the robot
        self.command_duration = COMMAND_DURATION_DEFAULT

        # Experimentally determined default starting values for pitch, roll, yaw, and height
        self.euler_angles = geometry.EulerZXY()
        self.robot_height = 0

        self._lock = threading.Lock()

        self.resumed_recording = False
        self.elements = []
        self.fiducial_objects = []
        self.dock = None
        self.dock_and_end_recording = False

        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None

        self.start()

        ui.run(reload=True)  # , native=True, window_size=(835, 460))

    def connect(self, config):
        self.spot = SpotClient(config)

        self.walk_name = self.spot.robot.get_id().nickname + f"_{time.strftime('%Y%m%d_%H%M%S')}"

        # Initialize clients
        self._recording_client: GraphNavRecordingServiceClient = self.spot.robot.ensure_client(
            GraphNavRecordingServiceClient.default_service_name)

        # Clear graph to ensure only the data recorded using this example gets packaged into map
        try:
            self.spot.graph_nav_client.clear_graph()
        except CannotModifyMapDuringRecordingError:
            self._toggle_record()

        try:
            self._estop_client = self.spot.robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'KGRecClient', 9.0)
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None

        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._command_dictionary = {27: self._stop,  # ESC key
                                    '\t': self._stop, ord('T'): self.spot.toggle_time_sync,
                                    ' ': self._toggle_estop,
                                    'r': self.spot.self_right, ord('P'): self.spot.toggle_power,
                                    'v': self.spot.sit,
                                    'f': self.spot.stand, ord('w'): self.spot.move_forward,
                                    's': self.spot.move_backward,
                                    'a': self.spot.strafe_left, ord('d'): self.spot.strafe_right,
                                    'q': self.spot.turn_left,
                                    'e': self.spot.turn_right, ord('u'): self.spot.unstow,
                                    'j': self.spot.stow,
                                    'l': self.spot.toggle_lease,

                                    # 'vel': self.spot.cmd_vel,

                                    # recording actions
                                    '1': self._toggle_record, ord('2'): self._save_autowalk, }
        self.spot.logger.warning("starting recorder ui")
        self.rec_ui()

    def __del__(self):
        if not hasattr(self, "spot"):
            return
        if hasattr(self, "_recording_client"):
            recording_status = self._recording_client.get_record_status()

            if recording_status.is_recording:
                self._toggle_record()

        self.shutdown()

    def _stop(self):
        self.shutdown()
        app.shutdown()

    @staticmethod
    def _init_walk():
        """Initialize walk object"""

        walk = walks_pb2.Walk()
        walk.global_parameters.self_right_attempts = RETRY_COUNT_DEFAULT
        walk.playback_mode.once.SetInParent()
        return walk

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self.spot.robot_state

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

    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
                ui.notify('Estop deactivated.')
            else:
                self.spot.try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None
                ui.notify('Estop activated.')

    def _toggle_record(self):
        """toggle recording on/off. Initial state is OFF"""
        if self._recording_client is not None:
            recording_status = self._recording_client.get_record_status()

            if not recording_status.is_recording:
                # Start recording map
                if not self.resumed_recording and not len(self.spot.get_visible_fiducials()):
                    ui.notify('No fiducials visible. Recording will not start until fiducials are visible.')
                    return False
                start_recording_response = self._recording_client.start_recording_full()
                if start_recording_response.status != recording_pb2.StartRecordingResponse.STATUS_OK:
                    ui.notify(f'Error starting recording (status = {start_recording_response.status}).')
                    return False
                else:
                    # self.strings.rec = 'Stop Recording'
                    if not self.resumed_recording:
                        del self.walk.elements[:]
                        start_element = self._create_element(
                            'Start', start_recording_response.created_waypoint, is_action=False)
                        self.elements.append(start_element)
                ui.notify('Recording started.')
            else:
                # Stop recording map
                while True:
                    try:
                        # For some reason it doesn't work the first time, no matter what
                        stop_status = self._recording_client.stop_recording()
                        if stop_status != recording_pb2.StopRecordingResponse.STATUS_NOT_READY_YET:
                            break
                    except NotReadyYetError:
                        time.sleep(0.1)

                if stop_status != recording_pb2.StopRecordingResponse.STATUS_OK:
                    ui.notify(f'Error stopping recording (status = {stop_status}).')
                    return False

                self.resumed_recording = True
                ui.notify('Recording stopped.')
                # self.strings.rec = 'Resume Recording'

    def _save_autowalk(self):
        """Save autowalk in directory"""

        self.directory = self.dir_text.value
        self.walk_name = self.walk_text.value
        self.walk.mission_name = self.walk_name
        ui.notify(f"Saving to {self.walk_name}.walk...")

        if not self.dock_and_end_recording:
            # Creates end waypoint and action if the last action is not docking
            self._toggle_record()
            waypoint_response = self._recording_client.create_waypoint()
            end_element = self._create_element('End Recording', waypoint_response.created_waypoint, is_action=False)
            self.elements.append(end_element)
            self._toggle_record()

        # Creates walk directory and saves autowalk in format
        walk_directory = os.path.join(self.directory, f'{self.walk_name}.walk')
        directory_copy_number = 0
        while os.path.isdir(walk_directory):
            # Appends (number) to folder name if folder with existing name exists
            directory_copy_number += 1
            walk_directory = os.path.join(self.directory, f'{self.walk_name}({directory_copy_number}).walk')
        self.spot.graph_nav_client.write_graph_and_snapshots(walk_directory)
        self.walk.elements.extend(self.elements)
        mission_directory = os.path.join(walk_directory, 'missions')
        os.mkdir(mission_directory)
        with open(os.path.join(mission_directory, 'autogenerated.walk'), 'wb') as autowalk_file:
            autowalk_file.write(self.walk.SerializeToString())

        self._reset_walk()
        ui.notify("Saved recording.")

    def _reset_walk(self):
        """Resets walk parameters and GUI"""
        self.walk = self._init_walk()
        # self.strings.rec = 'Start Recording'
        self.resumed_recording = False
        self.dock_and_end_recording = False
        self.spot.graph_nav_client.clear_graph()

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        # self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
        #                                        return_at_exit=True)

        self.connect({"name": "KGRecorder", "addr": self.hostname})

        def keybinds(e: KeyEventArguments):
            # self.spot.logger.warning(f"keybinds: {e.key.name}")
            if not e.action.keydown:
                return
            if e.modifiers.ctrl:
                if e.key == 'c':
                    self._stop()
                elif e.key == 'x':
                    self._toggle_estop()
                elif e.key == 'r':
                    self._toggle_record()
                elif e.key == 's':
                    self._save_autowalk()
                return
            elif e.modifiers.shift:
                if e.key == 'P':
                    self.spot.toggle_power()
                return
            k = e.key.name
            if k in self._command_dictionary:
                self._command_dictionary[k]()

        ui.keyboard(on_key=keybinds)

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info('Shutting down.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        del self.spot

    def _lease_str(self):
        if self.spot.lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            lease_keep_alive = self.spot.lease_keep_alive
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
        power_state = self.robot_state.power_state.motor_power_state
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

        def secs_to_hms(seconds):
            """Format a time in seconds as 'H:MM:SS'

            Args:
             seconds:   number of seconds (will be truncated to integer value)
            """
            isecs = int(seconds)
            seconds = isecs % 60
            minutes = (isecs // 60) % 60
            hours = isecs // 3600
            return '{:d}:{:02d}:{:02d}'.format(hours, minutes, seconds)

        if battery_state.estimated_runtime:
            time_left = f'({secs_to_hms(battery_state.estimated_runtime.seconds)})'
        return f'Battery: {status}{bat_bar} {time_left}'

    def connect_ui(self):
        with ui.card() as card:
            namebox = ui.input('Name', value="spot")
            addrbox = ui.input('Address', value="192.168.80.3")
            btn = ui.button('Connect')

            def on_connect():
                namebox.disable()
                addrbox.disable()
                btn.disable()
                ui.open(self.connect({"name": namebox.value, "addr": addrbox.value}))

            btn.on("click", on_connect)

    def update_strings(self):
        self.lease_label.set_text(self._lease_str())
        self.power_label.set_text(self._power_state_str())
        self.estop_label.set_text(self._estop_str())
        self.battery_label.set_text(self._battery_str())

        self.power_btn.set_text("Power" if self._power_state_str()[7:] == "OFF" else "Power Off")

        if self.spot.lease_keep_alive is not None and self.spot.lease_keep_alive.is_alive():
            self.lease_btn.set_text("Release")
            if self._power_state_str()[7:] == "OFF":
                self.power_btn.enable()
                self.power_btn.set_text("Power")
            else:
                self.power_btn.set_text("Power Off")
        else:
            self.lease_btn.set_text("Lease")
            self.power_btn.disable()
            self.power_btn.set_text("Power")

        if self.power_btn.text == "Power":
            self.joy.active = False
            self.sit_btn.disable()
            self.unstow_btn.disable()
            self.stand_btn.disable()
            self.stow_btn.disable()
        else:
            self.joy.active = True
            self.sit_btn.enable()
            self.unstow_btn.enable()
            self.stand_btn.enable()
            self.stow_btn.enable()

        if not self._recording_client.get_record_status().is_recording:
            self.rec_btn.set_text("Record")
            if not self.resumed_recording:
                self.save_btn.disable()
                if not len(self.spot.get_visible_fiducials()):
                    self.rec_btn.disable()
                else:
                    self.rec_btn.enable()
            else:
                self.save_btn.enable()
        else:

            self.rec_btn.set_text("Stop Rec")
            self.save_btn.disable()

    def rec_ui(self):
        cmd_vel = self.spot.cmd_vel
        ui.dark_mode(True)

        with ui.row().classes('items-stretch'):
            with ui.card().classes('w-44 text-center items-center'):
                ui.label('Control').classes('text-2xl')
                self.joy = ui.joystick(color='blue', size=50,
                                       on_move=lambda e: cmd_vel(float(e.y) * LINEAR_VELOCITY_DEFAULT,
                                                                 float(-e.x) * ANGULAR_VELOCITY_DEFAULT),
                                       on_end=lambda _: cmd_vel(0.0, 0.0))
                ui.label('Publish steering commands by dragging your mouse around in the blue field').classes(
                    'mt-6')
            with ui.card().classes('w-50 text-center items-center'):
                # ui.label('Data').classes('text-2xl')
                # ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                # slider_props = 'readonly selection-color=transparent'
                # self.linear_velocity = ui.slider(min=-LINEAR_VELOCITY_DEFAULT, max=LINEAR_VELOCITY_DEFAULT,
                #                                  step=0.05, value=0).props(slider_props)
                # ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                # self.angular_velocity = ui.slider(min=-ANGULAR_VELOCITY_DEFAULT, max=ANGULAR_VELOCITY_DEFAULT,
                #                                   step=0.05, value=0).props(slider_props)
                # ui.label('position').classes('text-xs mb-[-1.4em]')  # self.position = ui.label('---')
                with ui.column().classes('items-center'):
                    with ui.row().classes('items-stretch'):
                        with ui.column().classes('items-center btns'):
                            self.power_btn = ui.button('Power', on_click=self.spot.toggle_power)
                            self.sit_btn = ui.button('Sit', on_click=self.spot.sit)
                            self.unstow_btn = ui.button('Unstow', on_click=self.spot.unstow)
                            self.rec_btn = ui.button('Record', on_click=self._toggle_record)
                        with ui.column().classes('items-center btns'):
                            # ui.button('Lease', on_click=self.spot.toggle_lease)
                            # ui.button('Stand', on_click=self.spot.stand)
                            # ui.button('Stow', on_click=self.spot.stow)
                            # ui.button('Save', on_click=self._save_autowalk)
                            self.lease_btn = ui.button('Lease', on_click=self.spot.toggle_lease)
                            self.stand_btn = ui.button('Stand', on_click=self.spot.stand)
                            self.stow_btn = ui.button('Stow', on_click=self.spot.stow)
                            self.save_btn = ui.button('Save', on_click=self._save_autowalk)

                    self.estop_btn = ui.button('estop', on_click=self._toggle_estop).style(
                        "background-color: red; color: white;").tailwind.background_color('red')

                ui.query(".btns button").style("width: 100%")

            with ui.card().classes('w-100'):
                with ui.column():
                    self.lease_label = ui.label(self._lease_str())
                    self.power_label = ui.label(self._power_state_str())
                    self.estop_label = ui.label(self._estop_str())
                    self.battery_label = ui.label(self._battery_str())
                    self.dir_text = ui.input('Directory', value=self.directory).style('width: 100%')
                    self.walk_text = ui.input('Walk Name', value=self.walk_name).style('width: 100%')

        ui.timer(0.5, self.update_strings)


def start_recording(options):
    """Start the recording service."""
    RecorderGUI(options.hostname)
