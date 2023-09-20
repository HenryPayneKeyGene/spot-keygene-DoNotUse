#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT
from __future__ import annotations

import os
import threading
import time
from typing import Callable, Dict, List, Tuple

import bosdyn
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client
import cv2
import numpy as np
from bosdyn.api import image_pb2, world_object_pb2
from bosdyn.api.autowalk import walks_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.mission import mission_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.autowalk import AutowalkClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.util import authenticate, setup_logging
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.mission.client import MissionClient

from .exceptions import NoMissionRunningException
from .globals import VELOCITY_BASE_ANGULAR, VELOCITY_BASE_SPEED, VELOCITY_CMD_DURATION
from .tasks import AsyncImage, AsyncRobotState, update_tasks
from .util import get_img_source_list


class SpotClient:
    """
    A class to represent a Boston Dynamics Spot robot.

    This class is a wrapper around the Boston Dynamics SDK.
    """

    robot: bosdyn.client.Robot
    sdk: bosdyn.client.Sdk

    command_client: RobotCommandClient
    state_client: RobotStateClient
    lease_client: LeaseClient

    lease_keep_alive: LeaseKeepAlive | None

    def __init__(self, config: Dict[str, str], async_tasks=None):
        self.powered_on = False
        self.addr = config["addr"]
        self.name = config["name"]
        if async_tasks is None:
            async_tasks = []

        self.state: str = "startup"

        setup_logging()
        self.sdk = create_standard_sdk("keygene-client", [MissionClient])
        self.robot = self.sdk.create_robot(self.addr, self.name)
        self.logger = self.robot.logger
        self.logger.info("Starting up")

        authenticate(self.robot)
        self.logger.info("Authentication OK")

        self.robot_id = self.robot.get_id()
        self.robot.start_time_sync()

        # initialize clients
        self.command_client: RobotCommandClient = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client: RobotStateClient = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.lease_client: LeaseClient = self.robot.ensure_client(LeaseClient.default_service_name)
        self.estop_client: EstopClient = self.robot.ensure_client(EstopClient.default_service_name)
        self.world_object_client: WorldObjectClient = self.robot.ensure_client(WorldObjectClient.default_service_name)
        self.power_client: PowerClient = self.robot.ensure_client(PowerClient.default_service_name)
        self.mission_client: MissionClient = self.robot.ensure_client(MissionClient.default_service_name)
        self.autowalk_client: AutowalkClient = self.robot.ensure_client(AutowalkClient.default_service_name)
        self.graph_nav_client: GraphNavClient = self.robot.ensure_client(GraphNavClient.default_service_name)
        self.img_client: ImageClient = self.robot.ensure_client(ImageClient.default_service_name)

        try:
            self.estop_endpoint = EstopEndpoint(self.estop_client, 'kg-estop', 9.0)
        except Exception as _:
            # Not the estop.
            self.estop_endpoint = None

        self.robot_state_task = AsyncRobotState(self.state_client)
        self.image_task = AsyncImage(self.img_client, get_img_source_list(self.img_client))
        async_tasks = AsyncTasks([self.robot_state_task, self.image_task] + async_tasks)
        update_thread = threading.Thread(target=update_tasks, daemon=True, args=[async_tasks])
        update_thread.start()

        if self.estop_endpoint is not None:
            self.estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

        self.estop_endpoint.allow()

        self.estop_keep_alive = None
        self.exit_check = None
        self.lease_keep_alive = None

        self.graph_nav_client.clear_graph()

        self.logger.info("Spot initialized, startup complete.")

    def __del__(self):
        if self.state == "ready":
            self.shutdown()

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self.robot_state_task.proto

    @property
    def images(self):
        """Get latest images."""
        return self.image_task.proto

    # basics
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

        self.logger.info("Stopping time sync...")
        self.robot.time_sync.stop()
        self.logger.info("Time sync stopped")

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

    def toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self.estop_client is not None and self.estop_endpoint is not None:
            if not self.estop_keep_alive:
                self.estop_keep_alive = EstopKeepAlive(self.estop_endpoint)
            else:
                self._try_grpc('stopping estop', self.estop_keep_alive.stop)
                self.estop_keep_alive.shutdown()
                self.estop_keep_alive = None

    def toggle_time_sync(self):
        if self.robot.time_sync.stopped:
            self.robot.time_sync.start()
        else:
            self.robot.time_sync.stop()

    def toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.logger.error('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc("powering-on", self._request_power_on)
        else:
            self._try_grpc("powering-off", self.safe_power_off)

    def _request_power_on(self):
        bosdyn.client.power.power_on(self.power_client)

    def safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    # movement

    def _try_grpc(self, desc: str, thunk: Callable):
        try:
            return thunk()
        except (ResponseError, RpcError) as err:
            self.logger.error(f"Failed {desc}: {err}")
            return None

    def _start_robot_command(self, desc: str, command_proto, end_time_secs: float = None):

        def _start_command():
            self.command_client.robot_command(lease=None, command=command_proto,
                                              end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

    def move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

    def strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

    def strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

    def turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

    def turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

    def stop(self):
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
            end_time_secs=time.time() + 20)

    # world objects
    def get_visible_fiducials(self):
        """Return fiducials visible to robot."""
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        return self.world_object_client.list_world_objects(object_type=request_fiducials).world_objects

    def get_qr_tags(self):
        """Return QR tags visible to robot."""

        detector = cv2.QRCodeDetector()
        tags: List[Tuple[str, np.ndarray]] = []
        for image in self.images:
            if not image.source.image_type == image_pb2.ImageSource.IMAGE_TYPE_VISUAL:
                continue
            img = np.frombuffer(image.image.data, dtype=np.uint8).reshape(image.image.rows, image.image.cols, -1)
            data, bbox, _ = detector.detectAndDecode(img)
            if bbox is not None:
                tags.append((data, bbox))
        return tags

    # pose

    def scan_pose(self, duration=10):
        """Pose spot to scan."""
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command(body_height=1.2),
                                  end_time_secs=time.time() + duration)

    # autowalk
    def _upload_graph_and_snapshots(self, path, disable_alternate_route_finding=False, timeout=60):
        """
        Uploads the graph and snapshots to the robot.
        """

        # load the graph from the disk
        graph_filename = os.path.join(path, 'graph')
        self.logger.info(f"Loading graph from {graph_filename}")

        with open(graph_filename, 'rb') as graph_file:
            current_graph = map_pb2.Graph()
            current_graph.ParseFromString(graph_file.read())
            self.logger.info(
                f"Loaded graph with {len(current_graph.waypoints)} waypoints and {len(current_graph.edges)} edges")

        if disable_alternate_route_finding:
            self.logger.info("Disabling alternate route finding")
            for edge in current_graph.edges:
                edge.annotations.disable_alternate_route_finding = True

        # load waypoints from the disk
        current_waypoint_snapshots = dict()
        for waypoint in current_graph.waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            snapshot_filename = os.path.join(path, "waypoint_snapshots", waypoint.snapshot_id)
            self.logger.info(f"Loading waypoint snapshot from {snapshot_filename}")

            with open(snapshot_filename, 'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                current_waypoint_snapshots[waypoint_snapshot.snapshot_id] = waypoint_snapshot

        # load edges from the disk
        current_edge_snapshots = dict()
        for edge in current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            snapshot_filename = os.path.join(path, "edge_snapshots", edge.snapshot_id)
            self.logger.info(f"Loading edge snapshot from {snapshot_filename}")

            with open(snapshot_filename, 'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                current_edge_snapshots[edge_snapshot.snapshot_id] = edge_snapshot

        # upload the graph and snapshots to the robot
        self.logger.info("Uploading graph and snapshots to the robot...")
        anchors_are_empty = not len(current_graph.anchoring.anchors)
        response = self.graph_nav_client.upload_graph(graph=current_graph, generate_new_anchoring=anchors_are_empty,
                                                      timeout=timeout)
        self.logger.info(f"Uploaded graph.")

        for snapshot_id in response.unknown_snapshot_ids:
            waypoint_snapshot = current_waypoint_snapshots[snapshot_id]
            self.graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            self.logger.info(f"Uploaded waypoint snapshot {snapshot_id}.")

        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = current_edge_snapshots[snapshot_id]
            self.graph_nav_client.upload_edge_snapshot(edge_snapshot)
            self.logger.info(f"Uploaded edge snapshot {snapshot_id}.")

    def _upload_autowalk(self, filename, timeout=60):
        """
        Uploads the autowalk to the robot.
        """
        self.logger.info(f"Loading autowalk from {filename}")

        autowalk = walks_pb2.Autowalk()
        with open(filename, 'rb') as autowalk_file:
            autowalk.ParseFromString(autowalk_file.read())

        self.logger.info(f"Uploading autowalk to the robot...")
        self.autowalk_client.load_autowalk(autowalk, timeout=timeout)
        self.logger.info(f"Uploaded autowalk.")

    def upload_autowalk(self, path, disable_alternate_route_finding=False, timeout=60):
        """
        Uploads the autowalk to the robot.
        """
        self._upload_graph_and_snapshots(path, disable_alternate_route_finding, timeout)
        self._upload_autowalk(os.path.join(path, 'missions/autogenerate.walk'), timeout)

    def start_autowalk(self, timeout=60, disable_directed_exploration=False,
                       path_following_mode=map_pb2.Edge.Annotations.PATH_MODE_UNKNOWN):
        """
        Starts the autowalk.
        """
        self.logger.info("Starting autowalk...")
        mission_state: mission_pb2.State = self.mission_client.get_state()

        if mission_state.mission_id == -1:  # If no mission is loaded
            raise Exception("No mission is loaded. Please upload a mission first.")

        while mission_state in (mission_pb2.State.STATUS_NONE, mission_pb2.State.STATUS_RUNNING):
            if mission_state.questions:
                self.logger.info(f"Mission failed with questions: {mission_state.questions}")
                return False

            local_pause_time = time.time() + timeout

            play_settings = mission_pb2.PlaySettings(disable_directed_exploration=disable_directed_exploration,
                                                     path_following_mode=path_following_mode)
            lease = self.lease_client.lease_wallet.advance()
            self.mission_client.play_mission(local_pause_time, [lease], play_settings)
            time.sleep(1)

            mission_state = self.mission_client.get_state()

        self.logger.info(f"Mission status: {mission_state.Status.Name(mission_state.status)}")

        return mission_state.status in (mission_pb2.State.STATUS_SUCCESS, mission_pb2.State.STATUS_PAUSED)

    def stop_autowalk(self):
        """
        Stops the autowalk.
        """
        mission_state: mission_pb2.State = self.mission_client.get_state()
        if mission_state.status not in (mission_pb2.State.STATUS_RUNNING, mission_pb2.State.STATUS_PAUSED):
            raise NoMissionRunningException(f"Mission status: {mission_state.Status.Name(mission_state.status)}")

        self.logger.info("Stopping autowalk...")
        self.mission_client.stop_mission()
        time.sleep(1)

        mission_state = self.mission_client.get_state()
        self.logger.info(f"Mission status: {mission_state.Status.Name(mission_state.status)}")

        return mission_state.status in (mission_pb2.State.STATUS_SUCCESS, mission_pb2.State.STATUS_STOPPED)

    def pause_autowalk(self):
        """
        Pauses the autowalk.
        """
        mission_state: mission_pb2.State = self.mission_client.get_state()
        if mission_state.status != mission_pb2.State.STATUS_RUNNING:
            raise NoMissionRunningException(f"Mission status: {mission_state.Status.Name(mission_state.status)}")

        self.logger.info("Pausing autowalk...")
        self.mission_client.pause_mission()
        time.sleep(1)

        mission_state = self.mission_client.get_state()
        self.logger.info(f"Mission status: {mission_state.Status.Name(mission_state.status)}")

        return mission_state.status == mission_pb2.State.STATUS_PAUSED
