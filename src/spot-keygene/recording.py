from __future__ import annotations

import os
import time

import bosdyn.client
import bosdyn.client.graph_nav
import bosdyn.client.map_processing
import bosdyn.client.recording
from bosdyn.api.graph_nav import recording_pb2
from tqdm import tqdm


class RecordingInterface:
    def __init__(
            self,
            robot: bosdyn.client.Robot,
            download_path: str,
            rec_client: bosdyn.client.recording.GraphNavRecordingServiceClient,
            gn_client: bosdyn.client.graph_nav.GraphNavClient,
            mp_client: bosdyn.client.map_processing.MapProcessingServiceClient,
            client_meta,
    ) -> None:
        self.robot: bosdyn.client.Robot = robot
        self.download_path = download_path
        self.graph_nav_client = gn_client
        self.map_processing_client = mp_client
        self.recording_client = rec_client

        self.recording_environment = bosdyn.client.recording.GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=bosdyn.client.recording.GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_meta
            )
        )

        # graph data
        self.current_graph = None
        self.current_edges = dict()  # to_waypoint -> list[from_waypoint]
        self.current_waypoint_snapshots = dict()  # id -> wp snapshot
        self.current_edge_snapshots = dict()  # id -> edge snapshot
        self.current_annotation_name_to_wp_id = dict()

    def should_start_recording(self) -> bool:
        # check GraphNav state
        graph = self.graph_nav_client.download_graph()
        if graph is not None:
            # check for waypoints. if exists, localize to graph before recording
            if len(graph.waypoints) > 0:
                localization_state = self.graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoints_id:
                    # not localized to anything in map
                    # should clear graph or attempt to localize in current map
                    return False

        # there is either a graph we are localized to or no graph. ok to start recording
        return True

    def __clear_map(self):
        """clear map on the robot"""
        return self.graph_nav_client.clear_graph()

    def __start_recording(self):
        """Start recording a map."""
        should_start = self.should_start_recording()
        if not should_start:
            self.robot.logger.warn(
                "System not in a proper state to begin recording map."
            )
            return
        try:
            status = self.recording_client.start_recording(
                recording_environment=self.recording_environment
            )
            self.robot.logger.info("Started recording map.")
        except Exception as e:
            self.robot.logger.error(f"Start recording failed: {e}")

    def __stop_recording(self):
        """Stop or pause recording a map."""
        first_iter = True
        while True:
            try:
                status = self.recording_client.stop_recording()
                self.robot.logger.info("Stopped recording a map.")
                return

            except bosdyn.client.recording.NotReadyYetError as err:
                if first_iter:
                    self.robot.logger.info("Cleaning up recording...")
                first_iter = False
                time.sleep(1)
                continue
            except Exception as e:
                self.robot.logger.error(f"Stop recording failed: {e}")
                return

    def __display_recording_status(self):
        """Get recording service's status."""

        self.robot.logger.info(
            f"The recording service is {'on' if self.recording_client.get_record_status().is_recording else 'off'}."
        )

    def __create_default_waypoint(self):
        """Create a default waypoint at the current location."""
        resp = self.recording_client.create_waypoint(waypoint_name="default")
        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            self.robot.logger.info("Created default waypoint.")
        else:
            self.robot.logger.error("Could not create default waypoint.")

    def __download_full_graph(self):
        """Download the graph and snapshots from the robot."""
        graph = self.graph_nav_client.download_graph()
        if graph is None:
            self.robot.logger.error("Failed to download graph.")
            return
        self.__write_full_graph(graph)
        self.robot.logger.info(
            f"Graph downloaded with {len(graph.waypoints)} waypoints and {len(graph.edges)} edges."
        )

        # download wp and ed snapshots
        self.__download_and_write_wp_snapshots(graph.waypoints)
        self.__download_and_write_ed_snapshots(graph.edges)

    def __write_full_graph(self, graph):
        """Download graph to a local filepath."""
        self.__write_bytes(self.download_path, "graph", graph.SerializeToString())

    def __download_and_write_wp_snapshots(self, waypoints):
        """Download waypoint snapshots to a local filepath."""
        num_wp_snaps_downloaded = 0
        for wp in tqdm(waypoints):
            if len(wp.snapshot_id) == 0:
                continue
            try:
                wp_snap = self.graph_nav_client.download_waypoint_snapshot(
                    wp.snapshot_id
                )
            except Exception as _:
                self.robot.logger.error(
                    f"Failed to download waypoint snapshot: {wp.snapshot_id}"
                )
                continue
            self.__write_bytes(
                os.path.join(self.download_path, "wp_snapshots"),
                str(wp.snapshot_id),
                wp_snap.SerializeToString(),
            )
            num_wp_snaps_downloaded += 1

    def __download_and_write_ed_snapshots(self, edges):
        """Download edge snapshots to a local filepath."""
        num_ed_snaps_downloaded = 0
        for ed in tqdm(edges):
            if len(ed.snapshot_id) == 0:
                continue
            try:
                ed_snap = self.graph_nav_client.download_edge_snapshot(ed.snapshot_id)
            except Exception as _:
                self.robot.logger.error(
                    f"Failed to download edge snapshot: {ed.snapshot_id}"
                )
                continue
            self.__write_bytes(
                os.path.join(self.download_path, "ed_snapshots"),
                str(ed.snapshot_id),
                ed_snap.SerializeToString(),
            )
            num_ed_snaps_downloaded += 1

    def __write_bytes(self, filepath: str, filename: str, data: bytes):
        """Write data to a file."""
        os.makedirs(filepath, exist_ok=True)
        with open(os.path.join(filepath, filename), "wb+") as f:
            f.write(data)

    def __update_ids(self, do_print=False):
        graph = self.graph_nav_client.download_graph()
        if graph is None:
            self.robot.logger.warn("Empty graph.")
            return
        self.current_graph = graph

        loc_id = self.graph_nav_client.get_localization_state().localization.waypoint_id

        # update and print wp and ed
        self.current_annotation_name_to_wp_id
