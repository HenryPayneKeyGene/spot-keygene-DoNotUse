#  Copyright (c) Romir Kulshrestha 2023.

"""Utility functions."""

import logging
import signal
import time

from bosdyn.api.image_pb2 import ImageSource

LOGGER = logging.getLogger()

# Mapping from visual to depth data
VISUAL_SOURCE_TO_DEPTH_MAP_SOURCE = {
    'frontleft_fisheye_image': 'frontleft_depth_in_visual_frame',
    'frontright_fisheye_image': 'frontright_depth_in_visual_frame'
}


class Universe(set):
    """overriding the set class to add a contains method that always returns true"""

    def __contains__(self, _):
        return True


class ExitCheck:
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


def get_img_source_list(image_client):
    """Gets a list of image sources and filters based on config dictionary

    Args:
        image_client: Instantiated image client
    """

    # We are using only the visual images with their corresponding depth sensors
    sources = image_client.list_image_sources()
    source_list = []
    for source in sources:
        if source.image_type == ImageSource.IMAGE_TYPE_VISUAL:
            source_list.append(source.name)
    return source_list


def id_to_short_code(_id):
    """Convert a unique id to a 2 letter short code."""
    tokens = _id.split('-')
    if len(tokens) > 2:
        return '%c%c' % (tokens[0][0], tokens[1][0])
    return None


def find_unique_waypoint_id(short_code, graph, name_to_id):
    """Convert either a 2 letter short code or an annotation name into the associated unique id."""
    if graph is None:
        print(
            "Please list the waypoints in the map before trying to navigate to a specific one (Option #4)."
        )
        return

    if len(short_code) != 2:
        # Not a short code, check if it is an annotation name (instead of the waypoint id).
        if short_code in name_to_id:
            # Short code is a waypoint's annotation name. Check if it is paired with a unique waypoint id.
            if name_to_id[short_code] is not None:
                # Has an associated waypoint id!
                return name_to_id[short_code]
            else:
                print(
                    f"The waypoint name {short_code} is used for multiple different unique waypoints. Please use the "
                    "waypoint id.")
                return None
        # Also not a waypoint annotation name, so we will operate under the assumption that it is a
        # unique waypoint id.
        return short_code

    ret = short_code
    for waypoint in graph.waypoints:
        if short_code == id_to_short_code(waypoint.id):
            if ret != short_code:
                return short_code  # Multiple waypoints with same short code.
            ret = waypoint.id
    return ret


def sort_waypoints_chrono(graph):
    """Sort waypoints by time created."""
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        # Determine the timestamp that this waypoint was created at.
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except Exception as _:
            # Must be operating on an older graph nav map, since the creation_time is not
            # available within the waypoint annotations message.
            pass
        waypoint_to_timestamp.append((waypoint.id, timestamp, waypoint.annotations.name))

    # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
    # fallback to sorting by annotation name.
    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key=lambda x: (x[1], x[2]))

    return waypoint_to_timestamp


def countdown(length):
    """Print sleep countdown"""

    for i in range(length, 0, -1):
        print(i, end=' ', flush=True)
        time.sleep(1)
    print(0)
