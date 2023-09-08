import logging

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot_state import RobotStateClient

LOGGER = logging.getLogger()


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client: RobotStateClient):
        super().__init__("robot_state", robot_state_client, LOGGER, period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


def update_waypoints_and_edges(graph, localization_id, do_print=True):
    """Update and print waypoint ids and edge ids."""
    name_to_id = dict()
    edges = dict()

    short_code_to_count = {}
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        # Determine the timestamp that this waypoint was created at.
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except:
            # Must be operating on an older graph nav map, since the creation_time is not
            # available within the waypoint annotations message.
            pass
        waypoint_to_timestamp.append((waypoint.id, timestamp, waypoint.annotations.name))

        # Determine how many waypoints have the same short code.
        short_code = id_to_short_code(waypoint.id)
        if short_code not in short_code_to_count:
            short_code_to_count[short_code] = 0
        short_code_to_count[short_code] += 1

        # Add the annotation name/id into the current dictionary.
        waypoint_name = waypoint.annotations.name
        if waypoint_name:
            if waypoint_name in name_to_id:
                # Waypoint name is used for multiple different waypoints, so set the waypoint id
                # in this dictionary to None to avoid confusion between two different waypoints.
                name_to_id[waypoint_name] = None
            else:
                # First time we have seen this waypoint annotation name. Add it into the dictionary
                # with the respective waypoint unique id.
                name_to_id[waypoint_name] = waypoint.id

    # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
    # fallback to sorting by annotation name.
    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key=lambda x: (x[1], x[2]))

    # Print out the waypoints name, id, and short code in an ordered sorted by the timestamp from
    # when the waypoint was created.
    if do_print:
        print(f'{len(graph.waypoints):d} waypoints:')
        for waypoint in waypoint_to_timestamp:
            pretty_print_waypoints(waypoint[0], waypoint[2], short_code_to_count, localization_id)

    for edge in graph.edges:
        if edge.id.to_waypoint in edges:
            if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
        else:
            edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
        if do_print:
            print(f'(Edge) from waypoint {edge.id.from_waypoint} to waypoint {edge.id.to_waypoint} '
                  f'(cost {edge.annotations.cost.value})')

    return name_to_id, edges
