# Copyright (c) Romir Kulshrestha 2023.

from bosdyn.api import geometry_pb2

DOCK_ID = 520
"""The dock ID to use for docking."""

VELOCITY_BASE_SPEED = 0.5  # m/s
"""The base speed to use for velocity commands."""
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
"""The base angular speed to use for velocity commands."""
VELOCITY_CMD_DURATION = 0.6  # seconds
"""The duration to use for velocity commands."""

COMMAND_INPUT_RATE = 0.1
"""The rate at which to check for user input."""

# Velocity limits for navigation (optional)
NAV_VELOCITY_MAX_YAW = 1.2  # rad/s
"""The maximum yaw velocity for navigation."""
NAV_VELOCITY_MAX_X = 0.5  # m/s
"""The maximum x velocity for navigation."""
NAV_VELOCITY_MAX_Y = 0.5  # m/s
"""The maximum y velocity for navigation."""
NAV_VELOCITY_LIMITS: geometry_pb2.SE2VelocityLimit = geometry_pb2.SE2VelocityLimit(
    max_vel=geometry_pb2.SE2Velocity(
        linear=geometry_pb2.Vec2(x=NAV_VELOCITY_MAX_X, y=NAV_VELOCITY_MAX_Y),
        angular=NAV_VELOCITY_MAX_YAW), min_vel=geometry_pb2.SE2Velocity(
        linear=geometry_pb2.Vec2(x=-NAV_VELOCITY_MAX_X, y=-NAV_VELOCITY_MAX_Y),
        angular=-NAV_VELOCITY_MAX_YAW))
"""The velocity limits for navigation."""

ACTIONS = ["scan", "upload", "image"]
"""The actions that can be performed by Keygene."""
