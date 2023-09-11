from bosdyn.api import geometry_pb2

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

# Velocity limits for navigation (optional)
NAV_VELOCITY_MAX_YAW = 1.2  # rad/s
NAV_VELOCITY_MAX_X = 1.0  # m/s
NAV_VELOCITY_MAX_Y = 0.5  # m/s
NAV_VELOCITY_LIMITS = geometry_pb2.SE2VelocityLimit(
    max_vel=geometry_pb2.SE2Velocity(
        linear=geometry_pb2.Vec2(x=NAV_VELOCITY_MAX_X, y=NAV_VELOCITY_MAX_Y),
        angular=NAV_VELOCITY_MAX_YAW), min_vel=geometry_pb2.SE2Velocity(
        linear=geometry_pb2.Vec2(x=-NAV_VELOCITY_MAX_X, y=-NAV_VELOCITY_MAX_Y),
        angular=-NAV_VELOCITY_MAX_YAW))
