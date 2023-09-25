#  Copyright (c) Romir Kulshrestha 2023.
class NoMissionRunningException(Exception):
    """
    Exception thrown when no mission is running.
    """

    def __init__(self, message=None):
        super().__init__("No mission is running.\n" + (message or " Please start a mission first."))


class AutowalkStartError(Exception):
    pass
