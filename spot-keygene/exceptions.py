#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

class NoMissionRunningException(Exception):
    """
    Exception thrown when no mission is running.
    """

    def __init__(self, message=None):
        super().__init__("No mission is running.\n" + (message or " Please start a mission first."))
