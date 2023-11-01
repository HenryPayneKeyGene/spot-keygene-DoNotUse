"""
spotkg
============

A Python package developed for the Spot Keygene project. This package is
intended to be used with the Spot SDK, and provides a number of useful
functions for interacting with Spot, as well as services for controlling
a BLK-ARC LIDAR.
"""
from argparse import Namespace

import importlib_metadata

try:
    __version__ = importlib_metadata.version("keygene")
except importlib_metadata.PackageNotFoundError:
    __version__ = "unknown (check package metadata)"


def recorder_driver():
    from . import recording
    recording.start_recording(Namespace(hostname="192.168.80.3"))
