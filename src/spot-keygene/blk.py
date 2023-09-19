#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import logging

import blk_arc_grpc.device_pb2 as device_message
from blk_arc_api.blk_arc import BLK_ARC
from blk_arc_api.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED


def connect(blk_arc=BLK_ARC(), logger=logging.getLogger(__name__)):
    # Connect:
    if not blk_arc.connect(CONNECTION_TYPE):
        logger.warning(
            "Could not connect, make sure you selected the right connection-type."
        )
        return

        # Get Device Info:
    device_info = blk_arc.get_device_info()
    if device_info:
        logger.info(
            "\nDevice info:\n"
            f"  Device Name:   {device_info.device_name}\n"
            f"  Serial Number: {device_info.serial_number}"
        )
    else:
        logger.warning("Could not get device-info.")

    # Get Firmware Version:
    firmware_version = blk_arc.get_firmware_version()
    if firmware_version:
        logger.info(f"Firmware version: {firmware_version}")
    else:
        logger.warning("Could not get firmware-version.")

    # Get Device Status:
    device_status = blk_arc.get_device_status()
    if device_status:
        device_states = device_message.DeviceStateResponse.State
        logger.info(
            f"Device is in state '{device_states.Name(device_status.state)}'."
        )
    else:
        logger.warning("Could not get device-status.")

    # Get Disk Information
    free_disk_space = blk_arc.get_free_disk_space_percentage()
    if free_disk_space:
        logger.info(f"Free disk space: {free_disk_space:.2f}%")
    else:
        logger.warning("Could not get free disk space information.")

    return blk_arc
