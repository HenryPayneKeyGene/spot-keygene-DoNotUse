#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import logging
import time

import blk_arc_grpc.device_pb2 as device_message
from blk_arc_api.blk_arc import BLK_ARC
from blk_arc_api.blk_arc_config import ConnectionType

CONNECTION_TYPE = ConnectionType.WIRED


class BLK:
    def __init__(self):
        self.blk_arc = BLK_ARC()
        self.logger = logging.getLogger(__name__)

        # Connect:
        if not self.blk_arc.connect(CONNECTION_TYPE):
            self.logger.warning(
                "Could not connect, make sure you selected the right connection-type."
            )
            return

            # Get Device Info:
        device_info = self.blk_arc.get_device_info()
        if device_info:
            self.logger.info(
                "\nDevice info:\n"
                f"  Device Name:   {device_info.device_name}\n"
                f"  Serial Number: {device_info.serial_number}"
            )
        else:
            self.logger.warning("Could not get device-info.")

        # Get Firmware Version:
        firmware_version = self.blk_arc.get_firmware_version()
        if firmware_version:
            self.logger.info(f"Firmware version: {firmware_version}")
        else:
            self.logger.warning("Could not get firmware-version.")

        # Get Device Status:
        device_status = self.blk_arc.get_device_status()
        if device_status:
            device_states = device_message.DeviceStateResponse.State
            self.logger.info(
                f"Device is in state '{device_states.Name(device_status.state)}'."
            )
        else:
            self.logger.warning("Could not get device-status.")

        # Get Disk Information
        free_disk_space = self.blk_arc.get_free_disk_space_percentage()
        if free_disk_space:
            self.logger.info(f"Free disk space: {free_disk_space:.2f}%")
        else:
            self.logger.warning("Could not get free disk space information.")

        # self.blk_arc.disconnect()

    def __del__(self):
        self.blk_arc.disconnect()

    def start_capture(self):
        self.logger.info("Starting a scan.")
        start_scan_info = self.blk_arc.start_capture()
        if start_scan_info:
            # Wait for the scan to be initialized:
            while not self.blk_arc.is_scanning():
                time.sleep(0.5)
            self.logger.info(f"Initialized scan with ID: {start_scan_info.scan_id}.")

            # Mark a static pose:
            input(
                "Hit <Enter> to mark the beginning of a static pose. Only use this when the scanner is stationary."
            )
            self.logger.info("Marking the beginning of a static pose.")
            self.blk_arc.begin_static_pose()

            input("Hit <Enter> to mark the end of the static pose.")
            self.logger.info("Marking the end of a static pose.")
            self.blk_arc.end_static_pose()

            # Stop the scan:
            input("Hit <Enter> to stop the scan.")
            self.logger.info("Stopping the scan.")
            self.blk_arc.stop_capture()
            self.logger.info("Stopped the scan.")
        else:
            self.logger.warning("Could not start a scan.")

    def shutdown(self):
        self.blk_arc.disconnect()
