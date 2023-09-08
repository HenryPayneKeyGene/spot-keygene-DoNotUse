from blk_arc_api.blk_arc_config import ConnectionType
from blk_arc_api.blk_arc import BLK_ARC

import blk_arc_grpc.device_pb2 as device_message

import logging

import time

CONNECTION_TYPE = ConnectionType.WIRED


class BLK:
    def __init__(self):
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - (%(filename)s:%(lineno)d): %(message)s",
        )
        self.blk_arc = BLK_ARC()

        # Connect:
        if not self.blk_arc.connect(CONNECTION_TYPE):
            logging.warning(
                "Could not connect, make sure you selected the right connection-type."
            )
            return

            # Get Device Info:
        device_info = self.blk_arc.get_device_info()
        if device_info:
            logging.info(
                "\nDevice info:\n"
                f"  Device Name:   {device_info.device_name}\n"
                f"  Serial Number: {device_info.serial_number}"
            )
        else:
            logging.warning("Could not get device-info.")

        # Get Firmware Version:
        firmware_version = self.blk_arc.get_firmware_version()
        if firmware_version:
            logging.info(f"Firmware version: {firmware_version}")
        else:
            logging.warning("Could not get firmware-version.")

        # Get Device Status:
        device_status = self.blk_arc.get_device_status()
        if device_status:
            device_states = device_message.DeviceStateResponse.State
            logging.info(
                f"Device is in state '{device_states.Name(device_status.state)}'."
            )
        else:
            logging.warning("Could not get device-status.")

        # Get Disk Information
        free_disk_space = self.blk_arc.get_free_disk_space_percentage()
        if free_disk_space:
            logging.info(f"Free disk space: {free_disk_space:.2f}%")
        else:
            logging.warning("Could not get free disk space information.")

        # self.blk_arc.disconnect()

    def __del__(self):
        self.blk_arc.disconnect()

    def start_capture(self):
        logging.info("Starting a scan.")
        start_scan_info = self.blk_arc.start_capture()
        if start_scan_info:
            # Wait for the scan to be initialized:
            while not self.blk_arc.is_scanning():
                time.sleep(0.5)
            logging.info(f"Initialized scan with ID: {start_scan_info.scan_id}.")

            # Mark a static pose:
            input(
                "Hit <Enter> to mark the beginning of a static pose. Only use this when the scanner is stationary."
            )
            logging.info("Marking the beginning of a static pose.")
            self.blk_arc.begin_static_pose()

            input("Hit <Enter> to mark the end of the static pose.")
            logging.info("Marking the end of a static pose.")
            self.blk_arc.end_static_pose()

            # Stop the scan:
            input("Hit <Enter> to stop the scan.")
            logging.info("Stopping the scan.")
            self.blk_arc.stop_capture()
            logging.info("Stopped the scan.")
        else:
            logging.warning("Could not start a scan.")
