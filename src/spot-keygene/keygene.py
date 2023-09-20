#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT
import time
from logging import Logger

from blk_arc_api.blk_arc import BLK_ARC
from .blk import connect
from .spot_client import SpotClient


def scan(lidar: BLK_ARC, spot: SpotClient, logger: Logger):
    start_time = time.time()
    duration = 30
    end_time = start_time + duration

    logger.info(f"Starting lidar scan for {duration} seconds.")

    spot.scan_pose(duration)
    scan_id = lidar.start_capture()
    attempts = 0
    while not lidar.is_scanning() and attempts < 20:
        time.sleep(0.5)
        attempts += 1
    time.sleep(end_time - time.time())
    if not lidar.is_scanning():
        logger.error("Failed to start lidar scan.")
        return
    lidar.stop_capture()
    spot.stand()
    return scan_id


def upload(lidar, spot, logger):
    pass


COMMANDS = {
    "scan": scan,
    "upload": upload,
}


def keygene_main(logger=None):
    """
    Main function for keygene.
    """

    config = {
        "name": "spot-keygene",
        "addr": "192.18.80.3",
        "path": "./autowalks/scan.walk",
    }
    spot: SpotClient = SpotClient(config)
    lidar: BLK_ARC = connect()
    logger: Logger = logger or spot.logger

    spot.upload_autowalk(config["path"])
    spot.acquire()
    spot.start_autowalk()

    processed_tags = set()
    scans = set()

    while True:
        qrs = spot.get_qr_tags()
        if not qrs:
            time.sleep(0.2)
            continue

        for data, bbox in qrs:
            try:
                tag, command = data.split(":")
            except ValueError:
                continue

            if tag in processed_tags or command not in COMMANDS:
                continue

            logger.info(f"Found tag: {tag} with command: {command}")

            if not spot.pause_autowalk():
                raise Exception("Failed to pause autowalk.")

            if command == "scan":
                scan_id = COMMANDS[command](lidar, spot, logger)
                scans.add(scan_id)
            elif command == "upload":
                upload(lidar, spot, logger)

            if not spot.start_autowalk():
                raise Exception("Failed to start autowalk.")
