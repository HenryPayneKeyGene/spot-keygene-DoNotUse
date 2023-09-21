#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import time
from logging import Logger

from bosdyn.api.mission import mission_pb2

from blk_arc_api.blk_arc import BLK_ARC
from .globals import ACTIONS, DOCK_ID
from .lidar.blk import connect
from .spot_client import SpotClient
from .util import Universe, countdown


def scan(lidar: BLK_ARC, spot: SpotClient, logger: Logger):
    start_time = time.time()
    duration = 30
    end_time = start_time + duration

    logger.info(f"Starting lidar scan for {duration} seconds.")

    spot.scan_pose(duration)
    scan_response = lidar.start_capture()
    scan_id = scan_response.scan_id
    attempts = 0
    while not lidar.is_scanning() and attempts < 20:
        time.sleep(0.5)
        attempts += 1
    logger.info(f"scanning... ({scan_id})")
    time.sleep(end_time - time.time())
    if not lidar.is_scanning():
        logger.error("Failed to start lidar scan.")
        return
    lidar.stop_capture()
    spot.stand()
    logger.info(f"Finished lidar scan. ({scan_id})")
    return scan_id


def upload(lidar, spot, logger):
    raise NotImplementedError


def main(mission_fiducials: set = None, logger: Logger = None):
    """
    Main function for keygene.
    """

    config = {
        "name": "spot-keygene",
        "addr": "192.168.80.3",
        "path": "./autowalks/scan.walk",
    }
    spot: SpotClient = SpotClient(config)
    lidar: BLK_ARC = connect()
    logger: Logger = logger or spot.logger

    try:
        spot.release()

        if mission_fiducials is None:
            mission_fiducials = Universe()
        logger.info("Waiting for fiducials to be visible... Move the robot to a location where fiducials are visible.")
        while not spot.get_visible_fiducials() & mission_fiducials:
            time.sleep(0.2)

        # wait for lease to become available
        spot.logger.info("Waiting for lease to become available...")
        while not spot.acquire():
            logger.info("Lease not available. Waiting...")
            time.sleep(1)
        spot.logger.info("Waiting for ESTOP to be disengaged...")
        while spot.robot.is_estopped():
            time.sleep(1)

        spot.acquire()
        countdown(5)
        spot.power_on()

        if spot.is_docked and not spot.undock():
            raise Exception("could not undock")

        spot.upload_autowalk(config["path"])
        if not spot.start_autowalk(do_localize=True):
            raise Exception("could not start autowalk")

        processed_tags = set()
        scans = set()
        while True:
            if spot.mission_status not in (mission_pb2.State.STATUS_RUNNING, mission_pb2.State.STATUS_PAUSED):
                break
            qrs = spot.get_qr_tags()
            if not qrs:
                time.sleep(0.1)
                continue

            for data, bbox in qrs:
                try:
                    tag, command = data.split(":")
                except ValueError:
                    continue

                if tag in processed_tags or command not in ACTIONS:
                    continue

                logger.info(f"tag: {tag} -- {command}")

                if not spot.pause_autowalk():
                    raise Exception("Failed to pause autowalk.")

                if command == "scan":
                    scan_id = scan(lidar, spot, logger)
                    scans.add(scan_id)
                elif command == "upload":
                    upload(lidar, spot, logger)
                elif command == "image":
                    spot.save_images("./images")

                if not spot.start_autowalk():
                    raise Exception("Failed to start autowalk.")

        if not spot.dock(DOCK_ID):
            raise Exception("could not dock")
    except Exception as e:
        logger.error(e)
    finally:
        spot.shutdown()
        lidar.disconnect()

    logger.info("exiting")
