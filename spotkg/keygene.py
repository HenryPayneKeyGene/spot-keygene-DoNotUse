#  Copyright (c) Romir Kulshrestha 2023.

import time
from logging import Logger
from typing import List, Set

from bosdyn.api.mission import mission_pb2

from blk_arc_api.blk_arc import BLK_ARC
from .globals import ACTIONS, DOCK_ID
from .lidar.blk import connect
from .spot_client import SpotClient
from .util import countdown


def __scan(lidar: BLK_ARC, spot: SpotClient, logger: Logger):
    """Perform a lidar scan."""
    start_time = time.time()
    duration = 30
    end_time = start_time + duration

    logger.info(f"Starting lidar scan for {duration} seconds.")

    spot.scan_pose(duration)
    scan_response = lidar.start_capture()
    scan_id: int = scan_response.scan_id
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


def __upload(_lidar, _spot, _logger):
    raise NotImplementedError


def kg_run(
    mission_name: str,
    spot: SpotClient,
    lidar: BLK_ARC,
    logger: Logger = None,
    mission_fiducials: set = None,
):
    """
    Main function for keygene.
    """

    config = {
        "name": "spotkg",
        "addr": "192.168.80.3",
        "path": f"./autowalks/{mission_name}.walk",
    }

    logger: Logger = logger or spot.logger

    try:
        spot.release()

        # mission_fiducials = Universe()
        logger.info(
            "Waiting for fiducials to be visible... Move the robot to a location where fiducials are visible."
        )
        if mission_fiducials is None:
            while not spot.get_visible_fiducials():
                time.sleep(0.2)

        # wait for lease to become available
        spot.logger.info("Waiting for lease to become available...")
        while not spot.acquire():
            logger.info("Lease not available. Waiting...")
            time.sleep(1)
        spot.logger.info("Waiting for ESTOP to be disengaged...")
        while spot.robot.is_estopped():
            time.sleep(1)

        countdown(5)
        spot.power_on()

        try:
            spot.undock()
        except Exception as e:
            logger.warning(e)

        spot.upload_autowalk(config["path"])
        if not spot.start_autowalk(do_localize=True):
            raise Exception("could not start autowalk")

        processed_tags = set()
        scans: Set[int] = set()
        while True:
            if spot.mission_status not in (
                mission_pb2.State.STATUS_RUNNING,
                mission_pb2.State.STATUS_PAUSED,
            ):
                break

            qrs = spot.get_qr_tags()
            if not qrs:
                time.sleep(0.1)
                continue

            for data, _ in qrs:
                try:
                    tag, command = data.split(":")
                except ValueError:
                    continue

                tag: int = int(tag)
                command: str = command.lower()
                if tag in processed_tags or command not in ACTIONS:
                    continue

                processed_tags.add(tag)

                logger.info(f"tag: {tag} -- {command}")

                if not spot.pause_autowalk():
                    raise Exception("Failed to pause autowalk.")

                if command == "scan":
                    scan_id = __scan(lidar, spot, logger)
                    if scan_id:
                        scans.add(scan_id)
                    else:
                        logger.error("Failed to scan.")
                elif command == "upload":
                    __upload(lidar, spot, logger)
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

    logger.info("exiting")


def run_many(missions: List[str]):
    """
    Run multiple missions.
    """

    spot: SpotClient = SpotClient()
    lidar: BLK_ARC = connect()
    for mission in missions:
        kg_run(mission, spot, lidar)
