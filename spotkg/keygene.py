#  Copyright (c) Romir Kulshrestha 2023.
import os.path
import threading
import time
from datetime import datetime
from logging import Logger
from queue import PriorityQueue
from typing import List, Set

from bosdyn.api.mission import mission_pb2
from nicegui import ui

from blk_arc_api.blk_arc import BLK_ARC
from .globals import ACTIONS, DOCK_ID
from .lidar.blk import connect
from .spot_client import SpotClient
from .util import countdown


# from .local_file_picker import local_file_picker

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


def run_one(
        mission: str,
        spot: SpotClient,
        lidar: BLK_ARC,
        logger: Logger = None,
        mission_fiducials: set = None,
):
    """
    Main function for keygene.
    """

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

        spot.upload_autowalk(mission)
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

    spot: SpotClient = SpotClient(config={
        "name": "spotkg",
        "addr": "192.168.80.3",

    })
    try:
        lidar: BLK_ARC = connect()
    except:
        lidar = None
    for mission in missions:
        run_one(f"./autowalks/{mission}.walk", spot, lidar)


execution_order: PriorityQueue = PriorityQueue()
execution_order.put((int(datetime.utcnow().timestamp()), ["tst"]))
running = False


def __run_queue():
    while running:
        if execution_order.empty():
            time.sleep(1)
            print("waiting...")
            continue
        schedule, mission = execution_order.get()
        print(f"NXT: {mission} @ {datetime.utcfromtimestamp(schedule)}")
        while schedule > datetime.utcnow().timestamp():
            time.sleep(0.1)
        try:
            run_many(mission)
        except Exception as e:
            print(e)


def queue_runner():
    global running
    running = True
    thread = threading.Thread(target=__run_queue, daemon=True)
    thread.start()


@ui.refreshable
def exec_table():
    cols = [{'name': 'Time', 'label': 'Time', 'field': 'Time', 'sortable': True},
            {'name': 'Mission', 'label': 'Mission', 'field': 'Mission', 'sortable': False}]
    rows = [{'Time': datetime.utcfromtimestamp(t), 'Mission': ",".join(m for m in mission)} for t, mission in
            execution_order.queue]
    ui.table(cols, rows, row_key='Time')


def add_to_queue(mission: str, schedule: str):
    now = datetime.today()
    t = int(datetime.strptime(schedule, '%H:%M').replace(year=now.year, month=now.month, day=now.day).timestamp())
    i = (t, mission.split(","))
    print(i)
    execution_order.put(i)
    exec_table.refresh()


schedule = datetime.now()


# async def pick_file() -> None:
#     result = await local_file_picker('~', multiple=True)
#     ui.notify(f'You chose {result}')


def set_schedule(e):
    global schedule
    schedule = e.value


def gui():
    # ui.title("Keygene")
    exec_table()
    time_picker = ui.time(value='12:00', on_change=set_schedule)

    # ui.button('Choose file', on_click=pick_file, i/con='folder')
    file_picker = ui.input("Mission", placeholder="Choose file", value="", validation=lambda v: os.path.isdir(
        f"./autowalks/{v}.walk"))

    ui.button("Add to queue", on_click=lambda: add_to_queue(file_picker.value, time_picker.value), icon="plus")
    queue_runner()
    ui.run()
