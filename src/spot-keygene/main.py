from time import sleep

from blk_arc_api.blk_arc import BLK_ARC
from blk_arc_api.blk_arc_config import ConnectionType
from .spot import Spot

SPOT_IP = '192.168.80.3'
CONNECTION_TYPE = ConnectionType.WIRED


def main():
    spot = Spot(SPOT_IP, "spot")
    # blk_arc = BLK_ARC()
    # # Connect:
    # if not blk_arc.connect(CONNECTION_TYPE):
    #     spot.robot.logger.error("could not connect to LIDAR")
    #     return
    spot.power_on()
    spot.stand()
    sleep(3)
    spot.sit()
    spot.shutdown()
