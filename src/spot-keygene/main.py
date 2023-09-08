from time import sleep

from .blk import BLK
from .spot import Spot

SPOT_IP = '192.168.80.3'


def main():
    spot = Spot(SPOT_IP, "spot")
    blk = BLK()
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
