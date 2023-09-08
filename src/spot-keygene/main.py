from time import sleep

from .blk import BLK
from .spot import Spot

SPOT_IP = '192.168.80.3'


def main():
    spot = Spot(SPOT_IP, "spot")
    blk = BLK()
    # spot.power_on()
    # spot.stand()
    # sleep(3)
    # spot.sit()
    # blk.start_capture()
    spot.shutdown()
