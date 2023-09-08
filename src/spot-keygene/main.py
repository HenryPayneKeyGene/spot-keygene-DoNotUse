#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

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
