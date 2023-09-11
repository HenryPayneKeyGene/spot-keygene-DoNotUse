#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT
import os

from .run import Run

SPOT_IP = '192.168.80.3'


def main():
    config = {
        "spot": {
            "addr": SPOT_IP,
            "name": "spot-keygene",
            "download_path": os.getcwd(),
            "upload_path": os.getcwd()
        }
    }
    run = Run(config)
    run.autowalk()
