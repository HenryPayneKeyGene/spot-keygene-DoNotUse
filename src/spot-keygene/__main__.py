#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import argparse
import sys

from bosdyn.client.util import add_base_arguments, add_service_endpoint_arguments

from .lidar_service import start
from .main import main

parser = argparse.ArgumentParser(description='Spot Keygene')
parser.add_argument('--version', action='version', version='%(prog)s 0.1.0')

subparsers = parser.add_subparsers(dest='service')
lidar_parser = subparsers.add_parser('lidar', help='Run the lidar service.')
add_base_arguments(lidar_parser)
add_service_endpoint_arguments(lidar_parser)

options = parser.parse_args()

if options.service == 'lidar':
    start(options)
    sys.exit(0)

main()
