#  Copyright (c) Romir Kulshrestha 2023.
#  You may use, distribute and modify this code under the terms of the MIT License.
#  You should have received a copy of the MIT License with this file. If not, please visit:
#  https://opensource.org/licenses/MIT

import argparse
import sys

from bosdyn.client.util import add_base_arguments, add_service_endpoint_arguments

parser = argparse.ArgumentParser(description='Spot Keygene')
parser.add_argument('--version', action='version', version='%(prog)s 0.1.0')

subparsers = parser.add_subparsers(dest='service')

lidar_parser = subparsers.add_parser('lidar', help='Run the lidar service.')
add_base_arguments(lidar_parser)
add_service_endpoint_arguments(lidar_parser)

record_parser = subparsers.add_parser('record', help='Run the recording service.')
add_base_arguments(record_parser)
record_parser.add_argument('--output', help='Output directory for the recording.', default='.')

qr_parser = subparsers.add_parser('qr', help='Generate QR codes.')
qr_parser.add_argument('--output', help='Output directory for the QR codes.', default='./tags')
qr_parser.add_argument('--zip', help='Generate a zip file of the QR codes.', action='store_true')
qr_parser.add_argument("--set",
                       metavar="KEY=VALUE",
                       nargs='+',
                       help="Set a number of id-command pairs "
                            "(do not put spaces before or after the = sign). "
                            "If a value contains spaces, you should define "
                            "it with double quotes: "
                            'foo="this is a sentence". Note that '
                            "values are always treated as strings.")

default_parser = subparsers.add_parser('<blank>', help='Run the default service.')

options = parser.parse_args()

if options.service == 'lidar':
    print("Starting LiDAR service...")
    from .lidar_service import start_lidar

    start_lidar(options)
elif options.service == 'record':
    print("Starting recording service...")
    from .recording import start_recording

    start_recording(options)
elif options.service == 'qr':
    print("Generating QR codes...")
    from .gen_tags import gen_tags

    d = {}
    if options.set:
        for item in options.set:
            items = item.split('=')
            key = items[0].strip()  # we remove blanks around keys, as is logical
            if len(items) < 2:
                raise argparse.ArgumentTypeError('You must provide a value for each key.')
            value = '='.join(items[1:])
            if value == "":
                raise argparse.ArgumentTypeError('You must provide a value for each key.')
            # noinspection PyUnboundLocalVariable
            d[key] = value

    gen_tags(options.output, d, options.zip)
    sys.exit(0)
else:
    print("Starting...")
    from .keygene import keygene_main

    keygene_main()
