#!/bin/bash

# Copyright (c) Romir Kulshrestha 2023.

BASE_DIR="../$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source /home/spot/.pyenv/versions/kg/bin/activate

pushd "$BASE_DIR" || exit

    # ensure packages are installed
    ./setup.sh

    echo -e "\033[1;32mStarting lidar service...\033[0m"

    # unset proxy
    http_proxy=
    https_proxy=

    # start lidar service
    python -m spot-keygene lidar --host-ip 192.168.50.5 192.168.80.3
popd || exit
