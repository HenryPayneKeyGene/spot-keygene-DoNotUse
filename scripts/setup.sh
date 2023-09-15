#!/bin/bash

# Copyright (c) Romir Kulshrestha 2023.
# You may use, distribute and modify this code under the terms of the MIT License.
# You should have received a copy of the MIT License with this file. If not, please visit:
# https://opensource.org/licenses/MIT

BASE_DIR=$(dirname "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )")

source /etc/environment
source /home/spot/.pyenv/versions/kg/bin/activate

# get updates and install requirements
pushd "$BASE_DIR" || exit
    git checkout main
    git pull origin main
    python -m pip install -U pip
    pip install -r requirements.txt
popd || exit

# Install the BLKARC-Module-API_TuDelft
pushd "$BASE_DIR"/BLKARC-Module-API_TuDelft/src/blk_arc_api || exit
    git checkout -f
    git clean -dxf
    ./install.sh
popd || exit

# install spot-keygene
pushd "$BASE_DIR" || exit
    rm -rf ./dist/*
    python -m build
    pip install ./dist/spot_keygene-*-py3-none-any.whl --force-reinstall

popd

if [ $(whoami) = "spot" ]
then
    # copy service files to systemd
    sudo cp "$BASE_DIR"/scripts/lidar.service /etc/systemd/system/lidar.service
    sudo systemctl daemon-reload
else
    echo "run as spot to add to systemd"
fi

