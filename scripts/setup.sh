#!/bin/bash

# Copyright (c) Romir Kulshrestha 2023.

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
    pip uninstall spot-keygene -y
    pip install ./dist/*.whl
popd || exit
