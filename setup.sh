#!/bin/bash
# Copyright (c) Romir Kulshrestha 2023.
# You may use, distribute and modify this code under the terms of the MIT License.
# You should have received a copy of the MIT License with this file. If not, please visit:
# https://opensource.org/licenses/MIT

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
BASE_DIR=$( dirname "${SCRIPT_DIR}" )

source /home/spot/.pyenv/versions/kg/bin/activate

# install requirements
pushd "$BASE_DIR" || exit
    git checkout main
    git pull origin main
    pip install -r requirements.txt
popd || exit

# Install the BLKARC-Module-API_TuDelft
pushd "$BASE_DIR"/BLKARC-Module-API_TuDelft/src/blk_arc_api || exit
    ./install.sh
popd || exit

