Spot Keygene
============

Autonomous greenhouse scanning and image processing for plant phenotyping, with
Spot.  
Built by Romir Kulshrestha for YES!Delft for the Keygene project, 2023.

## Installation

1. Clone this repository and `cd` into it.
2. use pyenv to create a virtual environment with python 3.7 called `kg`.  
   (see `./scripts/setup.sh` for more details, will be automated later)
3. run `./scripts/setup.sh` to install dependencies.

> Note that if the current user is `spot`, this copies a systemd service to
> `/etc/systemd/system/lidar.service`

## Usage

### Recording

1. `cd` into the repository.
2. activate the virtual environment with `pyenv activate kg`.
3. run `python -m spot-keygene record <SPOT_IP>` to record a scan.
4. Use the GUI to control the robot and record a mission, and save it to a
   chosen location.

### Scanning

1. `cd` into the repository.
2. activate the virtual environment with `pyenv activate kg`.
3. run `python -m spot-keygene --walk <MISSION_DIR>` to scan a mission using a
   walking
   pattern.

### Actions

1. `cd` into the repository.
2. activate the virtual environment with `pyenv activate kg`.
3. run `python -m spot-keygene qr` to generate a set of QR codes for the
   mission. See the
   `--help` for more options.