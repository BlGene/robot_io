Using robots should be as simple as importing a python package and being able to start programming. This repository tries to enable this, in conjunction with providing code for human input devices, camera calibration, and sequence recording. This repository provides code to control the UR, Franka Emika Panda, and Kuka IIWA robots of AIS, University of Freiburg. Maintainers are Lukas Hermann, Max Argus, Iman Nematollahi, Oier Mees, and Adrian Röfer.

# [Setup](docs/setup.md)

---------------
# Usage

Examples of how to use `robot_io` can be found here: `robot_io/examples/move_robot.py`.

This repository make use of hydra for configuration management, configuration files are located in the `./conf` directory and specify which robot, input device and cameras to use. These files also contain information on the workspace limits e.g. `./conf/workspace/box.yaml` please update these to suit your setup.

## Camera Calibration

[see here.](docs/usage.md)

## Input Devices

Input devices can be found here: `./robot_io/input_devices`.  

### VR

The VR input device is more complex, its installation and usage is described [here.](docs/teleoperation.md)

---------------
