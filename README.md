# XBOX Joystick Setup

## Enable xbox controller input:
sudo apt-get install ros-kinetic-teleop-twist-joy

## Test xbox controller:
ls /dev/input/

sudo jstest /dev/input/jsX <-- X is typically 0

sudo chmod a+rw /dev/input/jsX
