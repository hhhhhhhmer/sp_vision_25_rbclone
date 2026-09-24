#!/bin/bash

# # ros2 build
# cd sp_ws
# colcon build --packages-select sp_msgs
# source ./install/setup.bash
# cd ..


cmake -B build
# make -C build/ -j12
# make -C build/ -j`nproc`
# exec "$@"
make -C build/ -j10
