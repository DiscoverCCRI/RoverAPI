#!/usr/bin/bash

cd ~/catkin_ws && catkin_make && catkin_make && catkin_make
eval "$(cat ~/.bashrc| tail -n +10)"
