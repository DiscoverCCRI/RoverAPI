#!/usr/bin/env bash

cd ~/catkin_ws && catkin_make
eval "$(cat ~/.bashrc| tail -n +10)"
