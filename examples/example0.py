#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023 DiscoverCCRI

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_utils import finish_experiment


# initialize the objects to control the hardware
rover = Rover()
lidar = Lidar()
cam = Camera()


# take a rosbag recording of the lidar data
lidar.start_recording()

# drive in a 1 meter square
for i in range(4):
    # drive forward at a rate of 0.2 m/s for 5s
    rover.move_forward(0.2, 5)
    
    # save an image of what the rover sees
    cam.get_jpg()

    # turn right at a rate of 15 deg/s for 6s
    rover.turn_right(15, 6) 


# stop the recording
lidar.stop_recording()

# finish the experiment - this will automatically call our finished_cb func
finish_experiment()
