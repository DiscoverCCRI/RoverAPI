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

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_init import ExperimentInitializer
from rover_api.discover_utils import finish_experiment
from rover_api.discover_utils import get_time_str

def finished_cb(lidar, num_images):
    
    # we call the lidar functions necessary to save the data within the 
    # finished function so that if, for some reason, the experiment is 
    # prematurely ended, the lidar still saves all of its data
    
    # stop the recording
    lidar.stop_recording()
    
    # write the time at the end of the experiment at the end of the file
    # along with the total number of frames for our video
    # we can use this information to determine the fps captured by
    # our rover, which should be somewhere between 24 and 28 fps
    with open("/experiment/information.txt", "a", "utf-8") as outfile:
        outfile.write(get_time_str())
        outfile.write(f"Num of frames: {num_image[0]}")

def take_video(cam, num_images):
    
    # save the jpg of the image taken, update the number of images
    cam.get_jpg()
    num_images[0] += 1


# initialize the objects to control the hardware
rover = Rover()
lidar = Lidar(callback=None, convert=True, subscribe=False)
cam = Camera()
    
# declare number of images as a list so that the value can be passed back
# after the anonymous function has been called
num_images = [0]
    
# initialize the experiment with a function that will be called when it
# is over
# we must instiantiate an initializer object before we make any calls to
# other objects
init = ExperimentInitializer(lambda: finished_cb(lidar, num_images))
    
# set the callback - this means that everytime the rover's camera has a 
# new image, this function will be called
# it is initialized as a lambda so values can be changed within the 
# function
cam.set_callback(lambda: take_video(cam, num_images))


# record the current time to a file
with open("/experiment/information.txt", "w", "utf-8") as outfile:
    outfile.write(get_time_str())

# start a recording of the lidar data
lidar.subscribe_to_scan()
lidar.start_recording()

# drive in a 1 meter square
for i in range(4):
    # drive forward at a rate of 0.2 m/s for 5s
    rover.move_forward(0.2, 5)
        
    # save an image of what the rover sees
    cam.get_jpg()

    # turn right at a rate of 15 deg/s for 6s
    rover.turn_right(15, 6) 

# finish the experiment - this will automatically call our finished_cb func
finish_experiment()
