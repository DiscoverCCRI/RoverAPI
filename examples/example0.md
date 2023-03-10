# Rover API First Script

**Description:** This is a guided example on how to write a simple python script to use DiscoverCCRI's RoverAPI

**Tutorial Level:** Beginner

**Previous Tutorial:** [Getting Started](starting.md)

**Next Tutorial:** [Utilizing More Features](example1.md)

**Contents:**
<ol type="1">
  <li><a href="#1">Prerequisites</a></li>
  <li><a href="#2">The Code</a></li>
  <ol type="1">
    <li><a href="#2.1">Writing the Code</a></li>
    <li><a href="#2.2">Explanation</a></li>
  </ol>
</ol>



<p>&nbsp;</p><p>&nbsp;</p>


<div id="1"></div>

### 1. Prerequisites

This code will make use of DiscoverCCRI's [RoverAPI](https://github.com/DiscoverCCRI/RoverAPI), which is designed to make operation of DiscoverCCRI's 
rovers easy. If you are unfamiliar with the Discover project, please check out our [website](https://discoverccri.org) to learn more. The tutorial 
assumes that you have already set up a development based on the [previous tutorial](examples/starting.md).
<p>&nbsp;</p><p>&nbsp;</p>


<div id="2"></div>

### 2. The Code
For this tutorial, we will write a simple python script that will make use of the rover's movement, camera, and lidar capabilities. We will direct the rover to drive alone a fixed path, take photos at regular intervals, and record laser scan data from the lidar.

<p>&nbsp;</p>
<p>&nbsp;</p>


<div id="2.1"></div>

#### 2.1 Writing the Code
First, access your development environment using the instructions from the [previous tutorial](examples/starting.md), open the terminal and 
change into the beginner_tutorials directory you created:
```
cd ~/beginner_tutorials
```  
<p>&nbsp;</p>

Download the python code for this tutorial and make it an executable file:
```
wget https://raw.githubusercontent.com/DiscoverCCRI/RoverAPI/main/examples/first_script.py
chmod +x first_script.py
```
<p>&nbsp;</p>

You can view and edit this code with `vim first_script.py` or view the code below:
```
#!/usr/bin/env python3
# license removed for concision

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
```
<p>&nbsp;</p>
<p>&nbsp;</p>

<div id="2.2"></div>

#### 2.2 Explanation

```
#!/user/bin/env python3
```
Every script we write will contain this [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)), which lets the computer know which type of interpreter to 
use when running scripts as executables.
<p>&nbsp;</p>

```
from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_utils import finish_experiment
```
For this experiment, we will need to access the rover's lidar, camera, and movement capabilities. Additionally, we will need to use the finish_experiment() 
function to let the system know that our experiment is done.
<p>&nbsp;</p>

```
# initialize the objects to control the hardware
rover = Rover()
lidar = Lidar()
cam = Camera()
```
First, we will instantiate the objects to actually access and control the rover's hardware.
<p>&nbsp;</p>

```
# take a rosbag recording of the lidar data
lidar.start_recording()
```
Next, we will start recording the data from the lidar. The data will be saved to a [rosbag](http://wiki.ros.org/rosbag). If you are unfamiliar with [ROS](https://ros.org),
that is alright. Nothing in this API requires you to know ROS, and in the future, we will save the lidar data to a more common format.
<p>&nbsp;</p>

```
# drive in a 1 meter square
for i in range(4):
    # drive forward at a rate of 0.2 m/s for 5s
    rover.move_forward(0.2, 5)
    
    # save an image of what the rover sees
    cam.get_jpg()

    # turn right at a rate of 15 deg/s for 6s
    rover.turn_right(15, 6) 
```
We will tell the rover to drive forward for 5 seconds at a rate of 0.2 meters per second. This will move the rover forward 1 meter. 
Then we tell the rover to save a .jpg image of what the camera is seeing. These images will automatically be saved to the `/experiment` directory. 
After taking the photo, the rover will turn right at a rate of 15 degrees for 6 seconds, making a complete 90 degree turn. 
Finally, we will tell the rover to repeat this series of steps 4 times, driving in a 1 meter by 1 meter square.
<p>&nbsp;</p>

```
# stop the recording
lidar.stop_recording()
```
Next, we will direct the lidar to stop recording. This will save the rosbag file to the `/experiment` directory.
<p>&nbsp;</p>

```
# finish the experiment
finish_experiment()
```
Finally, we will tell the system that you are finished running your experiment. This allows the system to shut down the docker container that our 
code is running in, and saves the `/experiment` container with all of our data.
<p>&nbsp;</p>
