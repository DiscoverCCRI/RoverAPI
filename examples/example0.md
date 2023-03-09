# Rover API Example

**Description:** This is a guided example on how to write a simple python script to use DiscoverCCRI's RoverAPI

**Tutorial Level:** Beginner

**Previous Tutorial:** [Getting Started](examples/starting.md)

**Next Tutorial:** [Utilizing More Features](examples/example1.md)

**Contents:**
<ol type="1">
  <li><a href="https://github.com/DiscoverCCRI/RoverAPI/examples#1-writing-the-code">Writing the Code</a></li>
  <ol type="1">
    <li><a href="https://github.com/DiscoverCCRI/RoverAPI/examples#11-the-code">The Code</a></li>
    <li><a href="https://github.com/DiscoverCCRI/RoverAPI/examples#12-explanation">Explanation</a></li>
  </ol>
</ol>




### 1. Writing the Code

This code will make use of DiscoverCCRI's [RoverAPI](https://github.com/DiscoverCCRI/RoverAPI), which is designed to make operation of DiscoverCCRI's 
rovers easy. If you are unfamiliar with the Discover project, please check out our [website](https://discoverccri.org) to learn more. The tutorial 
assumes that you have already set up a development based on the [previous tutorial](examples/starting.md).

#### 1.1 The Code
First, access your development environment using the instructions from the [previous tutorial](examples/starting.md), open the terminal and 
change into the beginner_tutorials directory you created:
```
cd ~/beginner_tutorials
```

Download the python code for this tutorial and make it an executable file:
```
wget https://github.com/DiscoverCCRI/RoverAPI/examples/first_script.py
chmod +x first_script.py
```

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

#### 1.2 Explanation

```
#!/user/bin/env python3
```
Every script you write will contain this [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)), which lets the computer know which type of interpreter to 
use when running scripts as executables.

```
from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_utils import finish_experiment
```
For this experiment, you will need to access the rover's lidar, camera, and movement capabilities. Additionally, you will need to use the finish_experiment() 
function to let the system know that your experiment is done.

```
# initialize the objects to control the hardware
rover = Rover()
lidar = Lidar()
cam = Camera()
```
First, you will create the objects to actually access and control the rover's hardware.

```
# take a rosbag recording of the lidar data
lidar.start_recording()
```
Next, you will start recording the data from the lidar. The data will be saved to a [rosbag](http://wiki.ros.org/rosbag). If you are unfamiliar with ROS,
that is alright. In the future, we will save the lidar data to a more common format.

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
You will tell the rover to drive forward for 5 seconds at a rate of 0.2 meters per second. This will move the rover forward 1 meter. 
Then you will tell the rover to save a .jpg image of what the camera is seeing. These images will automatically be saved to the `/experiment` directory. 
After taking the photo, the rover will turn right at a rate of 15 degrees for 6 seconds, making a complete 90 degree turn. 
You will tell the rover to repeat this series of steps 4 times, driving in a 1 meter by 1 meter square.

```
# stop the recording
lidar.stop_recording()
```
Next, you will tell the lidar to stop recording. This will save the rosbag file to the `/experiment` directory.

```
# finish the experiment - this will automatically call our finished_cb func
finish_experiment()
```
Finally, you will tell the system that you are finished running your experiment. This allows the system to shut down the docker container that your 
code is running in, and saves the `/experiment` container with all of your data.









