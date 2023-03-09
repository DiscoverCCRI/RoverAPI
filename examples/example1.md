# Utilizing More Features

**Description:** This is a guided example on how to write a more involved python script to use DiscoverCCRI's [RoverAPI](https://github.com/DiscoverCCRI/RoverAPI)

**Tutorial Level:** Beginner

**Previous Tutorial:** [First Script](example0.md)

**Next Tutorial:** [Utilizing More Features](example1.md)

**Contents:**
<ol type="1">
  <li><a href="#1">Prerequisites</a></li>
  <ol type="1">
    <li><a href="#1.1">The Code</a></li>
    <li><a href="#1.2">Explanation</a></li>
  </ol>
</ol>

<div id="1"></div>

### 1. Prerequisites
This tutorial expects you to have already completed the [previous tutorial](example0.md). It also expects you to have some knowledge and comfortability with
Python's [anonymous function](https://docs.python.org/3/reference/expressions.html#lambda) feature, along with knowledge of [callback functions](https://en.wikipedia.org/wiki/Callback_(computer_programming)).
<p>&nbsp;</p><p>&nbsp;</p>

<div id="2"></div>

### 2. The Code
This tutorial will build off of the [previous](example0.md) tutorial. You will utilize increased functionality within the API. 
<p>&nbsp;</p>

<div id="2.1"></div>

#### 2.2 Writing the Code
First, access your development environment, open the terminal and change into your beginner_tutorials directory:
```
cd ~/beginner_tutorials
```  
<p>&nbsp;</p>

Download the python code for this tutorial and make it an executable file:
```
wget https://github.com/DiscoverCCRI/RoverAPI/examples/more_features.py
chmod +x more_features.py
```
<p>&nbsp;</p>


You can view and edit this code with `vim more_features.py` or view the code below:
```
#!/usr/bin/env python3
# license removed for concision

from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_init import ExperimentInitializer
from rover_api.discover_utils import finish_experiment
from rover_api.discover_utils import get_time_str

def finished_cb(num_images):
    
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
init = ExperimentInitializer(lambda: finished_cb(num_images))
    
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
```
<p>&nbsp;</p>

<div id="2.2"></div>

#### 2.2 Explanation
```
#!/user/bin/env python3
```
Every script you write will contain this [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)), which lets the computer know which type of interpreter to 
use when running scripts as executables.
<p>&nbsp;</p>

```
from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_init import ExperimentInitializer
from rover_api.discover_utils import finish_experiment
from rover_api.discover_utils import get_time_str
```
For this experiment, you will need to use the capabilities of yhe camera, lidar, and movement systems, as in the previous tutorial, along with the function signaling that your experiment has ended. Additionally you will need extra functionailty to set up your experiment, and get a formatted time string.
<p>&nbsp;</p>

```
def finished_cb(lidar, num_images):
```
You will start by declaring a callback function that will be called when you signal to the system that you are finished with our experiment. You will pass the lidar object and the number of images captured during this experiment as arguments.
<p>&nbsp;</p>

```
    # stop the recording
    lidar.stop_recording()
```
Next, you will stop the recording. You will be calling the stop_recording() function within your finished function because this is the final function that will be called before the program exits, and you want your lidar to be capturing information for the duration of the program. Additionally, if the rover has some sort of problem, such as the battery becoming very low, your callback function will be called before it powers off. By placing the stop_recording() function call within this function, you ensure that your lidar data is properly saved.
<p>&nbsp;</p>

```
    # write the time at the end of the experiment at the end of the file
    # along with the total number of frames for our video
    # we can use this information to determine the fps captured by
    # our rover, which should be somewhere between 24 and 28 fps
    with open("/experiment/information.txt", "a", "utf-8") as outfile:
        outfile.write(get_time_str())
        outfile.write(f"Num of frames: {num_image[0]}")
```
After stopping the lidar, you will want to write the total number of images collected by the rover during the experiment to a file, along with the current time. With this information, you can determine the length of the experiment. So, if you wanted to turn the series of images captured during this experiment into a video, you can determine the number of frames captured per second.
<p>&nbsp;</p>

```
def take_video(cam, num_images):
```
Now, you will define the camera's callback function. This function will be called whenever the camera produces new image data. The camera object and the current number of images will be passed as arguments.
<p>&nbsp;</p>

```
    # save the jpg of the image taken, update the number of images
    cam.get_jpg()
    num_images[0] += 1
```
Within your camera callback function, you will save the jpg of the new image produced by the camera, and increment the total number of images. Notice that you will treat the number of images as an item within a list, as the reason for this will be explained later.
<p>&nbsp;</p>


```
# initialize the objects to control the hardware
rover = Rover()
lidar = Lidar(callback=None, convert=True, subscribe=False)
cam = Camera()
```
Now, you will initialize the objects allowing you to control the hardware. You will initialize your lidar to have no callback function (notice we only declared callbacks for finishing the experiment and the camera), set convert to true, and subscribe to false. Setting convert to true will store all of our lidar data as [.pcd](https://pointclouds.org/documentation/tutorials/pcd_file_format.html) files, which are more commonly used to store data than the rosbag you used in the [previous tutorial](example0.md). Setting the subscribe parameter to false will ensure that the lidar is only saving data for a certain period of time.
<p>&nbsp;</p>

```
# declare number of images as a list so that the value can be passed back
# after the anonymous function has been called
num_images = [0]
```
Now, you will initialize the number of images variable. You will want to initialize this as an item in a list, as this will make sure that when you pass the item as a parameter in an anonymous function, that function can change the value.
<p>&nbsp;</p>

```
# initialize the experiment with a function that will be called when it
# is over
# we must instiantiate an initializer object before we make any calls to
# other objects
init = ExperimentInitializer(lambda: finished_cb(lidar, num_images))
```
Next, you will initialize the experiment by creating an ExperimentInitializer object. This object will set the callback function that you previously declared as finished_cb(), so that it will be called when the experiment is over, or when the rover has a critical error.
<p>&nbsp;</p>

```
# set the callback - this means that everytime the rover's camera has a 
# new image, this function will be called
# it is initialized as a lambda so values can be changed within the 
# function
cam.set_callback(lambda: take_video(cam, num_images))
```
Now, you will set the callback function for the camera, enabling it call this function whenever there is a new image from the camera. You will use an anonymous function to pass the callback function as it lets you pass parameters along with the function.
<p>&nbsp;</p>


