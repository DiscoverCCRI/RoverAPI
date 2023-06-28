# RoverAPI

This is an API to use Discover's LeoRovers without the need to learn ROS.

### Installation

First, make sure that you have [ROS Noetic installed](http://wiki.ros.org/noetic/Installation/Ubuntu). 
All LeoRovers should have ROS installed as part of the OS. Next, make sure that you have [set up a catkin workspace](https://subscription.packtpub.com/book/iot-&-hardware/9781782175193/1/ch01lvl1sec11/creating-a-catkin-workspace). 
Then, clone this repository into `~/catkin_ws/src`, and use 
```mv ~/catkin_ws/src/RoverAPI/rover_api ~/catkin_ws/src``` 
and 
```rm -r ~/catkin_ws/src/RoverAPI``` 
to pull the ROS package out of the repository and delete unneeded parts of the repository. 
Next, initialize the [rosdep tool and use it to install all dependencies](http://wiki.ros.org/rosdep). Finally, 
use 
```cd ~/catkin_ws && catkin_make``` 
to build RoverAPI.
