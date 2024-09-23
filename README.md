# RoverAPI

This is an API to use Discover's LeoRovers using the stable version (Currently is ROS Noetic). Once [LEO ROVER](https://www.leorover.tech/guides/ros-2-support-experimental) finish their experimental part with ROS 2 and release a stable version the experimentes for the RoverAPI will be updated.

### Installation

First, make sure that you have [ROS Noetic installed](http://wiki.ros.org/noetic/Installation/Ubuntu). 
All LeoRovers should have ROS installed as part of the OS. Next, make sure that you have [set up a catkin workspace](https://subscription.packtpub.com/book/iot-&-hardware/9781782175193/1/ch01lvl1sec11/creating-a-catkin-workspace). 
Then, clone this repository into `~/catkin_ws/src`, and use 
```
mv ~/catkin_ws/src/RoverAPI/rover_api ~/catkin_ws/src
```

and 

```
rm -r ~/catkin_ws/src/RoverAPI
``` 

to pull the ROS package out of the repository and delete unneeded parts of the repository. 
Next, initialize the [rosdep tool and use it to install all dependencies](http://wiki.ros.org/rosdep). Finally, 
use 

```
cd ~/catkin_ws && catkin_make
``` 

to build RoverAPI.

### Warning

When running the RoverAPI on the LeoRovers, the default pi user may not have all of the necessary permissions to do certain operations,
such as creating the `/experiment` directory. So, all code should be run using the sudo command, or as the root user. If running as the root user, do not forget to run: 

```
source /opt/ros/setup.bash && source /home/pi/catkin_ws/devel/setup.bash
```
