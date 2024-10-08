# Starting with RoverAPI

**Description:** This is a guided example on how set up a development environment to use DiscoverCCRI's RoverAPI. This is the first step before to send the experiments.

**Tutorial Level:** Beginner

**Next Tutorial:** [First Script](example0.md)

**Contents:**
<ol type="1">
  <li><a href="#1">Prerequisites</a></li>
  <li><a href="#2">Setup</a></li>
  <ol type="1">
    <li><a href="#2.1">Installation</a></li>
    <li><a href="#2.2">Run</a></li>
    <li><a href="#2.3">Use</a></li>
    <li><a href="#2.4">Continuing On</a></li>
  </ol>
</ol>

### 1. Prerequisites
***Add prerequisites.txt file, specially for python***

This tutorial assumes that you are familiar with [DiscoverCCRI's project](https://discoverccri.org). If not, you should familiarize yourself with DiscoverCCRI's infrastructure and [portal](https://discover-dev.rc.nau.edu/). To correctly use the containerized development environment, you will need to have installed [docker](https://www.docker.com/) on a Linux, ***Windows (create un WSL)*** or MacOS device.

### 2. Setup
The DiscoverCCRI rover team has created a containerized simulation development environment allowing for users to easily run and test scripts that utilize our rovers. The development environment contains: [ROS](https://ros.org), the software suite that make controlling the rovers possible; [Gazebo](https://gazebosim.org/home), a 3D robotics simulator; and all required dependencies to make DiscoverCCRI's [RoverAPI](https://github.com/DiscoverCCRI/RoverAPI) work. The development environment is deployed in a docker container, and uses noVNC to allow for easy access.

#### 2.1 Installation
To install the docker container, simply run:
```
docker pull cjb873/sim_image:1.0
```

#### 2.2 Run
To run the container, use the command
```
docker run -p 9000:80 -it --name rover_development_container cjb873/sim_image:1.0
```

Then open your browser and go to `localhost:9000`. You should see the desktop of your container.

After running the container, you can use the following command to stop your container:
```
docker stop
```

After creating the container, you just have to use the following command to activate it:
```
docker start rover_development_container
```

#### 2.3 Use
To use the simulator in the container, run the following command in the terminal:
```
roslaunch leo_gazebo leo_gazebo
```
Within gazebo you can place all sorts of objects to create a scenario to test the capabilities of the rover. From here you can develop code just like you normally would in any other situation. There are some examples in [Python](./Cpp_examples/) and [C++](./Python_examples/) where you can use as a guide to develop your application.

#### 2.4 Continuing On
If you are going to be continuing on with the other tutorials, create a directory for all of your tutorial code using the command:
```
mkdir ~/beginner_tutorials
```
<p>&nbsp;</p>
