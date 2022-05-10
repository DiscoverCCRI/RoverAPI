touch Dockerfile
echo "FROM ros:noetic" >> Dockerfile
echo "ENV ROS_DISTO=noetic" >> Dockerfile
echo "RUN mkdir -p /files/" >> Dockerfile
echo "COPY . /files/" >> Dockerfile
echo "SHELL [\"bin/bash\", \"-c\"]" >> Dockerfile
echo "RUN echo \"source /opt/ros/\$ROS_DISTRO/setup.bash\" >> ~/.bashrc" >> Dockerfile
echo "RUN echo \"source /root/catkin_ws/devel/setup.bash\" >> ~/.bashrc" >> Dockerfile
echo "RUN cd /root/ && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src && catkin_create_pkg ros_test rospy roscpp std_msgs" >> Dockerfile
echo "RUN . /opt/ros/\$ROS_DISTRO/setup.bash && cd ~/catkin_ws && catkin_make" >> Dockerfile
echo "COPY scripts /root/catkin_ws/src/ros_test/scripts" >> Dockerfile
echo "RUN cd /root/catkin_ws/src/ros_test/scripts && chmod +x *" >> Dockerfile
echo "RUN sudo apt-get update" >> Dockerfile
echo "RUN sudo apt-get -y upgrade" >> Dockerfile

