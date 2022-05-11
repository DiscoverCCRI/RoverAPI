touch Dockerfile
echo "FROM ros:melodic" >> Dockerfile
echo "ENV ROS_DISTO=melodic" >> Dockerfile
echo "RUN mkdir -p /files/" >> Dockerfile
echo "COPY . /files/" >> Dockerfile
echo "SHELL [\"bin/bash\", \"-c\"]" >> Dockerfile
echo "RUN echo \"source /opt/ros/\$ROS_DISTRO/setup.bash\" >> ~/.bashrc" >> Dockerfile
echo "RUN echo \"source /root/catkin_ws/devel/setup.bash\" >> ~/.bashrc" >> Dockerfile
echo "RUN cd /root/ && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src && catkin_create_pkg experiment rospy roscpp std_msgs" >> Dockerfile
echo "RUN . /opt/ros/\$ROS_DISTRO/setup.bash && cd ~/catkin_ws && catkin_make" >> Dockerfile
echo "COPY rover_driver /root/catkin_ws/src/experiment/scripts" >> Dockerfile
echo "RUN cd /root/catkin_ws/src/experiment/scripts && chmod +x *" >> Dockerfile
#echo "RUN sudo apt-get update" >> Dockerfile
#echo "RUN sudo apt-get -y upgrade" >> Dockerfile
touch docker-compose.yaml
echo "version: '3'" >> docker-compose.yaml
echo "services:" >> docker-compose.yaml
echo "    client:" >> docker-compose.yaml
echo "        container_name: client" >> docker-compose.yaml
echo "        environment:" >> docker-compose.yaml
echo "         - ROS_HOSTNAME=client" >> docker-compose.yaml
echo "         - ROS_MASTER_URI=http://172.20.0.1:11311" >> docker-compose.yaml
echo "        image: ros_image:1.0" >> docker-compose.yaml
echo "        ports:" >> docker-compose.yaml
echo "         - 8083:8080" >> docker-compose.yaml
echo "        stdin_open: true" >> docker-compose.yaml
echo "        tty: true" >> docker-compose.yaml
echo "        command: /bin/bash -c \"cd /root/catkin_ws/src/experiment/scripts && ./examples.py\"" >> docker-compose.yaml
echo "        networks:" >> docker-compose.yaml
echo "            ros-network:" >> docker-compose.yaml
echo "                ipv4_address: 172.20.128.2" >> docker-compose.yaml
echo "networks:" >> docker-compose.yaml
echo "    ros-network:" >> docker-compose.yaml
echo "        ipam:" >> docker-compose.yaml
echo "            config:" >> docker-compose.yaml
echo "                - subnet: 172.20.0.0/16" >> docker-compose.yaml
mv docker-compose.yaml Dockerfile ..
cd .. && docker build -t ros_image:1.0 . && docker-compose -f docker-compose.yaml up -d


