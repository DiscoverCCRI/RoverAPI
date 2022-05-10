export ROS_DISTRO = noetic

echo echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd ~/ && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src && catkin_create_pkg experiment roscpp rospy std_msgs && cd ~/catkin_ws/src/experiment && mkdir scripts
cp ~/scripts/* ~/catkin_ws/src/experiment/scripts
cd ~/catkin_ws && catkin_make

