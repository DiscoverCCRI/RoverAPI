#!/usr/bin/bash

# First change to the superuser with the following command: sudo su
sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z"
sudo timedatectl set-timezone America/Phoenix
sudo apt-get update && sudo apt-get upgrade
. /opt/ros/$ROS_DISTRO/setup.bash && rosrun leo_fw update
curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh \
   && rm get-docker.sh
cd /home/pi && git clone https://github.com/DiscoverCCRI/leorover-test-image.git
cd /home/pi/leorover-test-image && docker build --network=host -t cjb873/leorover_image:1.0 .
sudo echo "172.18.0.2 client" >> /etc/hosts
cd /home/pi && git clone https://github.com/DiscoverCCRI/RoverAPI \
    && git clone https://github.com/Slamtec/rplidar_ros.git \
    && mkdir -p catkin_ws/src && mv RoverAPI/rover_api catkin_ws/src \
    && cd catkin_ws && . /opt/ros/noetic/setup.bash && catkin_make
echo "source /opt/ros/noetic/setup.bash" >> /home/pi/.bashrc \
    && echo "source ~/catkin_ws/devel/setup.bash" >> /home/pi/.bashrc \
    && mv /home/pi/RoverAPI/scripts/example.py /home/pi \
    && chmod u+x /home/pi/example.py \
    && sudo rm -r /home/pi/RoverAPI
sudo apt-get -y install cron
sudo usermod -aG docker pi 
sudo useradd -m experimenter
sudo chsh -s /bin/rbash experimenter
sudo usermod -aG docker experimenter
sudo cp /home/pi/.bashrc /home/experimenter/.bashrc
sudo echo "docker exec -it client /bin/bash" >> /home/experimenter/.bashrc 
# Finally run the following command: crontab -e
# Copy the following text into the crontab:
# @reboot sleep 60 && sudo date -s "$(wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z" && sudo chmod 666 /dev/ttyUSB*
