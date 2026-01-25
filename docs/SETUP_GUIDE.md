# 📖 Speech Interface - Full Recovery Guide

**Ziel:** Wiederherstellung des Systems auf einem frischen Raspberry Pi / Ubuntu PC.

## 1. ROS Installation (Falls noch nicht vorhanden)
```bash
# Quellen & Key
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installieren
sudo apt update
sudo apt install -y ros-noetic-ros-base python3-pip python3-rosdep alsa-utils git

# Init
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
