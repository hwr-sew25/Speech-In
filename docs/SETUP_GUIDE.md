<<<<<<< HEAD
# 📖 Speech Interface - Full Recovery Guide

**Ziel:** Wiederherstellung des Systems auf einem frischen Raspberry Pi / Ubuntu PC.
=======
# Speech Interface Implementation Guide

**Author:** Team Speech-In  
**Component:** Offline Speech Recognition & Command Parsing  
**Repository:** https://github.com/hwr-sew25/Speech-In

---

## 1. Overview & Achievements
This guide explains how to replicate the **offline speech recognition system** built for the SEW25 project.
The module enables a robot to understand spoken navigation commands (e.g., "Drive to the Kitchen") without an internet connection.

**Key Features:**
*    **Offline Speech-to-Text:** Powered by Vosk (Neural Network).
*    **Command Parsing:** Converts spoken sentences into structured JSON commands.
*    **Dynamic Room Mapping:** Uses a CSV database (`rooms.csv`) to resolve room names to IDs.
*    **Configurable Hardware:** Easy microphone selection via launch arguments.

---

## 2. Prerequisites

### Hardware
*   Raspberry Pi 4 / 5 or Laptop (Ubuntu 20.04)
*   USB Microphone (e.g., ReSpeaker, Jabra, or Generic USB)

### Software
*   ROS Noetic
*   Python 3.8+

---

## 3. Installation Guide (Replication Steps)

### Step 1: Install System Dependencies
Before cloning, install the required audio tools and Python libraries.
>>>>>>> f3b19965a95d07cc13e594c33bab06e49c63c26b

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
