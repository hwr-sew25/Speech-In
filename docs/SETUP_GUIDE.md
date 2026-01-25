# 📖 Speech Interface Implementation Guide

**Author:** Team Speech-In  
**Component:** Offline Speech Recognition & Command Parsing  
**Repository:** https://github.com/hwr-sew25/Speech-In

---

## 1. Overview & Achievements
This guide explains how to replicate the **offline speech recognition system** built for the SEW25 project.
The module enables a robot to understand spoken navigation commands (e.g., "Drive to the Kitchen") without an internet connection.

**Key Features:**
*   🗣️ **Offline Speech-to-Text:** Powered by Vosk (Neural Network).
*   🧠 **Command Parsing:** Converts spoken sentences into structured JSON commands.
*   🗺️ **Dynamic Room Mapping:** Uses a CSV database (`rooms.csv`) to resolve room names to IDs.
*   ⚙️ **Configurable Hardware:** Easy microphone selection via launch arguments.

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

```bash
sudo apt update
sudo apt install -y alsa-utils python3-pip
pip3 install vosk sounddevice
