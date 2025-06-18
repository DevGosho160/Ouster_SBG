# Ouster_SBG
ROS 2 driver packages to run, and log data from, an Ouster Lidar system and an SBG systems INS.

## Installation
### 1. Follow directions [here](https://docs.ros.org/en/humble/Installation.html) to install ROS2 Humble.
### 2. Install dependencies
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install libpcap-dev libopen3d-dev ros-humble-pcl-ros ros-humble-ament-cmake tcpdump
```
### 3. Create a workspace source folder and clone this github repository
```
cd ~
git clone
cd ~/Ouster_SBG
```
## Usage
### 1. Setup ethernet network for Ouster Lidar
a) Determine IP address of the Lidar
```
sudo tcpdump -i eno1 -n
```
b) Setup network with an IP address on the same subnet as the Ouster Lidar
```
sudo ip addr flush dev eno1
sudo ip addr add 192.168.1.10/24 dev eno1
sudo ip link set eno1 up
```
c) Ping Ouster Lidar to determine connection
```
ping 192.168.1.20
```
### 2. Run Ouster_SBG system
**Config Options**\
--mode: Lidar capture resolution and refresh rate, 512x10, 512x20, 1024x10, 1024x20, 2048x10, or 4096x5 (default: 1024x10)\
--start: Capture window start angle in millidegrees, 0-360000 (default: 0)\
--end: Capture window end angle in millidegrees, 0-360000 (default: 360000\
--viz: Use RViz2 to visualize Lidar data, True or False (default: False)\
```
sudo ~/ros2_ws/run_Ouster_SBG.sh --mode '1024x20' --start '90000' --end '270000' --viz 'True'
```
