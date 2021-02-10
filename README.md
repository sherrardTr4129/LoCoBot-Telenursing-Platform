# LoCoBot-Telenursing-Platform
This repository contains all the developed source code and digital assets needed for the construction, integration, and operation of the LoCoBot Telenursing Platform. All code software seen within this repository was adapted from the software developed for the LoCoBot platform by [facebook research](https://github.com/facebookresearch/pyrobot).

## Installation and Dependencies
This installation process was verified to complete sucessfully on Ubuntu 18.04 LTS using the python 2 option during installation. Follow the steps below to install required dependencies for executing the software found here. 


If you want to install the required software on your own machine for use within the Gazebo simulation enviornment, run the following on your local machine:

```bash
cd ~/catkin_ws_dir/src/LoCoBot-Telenursing-Platform/LoCoBot_ROS_packages/LoCoBot/install
chmod +x locobot_install_all.sh
./locobot_install_all.sh -t sim_only -p 2 -l interbotix
```

If you want to install the required software on the intel NUC onboard robot compute module, run the following within a terminal instance on the NUC. **Make sure you have the Intel RealSense camera plugged into the NUC before running the commands below.**

```bash
cd ~/catkin_ws_dir/src/LoCoBot-Telenursing-Platform/LoCoBot_ROS_packages/LoCoBot/install
chmod +x locobot_install_all.sh
./locobot_install_all.sh -t full -p 2 -l interbotix
```

**Note:** You can change the Python version flag from 2 to 3 if need be. Please not that the ROS packages contained within this repository were written to be used with Python 2.

## Usage
Just a stub for now

## Common Issues and Their Solutions
Just a stub for now

