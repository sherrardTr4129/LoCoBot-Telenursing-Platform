# This is the README for the LoCobot Telenursing platform
This repository contains all the developed source code and digital assets needed for the construction, integration, and operation of the LoCoBot Telenursing Platform. Our application depends on and derives from the software developed for the LoCoBot platform by [facebook research](https://github.com/facebookresearch/pyrobot).

## Pyrobot Installation and Dependencies
This installation process was verified to complete sucessfully on Ubuntu 18.04 LTS using the python 2 option during installation, and on a machine already configured with ROS Melodic. Follow the steps below to install required dependencies for executing the software found here. 

These instructions will set up your system to run a simulated LoCoBot in Gazebo only (instructions modified from https://pyrobot.org/docs/software).

### Prep your system and create a shortcut to the installation script:

sudo apt update

sudo apt-get install curl   (if not already installed)

curl 'https://raw.githubusercontent.com/facebookresearch/pyrobot/master/robots/LoCoBot/install/locobot_install_all.sh' > locobot_install_all.sh

### Make the script executable and run it.  
### Options: -t (type = simulator), -p (python version = 2), -l (platform = interbotix… Note for Trevor: we should verify this is correct for our hardware)

chmod +x locobot_install_all.sh
./locobot_install_all.sh -t sim_only -p 2 -l interbotix

## Start the LoCoBot simulation in Gazebo, and confirm successful startup by controlling the robot via teleop:

### Open three terminals.  In each:
source ~/pyenv_pyrobot_python2/bin/activate

### Start the simulation (terminal 1):

roslaunch locobot_control main.launch use_base:=true use_arm:= true use_sim:=true 

### Run teleop server (terminal 2):

cd ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_control/nodes
python robot_teleop_server.py

### Run teleop client (teminal 3):

cd ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_control/nodes
python keyboard_teleop_client.py

## Install and run Locobot Telenursing control and application sofware

### Install additional dependencies:

sudo apt-get install ros-melodic-web-video-server

pip install flask
pip install Flask-Cors

### Clone and run our software:

cd  ~/low_cost_ws/src
Clone this repository.
./catkin_make

### As above, start the simulation, this time without the teleop nodes:

source ~/pyenv_pyrobot_python2/bin/activate (in both terminals)

roslaunch telenursing_locobot_ctrl locobot_gui_control.launch 

roslaunch telenursing_web_gui locobot_GUI.launch

### Run the web GUI to view LoCoBot video and control the robot:

Open ~/low_cost_ws/src/LoCoBot-Telenursing-Platform/telenursing_web_gui/web/html/MainGUIPage_locobot.html
