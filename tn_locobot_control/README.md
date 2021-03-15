# tn_locobot_control Package
This package provides the Locobot control script to translate topics published by the flask bridge into pyRobot commands. It meters commands based on the update rate of the web interface and the capabilities of the simulated Locobot (pending testing on the actual robot).  A launch file is also provided that starts the Gazebo simulation and all of the robot-side nodes.

## Usage
Start the launch file within the pyrobot environment:

```bash
roslaunch tn_locobot_control tn_locobot_control.launch 
```
