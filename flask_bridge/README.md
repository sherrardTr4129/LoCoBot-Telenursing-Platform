# Flask Bridge Package
This package serves as the robot side of the cross-domain networked robot control scheme. It recieves POST requests from the robot web GUI backend and translates that data into ROS messages that are published throughout the rest of the system. 

## Usage
This node can be started using the following command:

```bash
rosrun flask_bridge ros_flask_bridge.py
```

## Published Topics Documentation
The table below outlines the topics that are published to from this node, along with descriptions of the message structure.

|             Topic Name            |        Message Type        |        Message Format       |                                                                    Description                                                                   |
|:---------------------------------:|:--------------------------:|:---------------------------:|:------------------------------------------------------------------------------------------------------------------------------------------------:|
| /pyRobot\_web\_data/cmd\_vel         | geometry\_msgs/Twist        | (see Twist doc.)            | The constructed command velocity from recieved user input. Both angular and linear velocities will be in range [-1,1].                           |
| /pyRobot\_web\_data/gripper\_state   | std\_msgs/Int8              | (see Int8 doc.)             | The state to set the robot gripper to, constructed from user input. A value of 1 will open the gripper, and a value of 0 will close the gripper. |
| /pyRobot\_web\_data/pan\_tilt\_offset | std\_msgs/Float32MultiArray | [pan, tilt]                 | The angle offset to move the pan/tilt controller for the Intel RealSense camera by, constructed from user input.                                 |
| /pyRobot\_web\_data/xyz\_arm\_offet   | std\_msgs/Float32MultiArray | [xOffset, yOffset, zOffset] | The individual cartesian axes increments to attempt to move the end-effector by, constructed from user input.                                    |
