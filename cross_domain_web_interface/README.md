# PyRobot Web interface
This folder contains the source code for the front-end and back-end to be served from a web hosting machine for the control of the physical pyrobot instance. 

## Frontend
The front end portion of the interface is contained within the web sub-directory within this folder.

## Backend
The flask backend application that communicates with the front-end application and with the robot's API can be found under the /src sub-folder in this section. The program can be run with the following command:

```bash
python3 src/web_backend.py -u <NGROK URL>
```

Make sure to replace \<NGROK URL\> with the current ngrok url of the robot flask bridge.

Note that the robot hostname is currently set to "http://localhost:12345" for testing purposes. Make sure to change this for the production demo. The backend will attempt to make a POST request to the robot URL when new user input is recieved, or when data axes are non-zero (except in the case of push buttons). 

## Backend API documentation
The API that can be used to interact with the back end from the front end can be seen documented in the table below.

|     Endpoint     | Method | JSON Data Format (Where x is the value) |                                                             Description                                                            |
|:----------------:|:------:|:---------------------------------------:|:----------------------------------------------------------------------------------------------------------------------------------:|
| /setFwdRev       | POST   | fwdRevData = { "fwdRev": x }            | This endpoint allows the frontend to update the linear velocity sent to the robot.                |
| /setSpin         | POST   | spinData = { "spin": x }                | This endpoint allows the frontend to update the angular velocity sent to the robot.                |
| /setArmOffsetX   | POST   | xData = { "xArmOffset": x}              | This endpoint allows the frontend to send an x-axis increment to move the end-effector in for the robot manipulator.               |
| /setArmOffsetY   | POST   | yData = { "yArmOffset": x}              | This endpoint allows the frontend to send an y-axis increment to move the end-effector in for the robot manipulator.               |
| /setArmOffsetZ   | POST   | zData = { "zArmOffset": x}              | This endpoint allows the frontend to send an z-axis increment to move the end-effector in for the robot manipulator.               |
| /setPanOffset    | POST   | panData = { "panOffsetData": x}         | This endpoint allows the frontend to send an offset to move the "pan" of the pan-tilt camera base by.                              |
| /setTiltOffset   | POST   | tiltData = {"tiltOffsetData": x}        | This endpoint allows the frontend to send an offset to move the "tile" of the pan-tilt camera base by.                             |
| /setGripperState | POST   | gripperState = {"gripperStateData": x}  | This endpoint allows the frontend to send a new state for the robot gripper to try to achieve. The value should be either 0 or 1.  |
