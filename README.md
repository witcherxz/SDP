# Indoor Localization and Navigation System for Autonomous Robots

The project aims to develop an indoor localization and navigation system for autonomous robots that can accurately and reliably guide them from their current position to a desired destination.

## Localization

For localization, a combination of computer vision based on ArUco markers and odometer is used. This approach leverages the detection and recognition of ArUco markers to estimate the robot's position in the environment.

## Obstacle Avoidance

To enable obstacle avoidance, an OAK-D camera is used to detect and localize objects in front of the robot. The camera utilizes computer vision algorithms to analyze the captured images and identify obstacles. This information is then utilized to plan a safe path and avoid collisions.

## Navigation

A star algorithm is used to find the shortest path.

## Dependencies

The following dependencies are required for the project:

- ROS 1.
- RosAria: A driver, which allows communication with Pioneer 3-DX robot controller.
- OpenCV (version 4.6.0).
- depthAI: used to work with the OAK-D camera.
- [Catkin Tools](https://github.com/catkin/catkin_tools): for working with ROS build system.

Please make sure to have these dependencies installed in your environment to successfully run the project.

---

## Build
To build the project, `cd` to RobotController folder and using [Catkin Tools](https://github.com/catkin/catkin_tools) run

```bash
catkin build
```

---
## Calibration 

To calibrate the camera use Matlab R2022b Computer Vision Toolbox and lanuch calibration app. 

For calibration of the ArUco markers use `PoseCollection` helper class in localization node `localization/RobotVision/CameraCalibration/pose_collection` which can be used to collect the readings from the Aruco and provide the real measurements then the file is saved as a JSON file in default path you can modify it in `opencv_constans`.

Using the collected data points file use `systemCalibration.py` file to get a leaner regression model for each ArUco marker
And convert the model to a JSON file using the provided function in `systemCalibration.py` and save the JSON file in the default path `Calibration_Folder/arucos_coef.json"` specified in `opencv_constans`.

---

## Usage

To start the robot controller, run the following command:

```bash
./run_controller.bash run
```

To start the localization, navigation, and object detection nodes with the desired goal (x and y), run the following command:

```bash
./run_project.bash x y
```

---

## For debugging purposes, you can start the project manually by following steps:

1. Start `roscore` by running the command:

```bash
roscore
```

2. Grant permissions to the appropriate USB port (e.g., `/dev/ttyUSB0`). Replace `/dev/ttyUSB0` with the correct USB port if necessary:

```bash
sudo chmod 777 /dev/ttyUSB0
```

3. Start the RosAria node for robot control. If you encounter an error related to connecting to the robot on port `/dev/ttyUSB0`, specify a different port using the `port` parameter:

```bash
rosrun rosaria RosAria
```

4. Start the localization node: 

if you are using Raspberry pi v2 camera attached to the Jetson nano you may need to restart nvargus-daemon 

```
sudo systemctl restart nvargus-daemon
```
```bash
rosrun localization localization_node
```

5. Start the object detection node:

```bash
rosrun OBJ_D OBJ_D_node
```

6. Start the navigation node.

 This will launch a simple command-line interface (CLI) to change internal parameters of the robot controller. If you prefer not to use the CLI, provide the x and y goal position as arguments:

```bash
rosrun navigation navigation_node
```

Please note that these commands assume you have the necessary packages and dependencies installed in your ROS environment. Adjust the commands as needed based on your specific setup.
