# Final Project Package: HockeyBot
## Authors: 
- **Ava Zahedi**
- **Hanyin Yuan**
- **Marthinius Nel**
- **Ritika Ghosh**

## **Description**
The Franka robot will be playing air hockey. A camera will be used to detect where
the puck is on the air hockey table and get the velocity of the puck, predict where it’s going, and
tell the Franka to meet the puck there.

## **Prerequisites**
1. Make sure ROS packages are most recent and up-to-date
```
sudo apt update
sudo apt upgrade
```
2.  Install moveit: `sudo apt install ros-humble-moveit`

3. Make a new ROS2 workspace and enter it
```
mkdir -p nuws/src
cd nuws
```
4. Install the follow packages:
* Librealsense2, with Python bindings
* interbotix ROS packages for using the PincherX 100 Robot arm
* An updated ros2 launch with some not-yet-released bugfixes

by clone the repositories `vcs import --recursive --input https://raw.githubusercontent.com/m-elwin/numsr_patches/main/numsr_patches.repos src`
Librealsense2, with Python bindings
interbotix ROS packages for using the PincherX 100 Robot arm
An updated ros2 launch with some not-yet-released bugfixes
5. Install dependencies
`rosdep install --from-paths src -r -y`

6. Add the numsr colcon mixin
```
colcon mixin add numsr_patches file://$(pwd)/src/numsr_patches/index.yaml
colcon mixin update numsr_patches
```
7. Build the workspace
`colcon build --mixin numsr`

8. Install the udev rules
```
sudo cp src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d
sudo cp src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d
```
9. Source the environment
`source /opt/ros/humble/setup.bash`

10. Git clone `git@github.com:ME495-EmbeddedSystems/hw3group-HockeyBot.git` into the /src directory of the customer workspace. 	This file will install the ros dependencies required to run this project.

## **Hardware Requirements**
This project requires the following hardware components:
* RealSense Camera: realsense-ros can be installed with `apt install ros-humble-realsense2-camera`
* Franka Emika Panda Robot ([Panda Robot](https://nu-msr.github.io/ros_notes/ros2/franka.html))
* Air-hocket Table (include puck, puddle)
* To connect to the robot, plug in the ethernet from the robots workstation PC into the ethernet port on the users computer.
* The hardware must be set up by connecting the RealSense camera via USB cable.

## **Contents**
The `hockeybot` package contains:
1. nodes:
* `main`:  This node receives data from the CV node to be passed into the trajectory
    calculations node (Traj_Calc). It also receives the calculations back from Traj_Calc for
    additional processing to ultimately be passed into the SimpleMove API.
* `cam_node`: Use computer vision to detect puck and table frame, and send data to Main node.
* `traj_calc`: This node receives a starting position and an end goal position of the end effector, plans the
    path to the end goal configuration and then executes the path with the help of different
    services.
* `moveit_help`: It can plan a path to a specified pose or just a position or or just an orientation from any start
configuration. 
2. launch: 
* `main.launch.py`: master launch file for the robot - this runs all the nodes required to update the planning scene as well as move the robot 
* `realsense2.launch.py`: launch file for realsense2 camera node

## **Instructions: Manually launch the services for robot.**
1. To launch the franka along with the simple_move node ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true.
2. Run the simple_move node with ros2 run moveit_helper simple_move.
    (Optional) Provide a starting configuration for planning with ros2 service call /initial_service moveit_interface/srv/Initial "{x: 0.5, y: 0.0, z: 0.0, roll: 1.0, pitch: 0.04, yaw: 0.0}".
3. Call the service to plan path to specifies goal pose ros2 service call /goal_service moveit_interface/srv/Goal "{x: 0.5, y: 0.0, z: 0.0, roll: 1.0, pitch: 0.04, yaw: 0.0}".
4. To execute the plan, use ros2 service call /execute_service moveit_interface/srv/Execute "exec_bool: True".
    a. If you wish to cancel your plan without executing, pass exec_bool: False instead of True.
5. To add a box in the planning scene, use ros2 service call /add_obj moveit_interface/srv/Addobj "{id: 1, x: 0.3, y: 0.6, z: 0.5, dim_x: 0.2, dim_y: 0.2, dim_z: 0.2}".

