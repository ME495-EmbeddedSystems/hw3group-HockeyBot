# Final Project Package: HockeyBot

FINISH THIS

## Authors: 
- **Ava Zahedi**
- **Hanyin Yuan**
- **Marthinius Nel**
- **Ritika Ghosh**

![grouppicture](https://user-images.githubusercontent.com/60728026/206877676-19116921-3ad7-4c1c-8c1e-c41048c0e0fe.jpeg)

## **Description**
The HockeyBot package allows a Franka robot to play air hockey. We use a realsense camera and computer vision to detect 
where the puck is on the air hockey table. Our TrajCalc node predicts the trajectory of the puck and sends those positions 
to our SimpleMove API, which tells the Franka to move to meet the puck. All of these tasks are integrated in our Main node, 
which completes our workflow and allows the robot to play repeatedly.


https://user-images.githubusercontent.com/60728026/206879150-de49eb95-b5cf-416f-8f51-24a34890e54b.mp4

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

10. Git clone `git@github.com:ME495-EmbeddedSystems/hw3group-HockeyBot.git` into the /src directory of the customer workspace. This file will install the ros dependencies required to run this project.

11. Additionally, users will need to install [Ubuntu install](https://docs.opencv.org/4.5.4/d2/de6/tutorial_py_setup_in_ubuntu.html) by using `sudo apt-get install python3-opencv`
All code for this package was developed and tested in Python 3.


## **Hardware Requirements**
This project requires the following hardware components:
* RealSense Camera: realsense-ros can be installed with `apt install ros-humble-realsense2-camera`
* Franka Emika Panda Robot ([Panda Robot](https://nu-msr.github.io/ros_notes/ros2/franka.html))
* Air-Hockey Table (include puck, puddle)
* To connect to the robot, plug in the ethernet from the robots workstation PC into the ethernet port on the users computer.
* The hardware must be set up by connecting the RealSense camera via USB cable.


## **Contents**
Packages:  
1. [hockeybot](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot/tree/main/hockeybot)
2. [moveit_helper](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot/tree/main/moveit_helper)
3. [moveit_interface](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot/tree/main/moveit_interface)

## **User Guide**
1. Follow the steps on website ([Turn on Franka](https://nu-msr.github.io/ros_notes/ros2/franka.html)) to start the Frank Robot.
2. Connecting the RealSense camera (via USB cable) and the Franka Emika Panda arm (via Ethernet cable) to the user's computer.

## Concepts and Overall System Architecture
The process loop of the robot is as follows:

### Start-Up Sequence

https://user-images.githubusercontent.com/60728026/206877658-1238a935-afeb-47ef-81f4-765bfd2c1d19.mp4

* Upon startup, the robot follows a start-up sequence to reach its home position. The robot follows a series of waypoints 
to reach the home x- and y-coordinates with an offset in the z. It then reaches down to grasp the paddle (with an adapter) 
and moves back up slightly. This slight increase in height allows the robot flexibility while moving, so that if it pushes 
down during movement, it will not apply a force into the table while still keeping the paddle level with the table.

### Computer Vision

https://user-images.githubusercontent.com/60728026/206877459-3620050d-346b-4bdc-b334-4dee55daace3.mp4

* Intel RealSense D435i is used at 480x270x90 allowing the puck to be tracked at 90 fps. As soon as the streaming has been enabled, this node detects the center of the table in pixel coordinates. Then with the help of the depth camera, the deproject function is used to convert pixel coordinates into real world coordinates with respect to the camera.  Since the distance between the air hockey table and the franks robot is known, points from the camera's frame of reference can be transformed to the robot's frame of reference. Next, with the help of OpenCV's HughCircles function the center of the puck is tracked in real time. For the calculation of the trajectory the puck is only tracked when going towards the robot and up to the center of the table. Inorder to get rid of noise, before publishing the puck's position its is checked whether the point is close (to a prefixed tolerance) to the best fit line of the previous positions obtained.Note: The output video shows the tracked puck encircled with a black border, regardless of whether all these points are published (shows the contour for every direction of movement of the puck).

### Trajectory Calculations
< insert media here >

### Hit the Puck
< insert media here >
* After receiving waypoint and goal positions, the robot receives service calls to move to those points, thereby meeting 
the puck along its trajectory and hitting it. If there is an edge case where the robot cannot successfully meet the puck 
given its trajectory, the robot will block instead.

### Return Home
< insert media here >
* As the robot is moving to hit the puck, it also plans a path from where it hits the puck back to the home position. 
Once the robot detects that its end-effector has reached the goal, it begins executing the path back home. This process 
also resets all internal variables and restarts the loop so that the robot can continue playing.

## Instructions: Manually launch the services for robot.
1. To launch the franka along with the simple_move node ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true.
2. Run the simple_move node with ros2 run moveit_helper simple_move.
    (Optional) Provide a starting configuration for planning with ros2 service call /initial_service moveit_interface/srv/Initial "{x: 0.5, y: 0.0, z: 0.0, roll: 1.0, pitch: 0.04, yaw: 0.0}".
3. Call the service to plan path to specifies goal pose ros2 service call /goal_service moveit_interface/srv/Goal "{x: 0.5, y: 0.0, z: 0.0, roll: 1.0, pitch: 0.04, yaw: 0.0}".
4. To execute the plan, use ros2 service call /execute_service moveit_interface/srv/Execute "exec_bool: True".
    a. If you wish to cancel your plan without executing, pass exec_bool: False instead of True.
5. To add a box in the planning scene, use ros2 service call /add_obj moveit_interface/srv/Addobj "{id: 1, x: 0.3, y: 0.6, z: 0.5, dim_x: 0.2, dim_y: 0.2, dim_z: 0.2}".
