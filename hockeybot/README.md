# ME495 Final Project Package: HockeyBot

## Authors: 
- **Ava Zahedi**
- **Marthinius Nel**
- **Ritika Ghosh**
- **Hanyin Yuan**

## **Description**
The HockeyBot package allows a Franka robot to play air hockey. We use a realsense camera and computer vision to detect 
where the puck is on the air hockey table. Our TrajCalc node predicts the trajectory of the puck and sends those positions 
to our SimpleMove API, which is part of the [moveit_helper](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot/tree/main/moveit_helper) package, which tells the Franka to move to meet the puck. All of these tasks are integrated in our Main node, which completes our workflow and allows the robot to play repeatedly.

Software and hardware requirements to use this package are detailed in the root of the repository [here](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot).

## **Contents**
Nodes:  
1. cam_node
2. traj_calc
3. main

## **User Guide**
1. Follow the steps on website ([Turn on Franka](https://nu-msr.github.io/ros_notes/ros2/franka.html)) to start the Frank Robot.
2. Connecting the RealSense camera (via USB cable) and the Franka Emika Panda arm (via Ethernet cable) to the user's computer.
3. Launch the hockeybot package using the command `ros2 launch hockeybot main.launch.py robot:=false`.

How this package works in conjunction with moveit_helper and moveit_interface packages, as well as instructions for usage, is also detailed in the root of the repository [here](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot).