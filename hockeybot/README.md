# Final_Project_Package: HockeyBot
Authors: 
- **Ava Zahedi**
- **Hanyin Yuan**
- **Marthinius Nel**
- **Ritika Ghosh**

## **Description**
The Franka robot will be playing air hockey. A camera will be used to detect where
the puck is on the air hockey table and get the velocity of the puck, predict where it’s going, and
tell the Franka to meet the puck there.

## **User Guide**
1. Make sure ROS packages are most recent and up-to-date
```
sudo apt update
sudo apt upgrade

```
2.  Git clone `git@github.com:ME495-EmbeddedSystems/hw3group-HockeyBot.git` into the /src directory of the customer workspace. 	This file will install the ros dependencies required to run this project.
3. The `hockeybot` package contains the following nodes:
* `main`:  This node receives data from the CV node to be passed into the trajectory
    calculations node (Traj_Calc). It also receives the calculations back from Traj_Calc for
    additional processing to ultimately be passed into the SimpleMove API.
* `cam_node`: Use computer vision to detect puck and table frame, and send data to Main node.
* `traj_calc`:

