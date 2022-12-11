# moveit_interface package

Ava Zahedi, Hanyin Yuan, Marthinius Nel & Ritika Ghosh

This is a package providing services to be used with the [moveit_helper](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot/tree/main/moveit_helper) package.  

### Services
* GripperSrv: Open or close the gripper.
* Initial: Provide a starting pose to plan the goal from. This service should be called before the Goal service.
* Goal: Provide a goal pose to plan movement to.
* Execute: Execute the plan.
* AddObj: Add a box of given dimension to the planning scene.

Instructions in how to call these services are in the [moveit_helper](https://github.com/ME495-EmbeddedSystems/hw3group-HockeyBot/tree/main/moveit_helper) documentation.