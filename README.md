# hw3group-HockeyBot

Ava Zahedi, Hanyin Yuan, Marthinius Nel & Ritika Ghosh

The moveit_helper and moveit_interfaces packages allow the user to send a desired end-effector\
pose (position and/or orientation) to the Franka robot, have a motion plan generated, and then\
have the option for the trajectory be executed.

Instructions:
1. Run the simple_move node with `ros2 run moveit_helper simple_move`.
2. (Optional) Provide a starting configuration for planning with `ros2 service call blahblah`.
3. Call the service to plan with `ros2 service call /goal_service moveit_interface/srv/Goal "{pose_x: 0.5, pose_y: 0.0, pose_z: 0.0, orientation_x: 0.0, orientation_y: 0.04, orientation_z: 0.5, orientation_w: 1.0}"`.
4. To execute the plan, use `ros2 service call /execute_service moveit_interface/srv/Execute "exec_bool: True"`.
    a. If you wish to cancel your plan without executing, pass `exec_bool: False` instead of `True`.
5. To add a box in the planning scene, use `box stuffs`.




Commands to run everything:

ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 run moveit_helper simple_move

ros2 service call /initial_service moveit_interface/srv/Initial "{x: 0.5, y: 0.0, z: 0.0, roll: 1.0, pitch: 0.04, yaw: 0.0}"

ros2 service call /goal_service moveit_interface/srv/Goal "{x: 0.5, y: 0.0, z: 0.0, roll: 1.0, pitch: 0.04, yaw: 0.0}"

ros2 service call /execute_service moveit_interface/srv/Execute "exec_bool: True"

ros2 service call /add_obj moveit_interface/srv/Addobj "{id: 1, x: 0.3, y: 0.6, z: 0.5, dim_x: 0.2, dim_y: 0.2, dim_z: 0.2}"