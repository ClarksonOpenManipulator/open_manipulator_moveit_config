To launch run the following:

```roslaunch open_manipulator_moveit_config moveit_planning_execution.launch```

In the MoveIt! interface within RVIZ you can control the manipulator with predefined poses specified/defined
in the SRDF in the config directory. The pick the pose, navigate to the "Planning" tab in RVIZ. 
Select "<current>" in the "Select Start State" windowthen click "Update". Next select your desired pose in 
the "Select Goal State" window and click update. FINALLY, click "Plan" to plan the trajectory, then "Execute" 
to issue the planned trajectory to the manipulator.


To operate the gripper issue std_msgs/String commands to the the /gripper_command topic:

```rostopic pub /gripper_command std_msgs/String "data: 'grip_on'"```
                            -OR-
```rostopic pub /gripper_command std_msgs/String "data: 'grip_off'"```