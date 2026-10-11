# turtlebot3_keyboard_teleop_sim

Keyboard teleoperation of TurtleBot3 in Gazebo (course task 1).

## Run environment
- Ubuntu 20.04, ROS Noetic, Gazebo 11 (course VM)

## Run steps
1. `source /opt/ros/noetic/setup.bash`
2. `python3 src/turtlebot3_keyboard_teleop_sim/main.py`
3. Drive with `w a s d x`; quit with `Ctrl+C`.

The world alone can also be started with
`roslaunch src/turtlebot3_keyboard_teleop_sim/launch/keyboard_teleop_sim.launch`.
