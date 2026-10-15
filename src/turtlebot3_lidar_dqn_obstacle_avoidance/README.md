# turtlebot3_lidar_dqn_obstacle_avoidance

LiDAR perception + DQN (neural network) obstacle avoidance for TurtleBot3 in
Gazebo (course task 2). Implemented with numpy only: no torch or tensorflow
required, so it runs on the course VM (Ubuntu 20.04, ROS Noetic, Python 3.8).

## Network and algorithm
- State: 24 sectors compressed from `/scan` ranges, normalised to [0, 1].
- Action: 5 discrete commands `(linear, angular)` sent to `/cmd_vel`.
- Q network: 3-layer MLP 24-64-64-5, ReLU hides, Adam optimiser.
- Experience replay (20000) + target network synced every 50 steps.
- Bellman target: `y = r + gamma * max Q_target(s')`, gamma = 0.95.
- Reward: `+0.05 * (range_max - dist_to_goal)`, `-1.0` on collision
  (min sector < 0.25 m), `-0.2 * (0.4 - front_dist)` when front is close,
  `+1.0` on reaching the goal (dist < 0.6 m).
- Epsilon-greedy exploration, eps decays 0.995 per episode, floor 0.05.

## Topics
| Topic | Direction | Type | Use |
|---|---|---|---|
| /scan | subscribe | sensor_msgs/LaserScan | LiDAR perception input |
| /odom | subscribe | nav_msgs/Odometry | robot pose, goal distance |
| /cmd_vel | publish | geometry_msgs/Twist | motion control output |

## Parameters (launch args or private params)
| Name | Default | Meaning |
|---|---|---|
| mode | infer | `train` collects experience and updates the net; `infer` loads weights and drives greedily |
| episodes | 30 | training episode count when mode=train |

## Run environment
- Ubuntu 20.04, ROS Noetic, Gazebo 11 (course VM), Python 3.8 + numpy.

## Run steps
1. `source /opt/ros/noetic/setup.bash`
2. Train: `roslaunch src/turtlebot3_lidar_dqn_obstacle_avoidance/launch/lidar_dqn_obstacle_avoidance.launch mode:=train episodes:=30`
   or `python3 src/turtlebot3_lidar_dqn_obstacle_avoidance/scripts/main.py --mode train --episodes 30`
3. Infer: same launch with default mode, or
   `python3 src/turtlebot3_lidar_dqn_obstacle_avoidance/scripts/main.py`
4. Weights are saved next to the entry script as `dqn_weights.json` (text,
   kept in the repo so inference works without retraining).

## Test
Local unit test without Gazebo:
`python3 -m unittest tests/test_lidar_dqn_local.py`

## AI declaration
This module (code and README) was generated with the assistance of a large
language model; the author has reviewed the content and takes full
responsibility for the submission.
