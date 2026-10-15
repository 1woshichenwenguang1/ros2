#!/usr/bin/env python3
"""Entry of course task 2: LiDAR perception + DQN obstacle avoidance.

Subscribes /scan (LiDAR) and /odom, publishes /cmd_vel.
mode=train : collect experience and update the DQN each step.
mode=infer : load saved weights and drive with greedy policy.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    from dqn_agent import DQNAgent, sectors_from_ranges
except ImportError:
    from turtlebot3_lidar_dqn_obstacle_avoidance.dqn_agent import (
        DQNAgent, sectors_from_ranges)

N_SECTORS = 24
RANGE_MAX = 2.0
ACTIONS = [(0.22, 0.0), (0.22, 0.8), (0.22, -0.8), (0.10, 0.0), (0.0, 0.0)]
WEIGHT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'dqn_weights.json')
CRASH_DIST = 0.25
GOAL_DIST = 0.6


class DQNAvoidanceNode(object):

    def __init__(self, mode, episodes, world_proc=None):
        self.mode = mode
        self.episodes = episodes
        self.world_proc = world_proc
        self.agent = DQNAgent(state_dim=N_SECTORS, n_actions=len(ACTIONS))
        self.state = np.ones(N_SECTORS, dtype=np.float32)
        self.pose = np.zeros(2)
        self.goal = np.array([1.2, 0.0])
        self.last_cmd = time.time()
        self.episode = 0
        self.ep_steps = 0
        self.ep_reward = 0.0
        self.hit_count = 0
        self.reach_count = 0
        self.done = False
        self.reset_world = None
        if os.path.exists(WEIGHT_PATH):
            self.agent.load(WEIGHT_PATH)
            rospy.loginfo('loaded weights from %s', WEIGHT_PATH)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)

    def scan_cb(self, msg):
        self.state = sectors_from_ranges(msg.ranges, N_SECTORS, RANGE_MAX)

    def odom_cb(self, msg):
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def reward(self, dist_to_goal):
        min_obs = float(np.min(self.state)) * RANGE_MAX
        forward = self.state[N_SECTORS // 2] * RANGE_MAX
        r = 0.0
        r += (RANGE_MAX - dist_to_goal) * 0.05
        if min_obs < CRASH_DIST:
            r -= 1.0
            self.done = True
        elif forward < 0.4:
            r -= 0.2 * (0.4 - forward)
        if dist_to_goal < GOAL_DIST:
            r += 1.0
            self.done = True
        return r

    def publish(self, idx):
        lin, ang = ACTIONS[idx]
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def reset(self):
        self.episode += 1
        self.ep_steps = 0
        self.ep_reward = 0.0
        self.done = False
        self.goal = np.array([1.2 * math.cos(self.episode),
                              1.2 * math.sin(self.episode)])
        self._reset_world()
        rospy.loginfo('episode %d started, goal=(%.2f, %.2f)',
                      self.episode, self.goal[0], self.goal[1])

    def _reset_world(self):
        """把 Gazebo 世界复位，使机器人回到起点，避免撞墙后卡死在墙边。

        回合结束（碰撞/到达目标）时调用；失败时记录警告并继续，
        不阻塞训练主循环。
        """
        if self.reset_world is None:
            try:
                rospy.wait_for_service('/gazebo/reset_world', timeout=5)
                from gazebo_msgs.srv import ResetWorld
                self.reset_world = rospy.ServiceProxy(
                    '/gazebo/reset_world', ResetWorld)
            except Exception as exc:
                rospy.logwarn('gazebo reset_world unavailable: %s', exc)
                self.reset_world = False
        if self.reset_world:
            try:
                self.reset_world()
            except Exception as exc:
                rospy.logwarn('reset_world call failed: %s', exc)

    def loop(self):
        rate = rospy.Rate(5)
        self.reset()
        while not rospy.is_shutdown():
            dist = float(np.linalg.norm(self.goal - self.pose))
            r = self.reward(dist)
            action = self.agent.act(self.state, evaluate=(self.mode != 'train'))
            self.publish(action)
            rate.sleep()
            next_state = self.state.copy()
            next_dist = float(np.linalg.norm(self.goal - self.pose))
            if self.mode == 'train':
                self.agent.remember(self.state, action, r, next_state, self.done)
                loss = self.agent.train_step()
                if self.ep_steps % 50 == 0:
                    self.agent.update_target()
                if loss is not None:
                    rospy.loginfo('ep %d step %d reward %.2f loss %.4f eps %.2f',
                                  self.episode, self.ep_steps, r, loss,
                                  self.agent.eps)
            else:
                rospy.loginfo('step %d dist %.2f min_obs %.2f action %d',
                              self.ep_steps, next_dist,
                              float(np.min(next_state)) * RANGE_MAX, action)
            self.state = next_state
            self.ep_steps += 1
            self.ep_reward += r
            if self.done or self.ep_steps >= 300:
                if self.mode == 'train':
                    rospy.loginfo('episode %d finished reward %.1f steps %d',
                                  self.episode, self.ep_reward, self.ep_steps)
                    self.agent.decay()
                    if self.episode >= self.episodes:
                        self.agent.save(WEIGHT_PATH)
                        rospy.loginfo('training done, weights saved to %s',
                                      WEIGHT_PATH)
                        break
                    self.reset()
                else:
                    if self.done:
                        if dist < GOAL_DIST:
                            self.reach_count += 1
                            rospy.loginfo('goal reached (%d so far)',
                                          self.reach_count)
                        else:
                            self.hit_count += 1
                            rospy.loginfo('collision (%d so far)',
                                          self.hit_count)
                        if self.hit_count >= 3 or self.reach_count >= 3:
                            break
                        self.reset()
        self.stop()


def world_running():
    try:
        import rosgraph
        return rosgraph.is_master_online()
    except Exception:
        return False


def wait_master(timeout=120):
    try:
        import rosgraph
    except ImportError:
        time.sleep(15)
        return None
    deadline = time.time() + timeout
    while time.time() < deadline:
        if rosgraph.is_master_online():
            return True
        time.sleep(2)
    return None


def wait_service(name, timeout=90):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            rospy.wait_for_service(name, timeout=2)
            return True
        except Exception:
            continue
    return False


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['train', 'infer'], default=None)
    parser.add_argument('--episodes', type=int, default=30)
    parser.add_argument('--no-launch-world', action='store_true')
    args, _ = parser.parse_known_args(argv)
    # 必须先拉起 Gazebo 世界（会顺带启动 rosmaster），再初始化本节点；
    # 否则 rospy.init_node 会在 master 尚未就绪时无限重试，卡死在启动阶段。
    world_proc = None
    if not args.no_launch_world:
        if not world_running():
            env = dict(os.environ)
            env.setdefault('TURTLEBOT3_MODEL', 'burger')
            env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
            env.setdefault('DISPLAY', ':0')
            world_proc = subprocess.Popen(
                ['roslaunch', 'turtlebot3_gazebo', 'turtlebot3_world.launch'],
                env=env)
            time.sleep(3)
            if world_proc.poll() is not None:
                print('roslaunch of turtlebot3_world exited with code %s' %
                      world_proc.poll())
                return 1
        if wait_master() is None:
            print('ROS master not reachable after 120s, abort')
            return 1
        if not wait_service('/gazebo/get_world_properties'):
            print('gazebo world service not ready, abort')
            return 1
    rospy.init_node('dqn_obstacle_avoidance', anonymous=False)
    mode = args.mode or rospy.get_param('~mode', 'infer')
    episodes = args.episodes or int(rospy.get_param('~episodes', 30))
    try:
        node = DQNAvoidanceNode(mode, episodes, world_proc)
        node.loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        if world_proc is not None:
            world_proc.terminate()
            world_proc.wait()
    return 0


if __name__ == '__main__':
    sys.exit(main())
