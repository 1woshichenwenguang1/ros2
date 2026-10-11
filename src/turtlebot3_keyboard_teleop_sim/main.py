#!/usr/bin/env python3
"""Entry of task 1: launch Gazebo world then run keyboard teleop in foreground."""
import os
import subprocess
import sys

def main():
    env = dict(os.environ)
    env.setdefault('TURTLEBOT3_MODEL', 'burger')
    env.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    import glob as _g, os as _o
    _xs = sorted(_g.glob('/tmp/.X11-unix/X*'))
    if _xs and not env.get('DISPLAY'):
        env['DISPLAY'] = ':' + _xs[0].rsplit('X', 1)[-1]
    for _xa in (_o.path.expanduser('~/.Xauthority'), '/run/user/%d/gdm/Xauthority' % _o.getuid()):
        if not env.get('XAUTHORITY') and _o.path.exists(_xa):
            env['XAUTHORITY'] = _xa
            break
    src_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    env['ROS_PACKAGE_PATH'] = src_dir + os.pathsep + env.get('ROS_PACKAGE_PATH', '')
    if not env.get('ROS_MASTER_URI'):
        env['ROS_MASTER_URI'] = 'http://localhost:11311'
    gazebo = subprocess.Popen(
        ['roslaunch', 'turtlebot3_keyboard_teleop_sim', 'keyboard_teleop_sim.launch'], env=env)
    print('Gazebo is starting; drive with w/a/s/d/x keys in THIS terminal.')
    try:
        subprocess.call(['rosrun', 'turtlebot3_teleop', 'turtlebot3_teleop_key'], env=env)
    finally:
        gazebo.terminate()
        gazebo.wait()

if __name__ == '__main__':
    sys.exit(main())
