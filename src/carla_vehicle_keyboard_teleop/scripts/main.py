#!/usr/bin/env python3
"""Course task 1 on CARLA: keyboard teleoperation of an ego vehicle.

Connects to a CARLA server on the Windows host, spawns an ego vehicle with
an RGB camera, drives it with w/a/s/d/x keys in synchronous 0.05 s steps and
saves camera frames as run evidence when --headless is used.
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

try:
    import carla
except ImportError:
    sys.stderr.write('carla python client missing: pip3 install carla==0.9.13\n')
    raise

from vehicle_control import KeyboardController

STEP = 0.05


def connect_client(host, port, retries=6):
    last = None
    for i in range(retries):
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        try:
            version = client.get_server_version()
            print('[main] connected to CARLA server %s:%s version %s'
                  % (host, port, version))
            return client
        except Exception as exc:  # noqa: BLE001
            last = exc
            print('[main] connect attempt %d failed: %s' % (i + 1, exc))
            time.sleep(3)
    raise RuntimeError('cannot reach CARLA server at %s:%s (%s)'
                       % (host, port, last))


def setup_world(client, town):
    try:
        world = client.load_world(town)
    except Exception:  # noqa: BLE001
        world = client.get_world()
        print('[main] load_world(%s) failed, using current world' % town)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = STEP
    world.apply_settings(settings)
    return world


def spawn_ego(world):
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find('vehicle.tesla.model3')
    spawn_points = world.get_map().get_spawn_points()
    actor = world.spawn_actor(bp, spawn_points[0])
    actor.set_autopilot(False)
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '360')
    transform = carla.Transform(carla.Location(x=1.6, z=1.6))
    camera = world.spawn_actor(camera_bp, transform, attach_to=actor)
    return actor, camera


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if r:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.27.1',
                        help='Windows host IP running CarlaUE4.exe')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--town', default='Town03')
    parser.add_argument('--headless', action='store_true',
                        help='no display: save camera frames as evidence')
    parser.add_argument('--demo', action='store_true',
                        help='auto-drive demo without keyboard')
    parser.add_argument('--save_dir', default='run_evidence')
    parser.add_argument('--frames', type=int, default=120,
                        help='evidence frame cap in headless mode')
    args = parser.parse_args(argv)

    client = connect_client(args.host, args.port)
    world = setup_world(client, args.town)
    vehicle, camera = spawn_ego(world)
    ctrl = KeyboardController()
    frames = []
    saved = 0

    def on_image(image):
        nonlocal saved
        if args.headless and saved < args.frames:
            if not os.path.isdir(args.save_dir):
                os.makedirs(args.save_dir)
            image.save_to_disk(os.path.join(
                args.save_dir, 'frame_%04d.png' % saved))
            saved += 1
        frames.append(image)

    camera.listen(on_image)
    print('[main] ego vehicle spawned; keys: w forward s brake/reverse '
          'a left d right x stop, q quit')
    demo_t = 0.0
    try:
        while True:
            if args.demo:
                demo_t += STEP
                ctrl.clear()
                ctrl.press('w')
                if int(demo_t * 4) % 4 in (1,):
                    ctrl.press('a')
                elif int(demo_t * 4) % 4 in (3,):
                    ctrl.press('d')
            else:
                key = read_key()
                if key == 'q':
                    break
                if key in 'wasdx':
                    if key in ctrl.keys:
                        ctrl.release(key)
                    else:
                        ctrl.press(key)
                        print('[main] pressed %s -> %s' % (key, ctrl.keys))
            speed = vehicle.get_velocity().length()
            cmd = ctrl.control(speed)
            vehicle.apply_control(carla.VehicleControl(
                throttle=cmd['throttle'], steer=cmd['steer'],
                brake=cmd['brake'], reverse=cmd['reverse']))
            world.tick()
            print('[main] v=%.2f m/s th=%.2f st=%.2f br=%.2f rev=%s saved=%d'
                  % (speed, cmd['throttle'], cmd['steer'], cmd['brake'],
                     cmd['reverse'], saved), flush=True)
            if args.headless and saved >= args.frames:
                print('[main] evidence complete: %d frames in %s'
                      % (saved, args.save_dir))
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        vehicle.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
    return 0


if __name__ == '__main__':
    sys.exit(main())
