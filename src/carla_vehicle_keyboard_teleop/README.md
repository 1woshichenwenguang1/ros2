# carla_vehicle_keyboard_teleop

Keyboard teleoperation of a CARLA ego vehicle (course task 1, CARLA
simulator). Uses the CARLA Python API directly: no ros-bridge required.

## Architecture
- `scripts/main.py`: connect with retry and version print, synchronous world
  (fixed 0.05 s), spawn tesla.model3 with a 640x360 RGB camera, keyboard loop
  (w/a/s/d/x, q quit), per-step console HUD, headless evidence mode that saves
  camera PNGs to `run_evidence/`.
- `scripts/vehicle_control.py`: pure mapping keys+speed to
  throttle/steer/brake/reverse with explicit reverse-gear rule
  (|v| < 0.5 m/s and s -> reverse; else s -> brake). No carla import, so it is
  unit-testable anywhere.

## Run environment
- Windows host: CARLA server 0.9.13 (`CarlaUE4.exe`), port 2000.
- VM (Ubuntu 20.04, ROS Noetic): carla python client 0.9.13
  (`pip3 install carla==0.9.13`, already present in the course VM).
- Host IP seen from the VM over NAT: the VMnet8 gateway address,
  default `192.168.27.1` (verify with `ipconfig` on the host).

## Run steps
1. On Windows start `CarlaUE4.exe`.
2. In the VM:
   `python3 src/carla_vehicle_keyboard_teleop/scripts/main.py --host 192.168.27.1 --town Town03`
   and drive with w/a/s/d/x in the terminal.
3. Headless evidence (no 3D display):
   `python3 .../main.py --host 192.168.27.1 --headless --demo --frames 120`
   saves 120 camera PNGs into `run_evidence/`.
4. launch form:
   `roslaunch src/carla_vehicle_keyboard_teleop/launch/main.launch host:=192.168.27.1`
   or headless: add `headless:=true`.

## Interfaces
- control output: `carla.VehicleControl(throttle, steer, brake, reverse)`
  applied to the ego vehicle each synchronous tick;
- sensor input: RGB camera frames (evidence PNGs in headless mode);
- ROS topics: none required (pure CARLA API path).

## Test
`python3 -m unittest tests.test_carla_keyboard_local`
covers the reverse-gear rule, brake/throttle mapping and file presence
without a CARLA server.
