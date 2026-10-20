"""Pure keyboard-to-VehicleControl mapping, no CARLA import.

Kept free of carla/ros imports so it can be unit tested anywhere.
"""

THROTTLE = 0.6
BRAKE = 0.8
STEER = 0.5
REVERSE_SPEED = 0.5


class KeyboardController(object):
    """Maps a key set plus current speed to throttle/steer/brake/reverse."""

    def __init__(self):
        self.keys = set()

    def press(self, key):
        self.keys.add(key)

    def release(self, key):
        self.keys.discard(key)

    def clear(self):
        self.keys = set()

    def control(self, speed):
        """Return dict(throttle, steer, brake, reverse) for current keys.

        Reverse rule: at low speed (|v| < REVERSE_SPEED) pressing s engages
        reverse gear; at higher speed s brakes first.
        """
        throttle = 0.0
        steer = 0.0
        brake = 0.0
        reverse = False
        if 'w' in self.keys:
            throttle = THROTTLE
        if 's' in self.keys:
            if abs(speed) < REVERSE_SPEED:
                reverse = True
                throttle = THROTTLE * 0.5
            else:
                brake = BRAKE
        if 'a' in self.keys:
            steer += STEER
        if 'd' in self.keys:
            steer -= STEER
        if 'x' in self.keys:
            throttle = 0.0
            brake = BRAKE
            reverse = False
        return {'throttle': throttle, 'steer': steer,
                'brake': brake, 'reverse': reverse}
