import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'carla_vehicle_keyboard_teleop',
                                'scripts'))
from vehicle_control import KeyboardController  # noqa: E402


class TestKeyboardController(unittest.TestCase):

    def test_forward(self):
        c = KeyboardController()
        c.press('w')
        cmd = c.control(0.0)
        self.assertGreater(cmd['throttle'], 0.0)
        self.assertEqual(cmd['brake'], 0.0)
        self.assertFalse(cmd['reverse'])

    def test_reverse_at_low_speed(self):
        c = KeyboardController()
        c.press('s')
        cmd = c.control(0.2)
        self.assertTrue(cmd['reverse'])
        self.assertGreater(cmd['throttle'], 0.0)

    def test_brake_at_high_speed(self):
        c = KeyboardController()
        c.press('s')
        cmd = c.control(3.0)
        self.assertFalse(cmd['reverse'])
        self.assertGreater(cmd['brake'], 0.0)
        self.assertEqual(cmd['throttle'], 0.0)

    def test_steer_and_stop(self):
        c = KeyboardController()
        c.press('a')
        self.assertGreater(c.control(1.0)['steer'], 0.0)
        c.release('a')
        c.press('d')
        self.assertLess(c.control(1.0)['steer'], 0.0)
        c.release('d')
        c.press('x')
        cmd = c.control(1.0)
        self.assertEqual(cmd['throttle'], 0.0)
        self.assertGreater(cmd['brake'], 0.0)

    def test_module_files_exist(self):
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                            'src', 'carla_vehicle_keyboard_teleop')
        self.assertTrue(os.path.isfile(
            os.path.join(root, 'scripts', 'main.py')))
        self.assertTrue(os.path.isfile(
            os.path.join(root, 'launch', 'main.launch')))


if __name__ == '__main__':
    unittest.main()
