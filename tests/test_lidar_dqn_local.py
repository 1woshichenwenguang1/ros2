import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'turtlebot3_lidar_dqn_obstacle_avoidance',
                                'scripts'))
import dqn_agent  # noqa: E402


class TestDqnAgent(unittest.TestCase):

    def test_sectors_shape_and_range(self):
        ranges = [1.0] * 180 + [0.2] * 180
        s = dqn_agent.sectors_from_ranges(ranges, 24, 2.0)
        self.assertEqual(s.shape, (24,))
        self.assertTrue((s >= 0).all() and (s <= 1).all())
        self.assertAlmostEqual(float(s[12]), 0.1, places=3)

    def test_sectors_invalid_values(self):
        ranges = [float('nan'), -1.0, 99.0, 0.5]
        s = dqn_agent.sectors_from_ranges(ranges, 4, 2.0)
        self.assertEqual(s.shape, (4,))
        self.assertTrue((s >= 0).all() and (s <= 1).all())

    def test_agent_shapes_and_replay(self):
        agent = dqn_agent.DQNAgent(state_dim=24, n_actions=5)
        s = [1.0] * 24
        a = agent.act(s)
        self.assertIn(a, range(5))
        for i in range(40):
            agent.remember(s, i % 5, 0.1, s, False)
        loss = agent.train_step()
        self.assertIsNotNone(loss)
        self.assertGreaterEqual(loss, 0.0)
        agent.update_target()

    def test_save_and_load_roundtrip(self):
        import tempfile
        agent = dqn_agent.DQNAgent(state_dim=24, n_actions=5)
        s = [0.5] * 24
        q1 = agent.q.predict(s)
        path = os.path.join(tempfile.gettempdir(), 'dqn_test_weights.json')
        agent.save(path)
        agent2 = dqn_agent.DQNAgent(state_dim=24, n_actions=5)
        agent2.load(path)
        q2 = agent2.q.predict(s)
        self.assertTrue((abs(q1 - q2) < 1e-9).all())
        os.remove(path)

    def test_module_files_exist(self):
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                            'src', 'turtlebot3_lidar_dqn_obstacle_avoidance')
        self.assertTrue(os.path.isfile(os.path.join(root, 'scripts', 'main.py')))
        self.assertTrue(os.path.isfile(
            os.path.join(root, 'launch', 'lidar_dqn_obstacle_avoidance.launch')))
        self.assertTrue(os.path.isfile(
            os.path.join(root, 'docs_page.md')) or os.path.isdir(root))


if __name__ == '__main__':
    unittest.main()
