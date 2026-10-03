"""Unit tests for reef_sim.stall with a simulated wall clock."""
import unittest

from reef_sim.stall import StallWatch


class FakeWorld:
    """Wall clock, sim time as last processed by the runner, and a queue of pending samples."""

    def __init__(self):
        self.wall, self.sim, self.pending = 0.0, 0.0, []

    def clock(self):
        return self.wall

    def sim_now(self):
        return self.sim

    def spin_once(self):
        self.wall += 0.05
        if self.pending:
            self.sim = self.pending.pop(0)


class StallWatchTests(unittest.TestCase):

    def setUp(self):
        self.w = FakeWorld()
        self.watch = StallWatch(20.0, drain_s=1.0, clock=self.w.clock)
        self.assertFalse(self.watch.stalled(self.w.sim_now, self.w.spin_once))

    def test_progress_is_not_a_stall(self):
        for _ in range(1000):
            self.w.wall += 0.05
            self.w.sim += 0.01
            self.assertFalse(self.watch.stalled(self.w.sim_now, self.w.spin_once))

    def test_whole_container_freeze_is_not_a_stall(self):
        # Gazebo and the runner both frozen for 30 s; on resume, samples are queued.
        self.w.wall += 30.0
        self.w.pending = [0.01, 0.02]
        self.assertFalse(self.watch.stalled(self.w.sim_now, self.w.spin_once))
        self.assertEqual(self.w.sim, 0.01)
        self.assertFalse(self.watch.stalled(self.w.sim_now, self.w.spin_once))

    def test_stopped_simulation_is_a_stall(self):
        for _ in range(390):   # 19.5 s without progress
            self.w.wall += 0.05
            self.assertFalse(self.watch.stalled(self.w.sim_now, self.w.spin_once))
        self.w.wall += 0.6
        t0 = self.w.wall
        self.assertTrue(self.watch.stalled(self.w.sim_now, self.w.spin_once))
        self.assertLessEqual(self.w.wall - t0, 1.0 + 0.05)   # drain is bounded

    def test_below_timeout_no_drain(self):
        self.w.wall += 19.0
        self.assertFalse(self.watch.stalled(self.w.sim_now, self.w.spin_once))
        self.assertEqual(self.w.wall, 19.0)   # no spin: not suspected yet


if __name__ == '__main__':
    unittest.main()
