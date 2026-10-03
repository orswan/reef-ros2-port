"""Stall detection for the scenario runners (no ROS dependency).

A simulation is stalled when sim time has not advanced for timeout_s wall
seconds while the runner was able to observe it. Wall time alone cannot tell
a stalled simulation from a runner that was not scheduled: when the whole
container is frozen (an overloaded host can pause Docker's VM for tens of
seconds), Gazebo and the runner stop together, and on resume the runner sees
its last sim time and a large wall-time gap before it has processed the
samples queued meanwhile. So a timeout is only a suspicion; the runner must
drain its subscriptions (drain_s wall seconds at most) and confirm that sim
time still stands still before it reports a stall.
"""
import time


class StallWatch:

    def __init__(self, timeout_s, drain_s=1.0, clock=time.monotonic):
        self.timeout_s, self.drain_s, self.clock = timeout_s, drain_s, clock
        self.last_sim, self.last_wall = None, clock()

    def reset(self, sim):
        self.last_sim, self.last_wall = sim, self.clock()

    def suspect(self, sim):
        """Record progress; True if sim time has not advanced for timeout_s."""
        if self.last_sim is None or sim > self.last_sim:
            self.reset(sim)
            return False
        return self.clock() - self.last_wall > self.timeout_s

    def stalled(self, sim_now, spin_once):
        """True only if sim time stays put through a drain of pending input.

        sim_now() returns the current sim time; spin_once() processes pending
        callbacks (blocking briefly at most).
        """
        if not self.suspect(sim_now()):
            return False
        deadline = self.clock() + self.drain_s
        while self.clock() < deadline:
            spin_once()
            if sim_now() > self.last_sim:
                self.reset(sim_now())
                return False
        return True
