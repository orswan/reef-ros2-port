"""Unit tests of analyze_vision's interval logic (P08)."""
import math

import numpy as np

from reef_sim.analyze_vision import first_time, lost_intervals, mask_of


def h(t, state, published):
    return dict(t=t, state=state, published=published)


def test_lost_interval_runs_from_first_lost_to_next_published():
    seq = [h(0, 'OK', True), h(1, 'LOST', False), h(2, 'LOST', False), h(3, 'LOST', False),
           h(4, 'OK', False), h(5, 'OK', True), h(6, 'OK', True)]
    assert lost_intervals(seq) == [(1, 5)]


def test_unrecovered_loss_is_open_ended():
    assert lost_intervals([h(0, 'OK', True), h(1, 'LOST', False)]) == [(1, math.inf)]


def test_no_loss():
    assert lost_intervals([h(0, 'OK', True), h(1, 'OK', True)]) == []


def test_mask_and_first_time():
    t = np.array([0.0, 1.0, 2.0, 3.0])
    assert mask_of(t, [(1.0, 3.0)]).tolist() == [False, True, True, False]
    assert first_time(t, t > 1.5) == 2.0
    assert first_time(t, t > 9) is None
