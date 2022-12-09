"""Test after impact y-intercept."""

from hockeybot.traj_calc import after_impact_traj_puck

def test_identity():
    assert after_impact_traj_puck([1, 1], 1) == 0