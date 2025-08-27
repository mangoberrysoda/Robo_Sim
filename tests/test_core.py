import robo_sim


def test_add():
    assert robo_sim.add(2, 3) == 5
    assert robo_sim.add(-1, 1) == 0
    assert robo_sim.add(0, 0) == 0
