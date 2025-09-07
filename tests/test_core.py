import sar_swarm


def test_add():
    assert sar_swarm.add(2, 3) == 5
    assert sar_swarm.add(-1, 1) == 0
    assert sar_swarm.add(0, 0) == 0
