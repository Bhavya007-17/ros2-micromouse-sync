"""The belief has to survive the trip from the brain to the viz window.

Both ends import ROS (and pygame), so these skip cleanly when run outside a
sourced workspace -- the rest of the suite deliberately needs neither.
"""
import pytest

from micromouse import belief_maze as B, conventions as C


def _round_trip(belief, phase="search", cell=(1, 2)):
    encode = pytest.importorskip("micromouse.explorer_brain").encode_belief
    decode = pytest.importorskip("micromouse.brain_viz").decode_belief
    return decode(encode(belief, phase, cell))


def test_a_belief_survives_encoding():
    belief = B.BeliefMaze(4).learn_many([
        (1, 1, C.N, True), (2, 2, C.E, False), (0, 0, C.S, True)])

    phase, cell, walls = _round_trip(belief)

    assert phase == "search"
    assert cell == (1, 2)
    assert walls[(1, 1)][C.N] == "#"
    assert walls[(1, 2)][C.S] == "#"      # reciprocity survives too
    assert walls[(2, 2)][C.E] == "."
    assert walls[(0, 0)][C.S] == "#"
    assert walls[(3, 3)][C.N] == "?"      # never sensed


def test_every_cell_is_present_so_the_viz_never_indexes_off_the_end():
    _, _, walls = _round_trip(B.BeliefMaze(6))
    assert len(walls) == 36
    assert all(len(walls[c]) == 4 for c in walls)


def test_a_malformed_frame_is_dropped_rather_than_crashing_the_window():
    viz = pytest.importorskip("micromouse.brain_viz")
    state = viz.VizState()
    state.set_belief("not a real frame")
    assert state.snapshot()[2] is None
