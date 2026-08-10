"""Run metrics count what the robot did, using only what the robot knows."""
import pytest

from micromouse import belief_maze as B, conventions as C
from micromouse import explorer as E, run_metrics as RM


def _belief(walls=0):
    b = B.BeliefMaze(4)
    updates = [(0, i, C.N, True) for i in range(walls)]
    return b.learn_many(updates)


def test_moves_are_counted_per_phase():
    m = RM.RunMetrics()
    for _ in range(3):
        m.record_move(E.SEARCH)
    m.record_move(E.RETURN)
    for _ in range(2):
        m.record_move(E.SPEED)

    assert m.steps(E.SEARCH) == 3
    assert m.steps(E.RETURN) == 1
    assert m.steps(E.SPEED) == 2


def test_a_phase_boundary_does_not_shift_a_move_between_phases():
    """The reason moves are counted rather than derived from the trail: the
    cell a phase ends on is the cell the next phase starts on, so trail
    lengths double-count exactly at the boundary."""
    m = RM.RunMetrics()
    b = _belief()
    # Robot occupies the goal cell; it is the last SEARCH cell and the first
    # RETURN cell.
    m.note(E.SEARCH, (1, 1), b)
    m.record_move(E.SEARCH)
    m.note(E.RETURN, (1, 1), b)

    assert m.steps(E.SEARCH) == 1
    assert m.steps(E.RETURN) == 0
    assert len(m.cells_seen) == 1


def test_cells_and_walls_come_from_the_belief():
    m = RM.RunMetrics()
    m.note(E.SEARCH, (0, 0), _belief(walls=2))
    m.note(E.SEARCH, (0, 1), _belief(walls=3))

    assert m.as_dict()["cells_visited"] == 2
    assert m.as_dict()["walls_discovered"] == 3   # latest belief wins


def test_unknown_phases_are_ignored_rather_than_crashing():
    m = RM.RunMetrics()
    m.record_move(E.DONE)
    m.note(E.DONE, (2, 2), _belief())

    assert sum(m.moves.values()) == 0
    assert (2, 2) in m.cells_seen      # still counts as somewhere it stood


def test_summary_reports_every_phase_and_the_speedup():
    m = RM.RunMetrics()
    for _ in range(10):
        m.record_move(E.SEARCH)
    for _ in range(4):
        m.record_move(E.SPEED)
    m.note(E.SEARCH, (0, 0), _belief(walls=1))

    text = m.summary()
    assert "search run : 10 cells" in text
    assert "speed run  : 4 cells" in text
    assert "40%" in text


def test_optimality_gap_is_computed_outside_the_brain():
    assert RM.optimality_gap(50, 50) == 0.0
    assert RM.optimality_gap(60, 50) == pytest.approx(0.2)
    assert RM.optimality_gap(10, 0) is None

    assert "optimal" in RM.describe_gap(50, 50)
    assert "20.0% longer" in RM.describe_gap(60, 50)
    assert "no optimum" in RM.describe_gap(10, 0)


def test_summary_survives_a_run_that_never_reached_a_speed_run():
    m = RM.RunMetrics()
    m.record_move(E.SEARCH)
    text = m.summary()
    assert "speed run  : 0 cells" in text
    assert "%" not in text.split("speed run  : 0 cells")[-1]
