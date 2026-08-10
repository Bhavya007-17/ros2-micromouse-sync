"""BeliefMaze is what the robot thinks the maze is.

The two things that must hold: it stays reciprocal like a real maze, and it
answers "can I go this way" differently depending on whether we are exploring
(optimistic) or committing to a speed run (pessimistic).
"""
import pytest

from micromouse import belief_maze as B, conventions as C
from micromouse import flood_fill as FF, planner as P, maze as M


def test_starts_completely_unknown_except_the_perimeter():
    b = B.BeliefMaze(4)
    assert b.state(1, 1, C.N) == B.UNKNOWN
    assert b.known_count() == 0
    # The perimeter needs no sensing: you cannot leave a 4x4 maze.
    assert not b.open_between(0, 0, C.S)
    assert not b.open_between(0, 0, C.W)
    assert not b.open_between(3, 3, C.N)
    assert not b.open_between(3, 3, C.E)


def test_optimistic_treats_unknown_as_open():
    b = B.BeliefMaze(4)
    assert b.open_between(1, 1, C.N)
    assert b.open_between(1, 1, C.E)


def test_pessimistic_treats_unknown_as_wall():
    b = B.BeliefMaze(4).pessimistic_view()
    assert not b.open_between(1, 1, C.N)
    b2 = b.learn_many([(1, 1, C.N, False)])
    assert b2.open_between(1, 1, C.N)


def test_learning_is_reciprocal():
    b = B.BeliefMaze(4).learn_many([(1, 1, C.N, True)])
    assert b.state(1, 1, C.N) == B.WALL
    assert b.state(1, 2, C.S) == B.WALL
    assert not b.open_between(1, 2, C.S)


def test_learn_many_does_not_mutate_the_original():
    b = B.BeliefMaze(4)
    b2 = b.learn_many([(1, 1, C.N, True), (2, 2, C.E, False)])
    assert b.state(1, 1, C.N) == B.UNKNOWN
    assert b.state(1, 2, C.S) == B.UNKNOWN
    assert b2.state(1, 1, C.N) == B.WALL
    assert b2.state(2, 2, C.E) == B.OPEN
    assert b2 is not b


def test_known_count_ignores_repeat_observations():
    b = B.BeliefMaze(4).learn_many([(1, 1, C.N, True)])
    assert b.known_count() == 1
    assert b.learn_many([(1, 1, C.N, True)]).known_count() == 1
    assert b.learn_many([(1, 2, C.S, True)]).known_count() == 1


def test_learning_from_a_cell_outside_the_grid_is_ignored():
    b = B.BeliefMaze(4).learn_many([(9, 9, C.N, True), (-1, 0, C.E, True)])
    assert b.known_count() == 0


def test_sensing_the_perimeter_from_inside_is_recorded():
    """(0,0,S) points out of the grid, but the robot is standing in (0,0) and
    really does see that wall -- it is an observation, not an out-of-bounds
    write."""
    b = B.BeliefMaze(4).learn_many([(0, 0, C.S, True)])
    assert b.state(0, 0, C.S) == B.WALL
    assert b.known_count() == 1


def test_later_observation_wins():
    """Static maze, so this only matters under sensor noise. Trusting the most
    recent read keeps one bad sample from being permanent."""
    b = B.BeliefMaze(4).learn_many([(1, 1, C.N, True)])
    assert b.learn_many([(1, 1, C.N, False)]).state(1, 1, C.N) == B.OPEN


def test_visiting_is_immutable_and_accumulates():
    b = B.BeliefMaze(4)
    b2 = b.visit((0, 0)).visit((0, 1)).visit((0, 0))
    assert b.visited == frozenset()
    assert b2.visited == {(0, 0), (0, 1)}


def test_views_share_knowledge_but_not_optimism():
    b = B.BeliefMaze(4).learn_many([(1, 1, C.N, True)])
    p = b.pessimistic_view()
    o = p.optimistic_view()
    assert p.state(1, 1, C.N) == o.state(1, 1, C.N) == B.WALL
    assert o.optimistic and not p.optimistic
    assert p.visited == b.visited


def test_flood_fill_and_planner_accept_a_belief_unchanged():
    """The whole design rests on BeliefMaze duck-typing as a Maze, so the
    existing flood_fill and planner are reused rather than reimplemented."""
    b = B.BeliefMaze(6)
    goals = C.goal_cells(6)
    dist = FF.flood_fill_distances(b, goals)
    # Nothing sensed yet and optimism assumes open, so it looks like open field:
    # Manhattan distance to the nearest goal cell.
    assert dist[(0, 0)] == 4
    assert P.pick_next_dir(b, dist, (0, 0), C.N) is not None


def test_belief_of_a_fully_sensed_maze_matches_the_real_thing():
    real = M.generate(n=8, seed=4)
    updates = [(x, y, d, real.has_wall(x, y, d))
               for x in range(8) for y in range(8)
               for d in (C.N, C.E, C.S, C.W)]
    b = B.BeliefMaze(8).learn_many(updates)
    goals = C.goal_cells(8)
    assert (FF.flood_fill_distances(b, goals)
            == FF.flood_fill_distances(real, goals))
    # And a pessimistic view of a fully-known maze is the same maze.
    assert (FF.flood_fill_distances(b.pessimistic_view(), goals)
            == FF.flood_fill_distances(real, goals))


@pytest.mark.parametrize("d", [C.N, C.E, C.S, C.W])
def test_has_wall_only_true_for_confirmed_walls(d):
    b = B.BeliefMaze(4)
    assert not b.has_wall(1, 1, d)               # unknown is not a wall
    assert b.learn_many([(1, 1, d, True)]).has_wall(1, 1, d)
    assert not b.learn_many([(1, 1, d, False)]).has_wall(1, 1, d)
