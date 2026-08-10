"""The explorer solving mazes it has never seen.

The whole policy runs offline here: a real generated maze, an oracle sensor that
answers only front/left/right from the robot's current cell, and a driver loop
that refuses to let the robot pass through a wall. No ROS, no Gazebo, no mms.

If these pass, the algorithm is correct and everything downstream is plumbing.
"""
import pytest

from micromouse import (conventions as C, explorer as E, flood_fill as FF,
                        maze as M, planner as P)

SEEDS = range(30)
N = 16


def _oracle(real, cell, heading):
    """A perfect three-way wall sensor: exactly what the robot could see from
    where it is standing, and nothing else."""
    x, y = cell
    return {
        "front": real.has_wall(x, y, heading),
        "left": real.has_wall(x, y, C.turn_left(heading)),
        "right": real.has_wall(x, y, C.turn_right(heading)),
    }


def _run(real, n=N):
    return _run_from(real, n, C.start_cell(), C.goal_cells(n))


def _run_from(real, n, start, goals):
    """Drive a full search/return/speed run. Returns the explorer and the number
    of cell moves spent in each phase."""
    ex = E.Explorer(n, start, goals)
    cell, heading = tuple(start), C.N
    moves = {E.SEARCH: 0, E.RETURN: 0, E.SPEED: 0}
    trail = {E.SEARCH: [cell], E.RETURN: [], E.SPEED: []}

    for _ in range(20000):
        ex.observe(cell, heading, _oracle(real, cell, heading))
        direction = ex.step(cell, heading)
        if direction is None:
            break
        phase = ex.phase
        for turn in P.turns_for(heading, direction):
            heading = P.apply_turn(heading, turn)
        assert heading == direction
        # The hard invariant: the robot may never move through a real wall.
        assert not real.has_wall(cell[0], cell[1], direction), (
            "walked through a wall at %s heading %s" % (cell, direction))
        cell = real.neighbor(cell[0], cell[1], direction)
        moves[phase] += 1
        trail[phase].append(cell)
    else:
        pytest.fail("explorer did not terminate")

    return ex, moves, trail, cell


def _optimal(real, n=N):
    return FF.flood_fill_distances(real, C.goal_cells(n))[C.start_cell()]


@pytest.mark.parametrize("seed", SEEDS)
def test_full_run_completes_without_cheating(seed):
    real = M.generate(n=N, seed=seed)
    ex, moves, trail, cell = _run(real)

    assert ex.phase == E.DONE
    assert cell in set(C.goal_cells(N))
    assert trail[E.RETURN][-1] == C.start_cell(), "did not return to start"
    assert trail[E.SPEED][-1] in set(C.goal_cells(N))
    assert moves[E.SEARCH] > 0 and moves[E.RETURN] > 0 and moves[E.SPEED] > 0


@pytest.mark.parametrize("seed", SEEDS)
def test_speed_run_never_beats_the_true_optimum(seed):
    """Beating the true shortest path would mean the robot saw the map."""
    real = M.generate(n=N, seed=seed)
    _, moves, _, _ = _run(real)
    assert moves[E.SPEED] >= _optimal(real)
    assert moves[E.SPEED] <= moves[E.SEARCH]


@pytest.mark.parametrize("seed", SEEDS)
def test_speed_run_is_optimal(seed):
    """Search + return exposes the true shortest route on these mazes.

    This is a property of the recursive-backtracker generator, not a guarantee
    of the algorithm: its passages branch rarely enough that heading for the
    goal through unknown ground happens to be right every time. It is pinned
    per-seed so a policy regression shows up as a specific maze that got worse
    -- see `test_recovers_from_a_dead_end_that_looked_like_a_shortcut` for the
    case where the heuristic is actually wrong.
    """
    real = M.generate(n=N, seed=seed)
    _, moves, _, _ = _run(real)
    assert moves[E.SPEED] == _optimal(real)


def _lure_maze():
    """An 8x8 built to punish 'head toward the goal through unknown ground'.

    Start is (0,4), the single goal is (4,4). A straight corridor runs east
    from the start directly at the goal and dead-ends one cell short. The only
    real route leaves north, away from the goal, and comes back down.

    Generated mazes never trigger this, so without it the backtracking code
    would be completely untested.
    """
    m = M.Maze(8)
    for x in range(3):                       # the lure: (0,4)..(3,4)
        m.carve(x, 4, C.E)
    m.carve(0, 4, C.N)                       # the real way out
    for x in range(4):                       # (0,5)..(4,5)
        m.carve(x, 5, C.E)
    m.carve(4, 5, C.S)                       # down into the goal
    return m


def test_the_lure_maze_really_is_a_trap():
    """Guard the fixture: if this stops being a trap the test below is
    vacuous."""
    m = _lure_maze()
    assert m.open_between(0, 4, C.E), "lure corridor must be open"
    assert not m.open_between(3, 4, C.E), "lure must dead-end short of the goal"
    assert FF.flood_fill_distances(m, [(4, 4)])[(0, 4)] == 6


def test_recovers_from_a_dead_end_that_looked_like_a_shortcut():
    m = _lure_maze()
    ex, moves, trail, cell = _run_from(m, n=8, start=(0, 4), goals=[(4, 4)])

    assert ex.phase == E.DONE
    assert cell == (4, 4)
    # It took the bait, hit the dead end, and had to walk back out.
    assert moves[E.SEARCH] > 6, "never entered the trap"
    assert (3, 4) in trail[E.SEARCH], "did not reach the dead end"
    assert len(trail[E.SEARCH]) > len(set(trail[E.SEARCH])), \
        "backtracking must revisit cells"
    # Having learned the trap, the speed run avoids it entirely.
    assert moves[E.SPEED] == 6
    assert (3, 4) not in trail[E.SPEED]


def test_belief_never_contradicts_the_real_maze():
    """Everything the robot thinks it knows must actually be true."""
    real = M.generate(n=N, seed=1)
    ex, _, _, _ = _run(real)
    from micromouse import belief_maze as B

    checked = 0
    for x in range(N):
        for y in range(N):
            for d in (C.N, C.E, C.S, C.W):
                st = ex.belief.state(x, y, d)
                if st == B.UNKNOWN:
                    continue
                assert (st == B.WALL) == real.has_wall(x, y, d), \
                    "belief is wrong at (%d,%d) %s" % (x, y, d)
                checked += 1
    assert checked > 100


def test_explorer_learns_rather_than_starting_informed():
    real = M.generate(n=N, seed=2)
    fresh = E.Explorer(N, C.start_cell(), C.goal_cells(N))
    assert fresh.belief.known_count() == 0

    ex, _, _, _ = _run(real)
    assert ex.belief.known_count() > 100
    # It should NOT need the whole maze -- exploring everything would be a
    # different (and much slower) algorithm.
    total_walls = 2 * N * (N + 1)
    assert ex.belief.known_count() < total_walls


def test_speed_phase_falls_back_to_searching_when_nothing_is_verified():
    """A robot told to speed-run before it has learned anything must go back to
    exploring, not guess its way through unknown ground."""
    ex = E.Explorer(N, C.start_cell(), C.goal_cells(N))
    ex.phase = E.SPEED

    direction = ex.step(C.start_cell(), C.N)

    assert ex.phase == E.SEARCH
    assert ex.fallbacks == 1
    assert direction is not None, "should have resumed exploring"


def test_repeated_fallbacks_give_up_instead_of_looping_forever():
    ex = E.Explorer(N, C.start_cell(), C.goal_cells(N))
    ex.phase = E.SPEED
    ex.fallbacks = E.MAX_FALLBACKS

    assert ex.step(C.start_cell(), C.N) is None
    assert ex.phase == E.SPEED


def test_distances_follow_the_phase():
    real = M.generate(n=N, seed=8)
    ex, _, _, _ = _run(real)
    assert ex.phase == E.DONE

    # A finished run steers by the verified map, and it agrees with reality.
    ex.phase = E.SPEED
    speed = ex.distances()
    assert speed[C.start_cell()] == _optimal(real)

    # Mid-search the target is the goal; on the way home it is the start.
    ex.phase = E.SEARCH
    assert ex.targets() == set(C.goal_cells(N))
    assert ex.distances()[C.start_cell()] == _optimal(real)
    ex.phase = E.RETURN
    assert ex.targets() == {C.start_cell()}
    assert ex.distances()[C.start_cell()] == 0


def test_pessimistic_route_is_unreachable_before_exploring():
    """The premise of the SPEED phase: a robot that has seen nothing cannot
    verify any route, so it must fall back to searching."""
    from micromouse import belief_maze as B

    blind = B.BeliefMaze(N).pessimistic_view()
    dist = FF.flood_fill_distances(blind, C.goal_cells(N))
    assert dist[C.start_cell()] >= FF.INF
