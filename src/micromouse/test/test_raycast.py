"""Ray distances checked against hand-computed geometry.

Cell pitch 0.30, wall thickness 0.02, so from a cell centre the near face of an
adjacent wall is 0.15 - 0.01 = 0.14 m away, and one open cell further is 0.44 m.
Those two numbers are what `scan_to_walls` keys off, so they are worth pinning.
"""
import math

import pytest

from micromouse import raycast as R, maze as M, conventions as C

CS = 0.30
WT = 0.02
NEAR = CS / 2 - WT / 2      # 0.14
FAR = CS + NEAR             # 0.44
RANGE_MAX = 4.0

BEARING = {C.N: math.pi / 2, C.E: 0.0, C.S: -math.pi / 2, C.W: math.pi}


def _centre(col, row):
    return C.cell_to_world(col, row, CS)


def _cast(maze, cell, direction, range_max=RANGE_MAX):
    x, y = _centre(*cell)
    return R.ray_distance(maze, CS, x, y, BEARING[direction], range_max, WT)


@pytest.mark.parametrize("direction", [C.N, C.E, C.S, C.W])
def test_a_sealed_cell_reads_the_same_short_range_all_round(direction):
    sealed = M.Maze(3)
    assert _cast(sealed, (1, 1), direction) == pytest.approx(NEAR, abs=1e-9)


def test_an_open_side_reads_one_cell_further():
    m = M.Maze(3)
    m.carve(1, 1, C.N)
    assert _cast(m, (1, 1), C.N) == pytest.approx(FAR, abs=1e-9)
    # Carving is reciprocal, so looking back down the corridor matches.
    assert _cast(m, (1, 2), C.S) == pytest.approx(FAR, abs=1e-9)


def test_a_long_corridor_accumulates_cell_by_cell():
    m = M.Maze(6)
    for y in range(5):
        m.carve(0, y, C.N)
    assert _cast(m, (0, 0), C.N) == pytest.approx(5 * CS + NEAR, abs=1e-9)


def test_range_max_saturates_instead_of_reporting_a_far_wall():
    m = M.Maze(6)
    for y in range(5):
        m.carve(0, y, C.N)
    assert _cast(m, (0, 0), C.N, range_max=0.5) == 0.5


def test_a_ray_along_a_corridor_ignores_the_walls_beside_it():
    """The classic ray-casting bug: a beam running parallel to a wall clips it.
    Here the beam runs the length of a corridor whose sides are solid."""
    m = M.Maze(6)
    for y in range(5):
        m.carve(0, y, C.N)
    x, y = _centre(0, 0)
    # Nudge the start off-centre toward the east wall, still inside the cell.
    hugging = R.ray_distance(m, CS, x + 0.05, y, math.pi / 2, RANGE_MAX, WT)
    assert hugging == pytest.approx(5 * CS + NEAR, abs=1e-9)


def test_starting_outside_the_maze_returns_no_hit():
    assert R.ray_distance(M.Maze(3), CS, -1.0, -1.0, 0.0, RANGE_MAX) == RANGE_MAX


def test_scan_matches_individual_casts_and_respects_yaw():
    m = M.Maze(3)
    m.carve(1, 1, C.N)
    x, y = _centre(1, 1)
    # Facing north: front is the open side, left and right are walls.
    front, left, right = R.scan(m, CS, x, y, math.pi / 2,
                                [0.0, math.pi / 2, -math.pi / 2],
                                RANGE_MAX, WT)
    assert front == pytest.approx(FAR, abs=1e-9)
    assert left == pytest.approx(NEAR, abs=1e-9)
    assert right == pytest.approx(NEAR, abs=1e-9)


def test_noise_perturbs_ranges_without_leaving_the_valid_interval():
    import random

    m = M.Maze(3)
    x, y = _centre(1, 1)
    rng = random.Random(0)
    vals = [R.scan(m, CS, x, y, 0.0, [0.0], RANGE_MAX, WT,
                   noise_stddev=0.01, rng=rng)[0] for _ in range(200)]
    assert any(v != NEAR for v in vals), "noise had no effect"
    assert all(0.0 <= v <= RANGE_MAX for v in vals)
    assert abs(sum(vals) / len(vals) - NEAR) < 0.005


def test_fan_angles_covers_a_full_circle_without_duplicating_a_bearing():
    angles = R.fan_angles(8)
    assert len(angles) == 8
    assert angles[0] == pytest.approx(-math.pi)
    assert angles[-1] < math.pi
    assert R.fan_angles(0) == []


def test_fan_angles_over_a_partial_arc_includes_both_ends():
    angles = R.fan_angles(3, -math.pi / 2, math.pi / 2)
    assert angles == pytest.approx([-math.pi / 2, 0.0, math.pi / 2])


def test_ranges_agree_with_the_wall_booleans_the_brain_will_derive():
    """The end-to-end contract: what the ray caster reports must make
    `scan_to_walls` produce the maze's actual walls."""
    from micromouse import scan_to_walls as S

    real = M.generate(n=8, seed=3)
    angles = R.fan_angles(72)
    for cell in [(0, 0), (3, 4), (7, 7), (5, 2)]:
        x, y = _centre(*cell)
        for heading in (C.N, C.E, C.S, C.W):
            ranges = R.scan(real, CS, x, y, BEARING[heading], angles,
                            RANGE_MAX, WT)
            rel = S.walls_from_scan(list(zip(angles, ranges)), CS)
            for x_, y_, d, is_wall in S.to_absolute(rel, cell, heading):
                assert is_wall == real.has_wall(x_, y_, d), (
                    "disagreed at %s facing %s about %s" % (cell, heading, d))
