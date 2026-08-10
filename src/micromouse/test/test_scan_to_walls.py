"""Turning a LaserScan into three wall booleans.

Kept free of ROS so the sensor interpretation can be tested with hand-written
numbers instead of a running simulator.
"""
import math

from micromouse import scan_to_walls as S, conventions as C

CELL = 0.30
NEAR = 0.14    # cell centre to the face of an adjacent wall
FAR = 0.44     # one open cell, then a wall


def _beam(center, value, spread=0.10, count=5):
    """A little fan of samples around `center`, all reading `value`."""
    return [(center + spread * (i / (count - 1) - 0.5), value)
            for i in range(count)]


def _scan(front=FAR, left=FAR, right=FAR):
    return (_beam(0.0, front)
            + _beam(math.pi / 2, left)
            + _beam(-math.pi / 2, right))


def test_near_returns_are_walls_and_far_returns_are_open():
    got = S.walls_from_scan(_scan(front=NEAR, left=FAR, right=NEAR), CELL)
    assert got == {"front": True, "left": False, "right": True}


def test_all_open():
    assert S.walls_from_scan(_scan(), CELL) == {
        "front": False, "left": False, "right": False}


def test_infinite_and_nan_returns_count_as_open():
    scan = _beam(0.0, float("inf")) + _beam(math.pi / 2, float("nan"))
    got = S.walls_from_scan(scan, CELL)
    assert got["front"] is False
    assert got["left"] is False


def test_a_sector_with_no_samples_is_unknown_not_open():
    got = S.walls_from_scan(_beam(0.0, NEAR), CELL)
    assert got["front"] is True
    assert got["left"] is None
    assert got["right"] is None


def test_samples_outside_the_window_are_ignored():
    """A beam pointing 45 degrees off must not be read as a front return."""
    got = S.walls_from_scan(_beam(math.pi / 4, NEAR, spread=0.0), CELL)
    assert got["front"] is None


def test_median_rejects_an_outlier():
    scan = [(0.0, NEAR), (0.02, NEAR), (-0.02, 3.0),
            (0.04, NEAR), (-0.04, NEAR)]
    assert S.walls_from_scan(scan, CELL)["front"] is True


def test_angles_wrap_so_a_rear_facing_scan_still_finds_left_and_right():
    scan = _beam(-3 * math.pi / 2, NEAR)      # same bearing as +pi/2
    assert S.walls_from_scan(scan, CELL)["left"] is True


def test_samples_from_ranges_reproduces_the_angles():
    samples = S.samples_from_ranges([1.0, 2.0, 3.0], -math.pi / 2,
                                    math.pi / 2)
    assert [a for a, _ in samples] == [-math.pi / 2, 0.0, math.pi / 2]
    assert [r for _, r in samples] == [1.0, 2.0, 3.0]


def test_to_absolute_maps_relative_readings_onto_compass_directions():
    rel = {"front": True, "left": False, "right": True}
    assert set(S.to_absolute(rel, (2, 3), C.N)) == {
        (2, 3, C.N, True), (2, 3, C.W, False), (2, 3, C.E, True)}
    assert set(S.to_absolute(rel, (2, 3), C.E)) == {
        (2, 3, C.E, True), (2, 3, C.N, False), (2, 3, C.S, True)}
    assert set(S.to_absolute(rel, (2, 3), C.S)) == {
        (2, 3, C.S, True), (2, 3, C.E, False), (2, 3, C.W, True)}
    assert set(S.to_absolute(rel, (2, 3), C.W)) == {
        (2, 3, C.W, True), (2, 3, C.S, False), (2, 3, C.N, True)}


def test_to_absolute_drops_unknown_sectors():
    rel = {"front": True, "left": None, "right": None}
    assert S.to_absolute(rel, (0, 0), C.N) == [(0, 0, C.N, True)]


def test_threshold_sits_clear_of_both_real_distances():
    """The decision boundary must not be near either physical case, or noise
    flips readings. NEAR and FAR should both be far from the threshold."""
    threshold = CELL * S.DEFAULT_THRESHOLD_FRAC
    assert NEAR < threshold - 0.05
    assert FAR > threshold + 0.05
