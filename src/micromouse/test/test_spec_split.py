"""The public spec must not leak the maze.

The explorer brain is only allowed to read the public spec. If walls can reach
it through that file, the whole "solves an unseen maze" claim is void, so these
tests pin the boundary rather than trusting convention.
"""
import json
import os
import tempfile

import pytest

from micromouse import maze as M, spec, conventions as C


def _maze(seed=11, n=16):
    return M.generate(n=n, seed=seed)


def test_public_spec_has_no_walls():
    m = _maze()
    path = os.path.join(tempfile.mkdtemp(), spec.PUBLIC_NAME)
    spec.write_public(path, m, 0.30, 0.02, 0.15)
    with open(path) as f:
        data = json.load(f)
    assert "walls" not in data
    assert set(data) == set(spec.PUBLIC_KEYS)


def test_public_spec_carries_everything_the_brain_needs():
    m = _maze(n=8)
    path = os.path.join(tempfile.mkdtemp(), spec.PUBLIC_NAME)
    spec.write_public(path, m, 0.25, 0.02, 0.15)
    data = spec.read_public(path)
    assert data["n"] == 8
    assert data["cell_size_m"] == 0.25
    assert tuple(data["start_cell"]) == C.start_cell()
    assert {tuple(g) for g in data["goal_cells"]} == set(C.goal_cells(8))


def test_truth_spec_roundtrips_walls():
    m = _maze(seed=12)
    path = os.path.join(tempfile.mkdtemp(), spec.TRUTH_NAME)
    spec.write_truth(path, m, 0.30, 0.02, 0.15)
    back, data = spec.read_truth(path)
    assert data["cell_size_m"] == 0.30
    assert all(back.walls[c] == m.walls[c] for c in m.walls)


def test_read_public_refuses_a_file_that_contains_walls():
    """Pointing the brain at maze_truth.json must fail loudly, not silently
    hand it the answer."""
    m = _maze()
    path = os.path.join(tempfile.mkdtemp(), spec.TRUTH_NAME)
    spec.write_truth(path, m, 0.30, 0.02, 0.15)
    with pytest.raises(ValueError, match="walls"):
        spec.read_public(path)


def test_legacy_read_write_still_work():
    """gazebo_sync_brain (the omniscient baseline) still uses these."""
    m = _maze(seed=13)
    path = os.path.join(tempfile.mkdtemp(), "spec.json")
    spec.write(path, m, 0.30, 0.02, 0.15)
    back, data = spec.read(path)
    assert all(back.walls[c] == m.walls[c] for c in m.walls)
    assert data["n"] == 16


def test_the_blind_modules_never_mention_the_truth_file():
    """A grep-level guard on the anti-cheat boundary.

    `read_public` already refuses a file containing walls at runtime, but this
    catches the other direction: someone adding a direct read of the truth file
    or of a wall dictionary to code that is supposed to be blind.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.join(os.path.dirname(here), "micromouse")
    blind = ["explorer_brain.py", "explorer.py", "belief_maze.py",
             "scan_to_walls.py", "run_metrics.py"]

    for name in blind:
        with open(os.path.join(pkg, name)) as f:
            source = f.read()
        code = "\n".join(ln for ln in source.splitlines()
                         if not ln.strip().startswith("#"))
        for forbidden in ("read_truth", "truth_path", "TRUTH_NAME",
                          '["walls"]', "maze_truth"):
            assert forbidden not in code, \
                "%s reaches for ground truth via %s" % (name, forbidden)


def test_generate_maze_writes_both_files():
    from micromouse import generate_maze

    out = tempfile.mkdtemp()
    generate_maze.main(["--size", "8", "--seed", "3", "--out-dir", out])

    public_path = os.path.join(out, spec.PUBLIC_NAME)
    truth_path = os.path.join(out, spec.TRUTH_NAME)
    assert os.path.exists(public_path)
    assert os.path.exists(truth_path)
    assert os.path.exists(os.path.join(out, "maze.num"))
    assert os.path.exists(os.path.join(out, "maze.world"))

    with open(public_path) as f:
        assert "walls" not in json.load(f)
    back, _ = spec.read_truth(truth_path)
    assert back.n == 8
