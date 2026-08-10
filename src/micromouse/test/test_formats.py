import os
import tempfile
import xml.etree.ElementTree as ET

from micromouse import maze as M, mms_format, spec, sdf_builder


def test_num_format_roundtrip():
    m = M.generate(n=16, seed=5)
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "maze.num")
    mms_format.write(path, m)
    lines = [ln for ln in open(path) if ln.strip()]
    assert len(lines) == 256
    back = M.Maze(16)
    for ln in lines:
        x, y, n, e, s, w = (int(t) for t in ln.split())
        back.walls[(x, y)] = {"N": bool(n), "E": bool(e),
                              "S": bool(s), "W": bool(w)}
    assert all(back.walls[c] == m.walls[c] for c in m.walls)


def test_spec_roundtrip():
    m = M.generate(n=16, seed=6)
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, "spec.json")
    spec.write(path, m, 0.30, 0.02, 0.15)
    m2, data = spec.read(path)
    assert data["cell_size_m"] == 0.30
    assert all(m2.walls[c] == m.walls[c] for c in m.walls)


def test_world_sdf_is_well_formed():
    m = M.generate(n=16, seed=7)
    world = sdf_builder.build_world(m, 0.30, 0.02, 0.15)
    root = ET.fromstring(world)            # raises if malformed
    assert root.tag == "sdf"
    assert world.count('<collision name="col_') > 100


def test_the_sensors_system_is_opt_in():
    """It initialises the Ogre render engine, which kills the world on a
    GPU-less host, so the default world must not carry it."""
    m = M.generate(n=4, seed=7)
    plain = sdf_builder.build_world(m, 0.30, 0.02, 0.15)
    withsensors = sdf_builder.build_world(m, 0.30, 0.02, 0.15, sensors=True)

    assert "gz-sim-sensors-system" not in plain
    assert "gz-sim-sensors-system" in withsensors
    assert "<render_engine>ogre2</render_engine>" in withsensors
    ET.fromstring(withsensors)             # still well formed
