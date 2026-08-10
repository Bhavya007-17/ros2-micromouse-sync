"""Generate a maze and write everything the rest of the system needs.

Outputs (default dir ~/.micromouse):
  maze.num         -> load this in the mms GUI
  maze_spec.json   -> PUBLIC spec (size, geometry, start, goal -- no walls)
  maze_truth.json  -> the walls; for the sim, the viz and the baseline brain
  maze.world       -> Gazebo world with physical walls

The two-file split is what makes "the explorer solves an unseen maze" checkable
rather than a claim. See micromouse/spec.py.

Run:  ros2 run micromouse generate_maze            (or: python3 generate_maze.py)
"""
import argparse
import os
import sys

from . import maze as maze_mod
from . import flood_fill
from . import mms_format
from . import spec as spec_mod
from . import sdf_builder
from . import conventions as C


def main(argv=None):
    p = argparse.ArgumentParser(description="Generate a micromouse maze.")
    p.add_argument("--size", type=int, default=16, help="grid size (even)")
    p.add_argument("--seed", type=int, default=None, help="RNG seed")
    p.add_argument("--cell-size", type=float, default=0.30,
                   help="cell size in meters (Gazebo)")
    p.add_argument("--wall-thickness", type=float, default=0.02)
    p.add_argument("--wall-height", type=float, default=0.15)
    p.add_argument("--out-dir", default=os.path.expanduser("~/.micromouse"))
    p.add_argument("--preview", action="store_true",
                   help="print an ASCII maze + distances")
    p.add_argument("--top-down-cam", action="store_true",
                   help="bake a straight-down GUI camera into the world")
    p.add_argument("--sensors", action="store_true",
                   help="add the gz sensors system so the real gpu_lidar works "
                        "(needs a GPU render engine; use with lidar:=gz)")
    args = p.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    m = maze_mod.generate(n=args.size, seed=args.seed)

    # Sanity: the maze must be valid and solvable from the start.
    goals = C.goal_cells(m.n)
    dist = flood_fill.flood_fill_distances(m, goals)
    start = C.start_cell()
    assert m.perimeter_closed(), "perimeter is not fully enclosed"
    assert m.reciprocal(), "walls are not reciprocal"
    assert dist[start] < flood_fill.INF, "goal is not reachable from start"

    num_path = os.path.join(args.out_dir, "maze.num")
    spec_path = os.path.join(args.out_dir, spec_mod.PUBLIC_NAME)
    truth_path = os.path.join(args.out_dir, spec_mod.TRUTH_NAME)
    world_path = os.path.join(args.out_dir, "maze.world")

    mms_format.write(num_path, m)
    # Two files on purpose: the explorer brain may read the public spec only.
    # See micromouse/spec.py.
    spec_mod.write_public(spec_path, m, args.cell_size, args.wall_thickness,
                          args.wall_height)
    spec_mod.write_truth(truth_path, m, args.cell_size, args.wall_thickness,
                         args.wall_height)
    with open(world_path, "w") as f:
        f.write(sdf_builder.build_world(m, args.cell_size, args.wall_thickness,
                                        args.wall_height,
                                        add_gui=args.top_down_cam,
                                        sensors=args.sensors))

    if args.preview:
        print(maze_mod.ascii_art(m, dist))

    bar = "=" * 68
    print(bar)
    print("Maze generated ({0}x{0}, seed={1}). Start {2} is {3} moves "
          "from the goal.".format(m.n, args.seed, start, dist[start]))
    print(bar)
    print("Files written:")
    print("  mms maze file : {}".format(num_path))
    print("  public spec   : {}  (no walls -- what the explorer may read)"
          .format(spec_path))
    print("  maze truth    : {}  (walls -- sim + viz only)".format(truth_path))
    print("  gazebo world  : {}".format(world_path))
    print(bar)
    print("Next:")
    print("  1) ros2 launch micromouse maze_gazebo.launch.py")
    print("  2) Open the mms GUI. Set Maze -> {}".format(num_path))
    print("     Set Mouse -> Run command:")
    solver = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "gazebo_sync_brain.py")
    print("        python3 {}".format(solver))
    print("     (launch mms from a terminal that has sourced ROS + the ws)")
    print("  3) Click Run. Distances fill in; the Gazebo robot mirrors each "
          "cell.")
    print(bar)
    return 0


if __name__ == "__main__":
    sys.exit(main())
