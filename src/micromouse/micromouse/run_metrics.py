"""What the run cost, measured from inside the robot.

Everything here comes from the robot's own belief and its own trail. The
optimality gap -- how the speed run compares to the true shortest path -- is
deliberately NOT computed here, because that needs the maze, and the brain is
not allowed to see it. The recorder, which may read the truth file, works that
out afterwards.
"""
from . import explorer as E

PHASES = (E.SEARCH, E.RETURN, E.SPEED)


class RunMetrics:
    def __init__(self):
        self.trail = {phase: [] for phase in PHASES}
        self.moves = {phase: 0 for phase in PHASES}
        self.walls_known = 0
        self.cells_seen = set()

    def note(self, phase, cell, belief):
        """Record the robot standing at `cell` during `phase`."""
        cell = tuple(cell)
        if phase in self.trail:
            self.trail[phase].append(cell)
        self.cells_seen.add(cell)
        self.walls_known = belief.known_count()

    def record_move(self, phase):
        """Count one completed cell move.

        Counted explicitly rather than derived from the trail: the cell where a
        phase ends is where the next phase begins, so trail lengths are
        ambiguous at exactly the boundary that matters.
        """
        if phase in self.moves:
            self.moves[phase] += 1

    def steps(self, phase) -> int:
        """Cell moves made during a phase."""
        return self.moves[phase]

    def as_dict(self) -> dict:
        return {
            "search_steps": self.steps(E.SEARCH),
            "return_steps": self.steps(E.RETURN),
            "speed_steps": self.steps(E.SPEED),
            "cells_visited": len(self.cells_seen),
            "walls_discovered": self.walls_known,
        }

    def summary(self, ex=None) -> str:
        d = self.as_dict()
        lines = [
            "[explorer] run complete"
            + ("" if ex is None else " (phase=%s)" % ex.phase),
            "[explorer]   search run : %d cells" % d["search_steps"],
            "[explorer]   return trip: %d cells" % d["return_steps"],
            "[explorer]   speed run  : %d cells" % d["speed_steps"],
            "[explorer]   cells visited   : %d" % d["cells_visited"],
            "[explorer]   walls discovered: %d" % d["walls_discovered"],
        ]
        search, speed = d["search_steps"], d["speed_steps"]
        if search and speed:
            lines.append("[explorer]   speed run is %.0f%% of the search run"
                         % (100.0 * speed / search))
        return "\n".join(lines)


def optimality_gap(speed_steps, true_optimal):
    """How much longer the speed run was than the best possible route.

    Deliberately a free function taking the answer as an argument, rather than a
    method: computing it needs the real maze, and the brain must not hold that.
    The recorder, which may read the truth file, calls this afterwards.
    """
    if not true_optimal:
        return None
    return (speed_steps - true_optimal) / float(true_optimal)


def describe_gap(speed_steps, true_optimal) -> str:
    gap = optimality_gap(speed_steps, true_optimal)
    if gap is None:
        return "speed run %d cells (no optimum to compare against)" % speed_steps
    if gap == 0:
        return ("speed run %d cells -- optimal (true shortest path is %d)"
                % (speed_steps, true_optimal))
    return ("speed run %d cells vs optimal %d -- %.1f%% longer"
            % (speed_steps, true_optimal, 100.0 * gap))
