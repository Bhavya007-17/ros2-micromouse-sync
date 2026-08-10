"""The exploration policy: search the maze, come home, then run it fast.

Three phases, the way a competition micromouse does it:

  SEARCH -- flood-fill the *optimistic* belief (unsensed walls assumed open) and
            walk down the gradient. Optimism is what makes this explore: the
            unknown always looks like the shortest way to the goal, so the robot
            is pulled into it, and every scan that finds a wall re-routes it.
  RETURN -- same idea, flooded toward the start instead. The trip home is not
            wasted; it keeps sensing.
  SPEED  -- flood the *pessimistic* belief (unsensed walls assumed solid) so the
            route only uses passages actually verified. If the goal is not
            reachable that way, the robot has not learned enough yet, so it
            drops back to SEARCH rather than guessing.

No ROS, no mms, no simulator -- this runs against any sensor that can answer
"wall in front / left / right", which is what makes the whole policy testable
offline.
"""
from . import conventions as C
from . import flood_fill as FF
from . import planner as P
from . import scan_to_walls as S
from .belief_maze import BeliefMaze

SEARCH, RETURN, SPEED, DONE = "search", "return", "speed", "done"

# Safety net for the SPEED -> SEARCH fallback. Each cycle discovers strictly
# more of the maze, so this is never reached in practice; it exists so a bug
# cannot turn into an infinite loop on a robot.
MAX_FALLBACKS = 16


class Explorer:
    """Drives one full run. Holds the belief; the belief itself is immutable."""

    def __init__(self, n, start, goals):
        self.n = n
        self.start = tuple(start)
        self.goals = {tuple(g) for g in goals}
        self.belief = BeliefMaze(n).visit(self.start)
        self.phase = SEARCH
        self.fallbacks = 0

    # -- sensing -------------------------------------------------------------
    def observe(self, cell, heading, relative_walls):
        """Fold one scan into the belief. `relative_walls` is what
        `scan_to_walls.walls_from_scan` returned."""
        updates = S.to_absolute(relative_walls, cell, heading)
        self.belief = self.belief.learn_many(updates).visit(tuple(cell))

    # -- deciding ------------------------------------------------------------
    def targets(self):
        """Cells the current phase is trying to reach."""
        return {self.start} if self.phase == RETURN else self.goals

    def step(self, cell, heading):
        """Return the direction to move next, or None when there is nowhere
        better to go (goal reached, or the robot is boxed in)."""
        cell = tuple(cell)
        self._advance_phase(cell)
        if self.phase == DONE:
            return None

        if self.phase == SPEED:
            maze = self.belief.pessimistic_view()
            dist = FF.flood_fill_distances(maze, list(self.goals))
            if dist[cell] >= FF.INF:
                # Not enough verified passage to commit to. Go learn more.
                if self.fallbacks >= MAX_FALLBACKS:
                    return None
                self.fallbacks += 1
                self.phase = SEARCH
                return self.step(cell, heading)
            # A speed run commits to the known-best route, so no exploration
            # tie-break here.
            return P.pick_next_dir(maze, dist, cell, heading)

        maze = self.belief.optimistic_view()
        dist = FF.flood_fill_distances(maze, list(self.targets()))
        return P.pick_next_dir(maze, dist, cell, heading,
                               visited=self.belief.visited)

    def _advance_phase(self, cell):
        if self.phase == SEARCH and cell in self.goals:
            self.phase = RETURN
        elif self.phase == RETURN and cell == self.start:
            self.phase = SPEED
        elif self.phase == SPEED and cell in self.goals:
            self.phase = DONE

    # -- reporting -----------------------------------------------------------
    def distances(self):
        """The distance field the robot is currently steering by -- this is
        what gets painted into mms and the viz window."""
        if self.phase == SPEED:
            return FF.flood_fill_distances(self.belief.pessimistic_view(),
                                           list(self.goals))
        return FF.flood_fill_distances(self.belief.optimistic_view(),
                                       list(self.targets()))
