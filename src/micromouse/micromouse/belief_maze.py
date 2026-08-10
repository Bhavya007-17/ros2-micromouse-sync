"""What the robot believes the maze looks like, built up one scan at a time.

A `Maze` wall is a boolean: there or not. A belief wall has three states, because
"I have not looked there yet" is different from "I looked and it was open":

    UNKNOWN -- never sensed
    WALL    -- sensed, blocked
    OPEN    -- sensed, passable

That third state is what makes exploration work. Reading the belief optimistically
(UNKNOWN counts as open) produces a flood-fill gradient that pulls the robot
toward unexplored territory, which is exactly the classic micromouse search.
Reading it pessimistically (UNKNOWN counts as wall) produces a route that only
uses passages the robot has actually verified -- what a speed run needs.

Optimism is a property of the object, not an argument, so `flood_fill` and
`planner` consume a BeliefMaze unchanged. Swap views with `optimistic_view()` /
`pessimistic_view()`.

Updates return a new BeliefMaze; nothing here mutates in place.
"""
from . import conventions as C

UNKNOWN, WALL, OPEN = 0, 1, 2

_ALL_DIRS = (C.N, C.E, C.S, C.W)


class BeliefMaze:
    """Tri-state wall knowledge. Duck-types as `maze.Maze` for the solvers."""

    def __init__(self, n: int, walls=None, visited=frozenset(),
                 optimistic: bool = True):
        self.n = n
        self.optimistic = optimistic
        self.visited = frozenset(visited)
        if walls is None:
            walls = {(x, y): {"N": UNKNOWN, "E": UNKNOWN,
                              "S": UNKNOWN, "W": UNKNOWN}
                     for x in range(n) for y in range(n)}
        self.walls = walls

    # -- queries -------------------------------------------------------------
    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.n and 0 <= y < self.n

    def neighbor(self, x: int, y: int, direction: int):
        dx, dy = C.DELTA[direction]
        return (x + dx, y + dy)

    def state(self, x: int, y: int, direction: int) -> int:
        return self.walls[(x, y)][C.DIR_KEYS[direction]]

    def has_wall(self, x: int, y: int, direction: int) -> bool:
        """True only for a *confirmed* wall. Unknown is not a wall."""
        return self.state(x, y, direction) == WALL

    def open_between(self, x: int, y: int, direction: int) -> bool:
        """True if the robot believes it can move from (x,y) in `direction`.

        The perimeter needs no sensing -- you cannot leave the grid -- so the
        bounds check does that job for free, exactly as `Maze.open_between` does.
        """
        nx, ny = self.neighbor(x, y, direction)
        if not self.in_bounds(nx, ny):
            return False
        st = self.state(x, y, direction)
        if st == WALL:
            return False
        if st == OPEN:
            return True
        return self.optimistic

    def known_count(self) -> int:
        """Number of distinct physical walls sensed (each counted once)."""
        seen = 0
        for x in range(self.n):
            for y in range(self.n):
                # Count the N and E side of every cell plus the S/W perimeter,
                # which is exactly one visit per physical wall.
                for d in (C.N, C.E):
                    if self.state(x, y, d) != UNKNOWN:
                        seen += 1
                if y == 0 and self.state(x, y, C.S) != UNKNOWN:
                    seen += 1
                if x == 0 and self.state(x, y, C.W) != UNKNOWN:
                    seen += 1
        return seen

    # -- updates (all return a new BeliefMaze) --------------------------------
    def _replace(self, walls=None, visited=None, optimistic=None):
        return BeliefMaze(
            self.n,
            walls=self.walls if walls is None else walls,
            visited=self.visited if visited is None else visited,
            optimistic=self.optimistic if optimistic is None else optimistic)

    def learn_many(self, updates) -> "BeliefMaze":
        """Apply `(x, y, direction, is_wall)` observations, keeping reciprocity.

        Out-of-grid observations are dropped. A later observation overrides an
        earlier one: the maze is static, so this only matters under sensor noise,
        where trusting the most recent read stops one bad sample being permanent.

        Copy-on-write: only the cells actually touched are copied, and nothing
        mutates an existing BeliefMaze.
        """
        walls = None
        for x, y, direction, is_wall in updates:
            if not self.in_bounds(x, y):
                continue
            nx, ny = self.neighbor(x, y, direction)
            value = WALL if is_wall else OPEN
            if walls is None:
                walls = dict(self.walls)
            walls[(x, y)] = dict(walls[(x, y)])
            walls[(x, y)][C.DIR_KEYS[direction]] = value
            if self.in_bounds(nx, ny):
                walls[(nx, ny)] = dict(walls[(nx, ny)])
                walls[(nx, ny)][C.DIR_KEYS[C.opposite(direction)]] = value
        if walls is None:
            return self
        return self._replace(walls=walls)

    def visit(self, cell) -> "BeliefMaze":
        if cell in self.visited:
            return self
        return self._replace(visited=self.visited | {cell})

    # -- views ---------------------------------------------------------------
    def optimistic_view(self) -> "BeliefMaze":
        """Unknown = open. Use while searching."""
        return self if self.optimistic else self._replace(optimistic=True)

    def pessimistic_view(self) -> "BeliefMaze":
        """Unknown = wall. Use to commit to a speed run."""
        return self._replace(optimistic=False) if self.optimistic else self
