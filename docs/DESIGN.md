# Design — Micromouse ROS 2 stack

## Problem statement

The first version of this stack solved the maze from a god's-eye view: the brain
read every wall from disk, flooded once, and fired the whole path in a burst.
The robot faithfully executed a known solution. It discovered nothing.

The goal now is to take the map away:

1. Solve a maze the brain has never seen, using only what it can sense.
2. Feel out walls one cell at a time with a LiDAR; track position with odometry.
3. Exploit the learned map on a speed run.
4. Still run on WSL without a discrete GPU.

## Solution overview

The maze definition is deliberately **split in two**, so that "the brain cannot
see the maze" is a property of the architecture rather than a promise:

| File | Contains | Consumer |
|------|----------|----------|
| `maze_spec.json` | size, cell geometry, start, goal — **no walls** | `explorer_brain`, `cell_motion_controller` |
| `maze_truth.json` | the walls | `raycast_lidar`, `brain_viz`, recorder, baseline brain |
| `maze.num` | walls, mms format | mms GUI |
| `maze.world` | walls as physical boxes | Gazebo Harmonic |

`spec.read_public()` raises on a file containing walls, so wiring a blind
component to the truth file fails loudly instead of quietly handing it the
answer. A test also scans the source of every module that must stay blind.

The brain (`gazebo_sync_brain.py`) is launched by mms as a subprocess. Its **stdout** is the mms command channel; diagnostics go to **stderr** only.

## Protocol bridge

The brain publishes ROS commands in parallel with mms animation:

```
mms.move_forward()  ─┬─► mms GUI animates the virtual mouse
                     └─► bridge.send("moveForward") → /maze/mms_command
```

The controller acknowledges each finished move on `/maze/step_complete`. The brain waits only at the **end** so mms keeps the process alive until Gazebo drains the queue.

## The belief map

`BeliefMaze` gives every wall three states instead of two: `UNKNOWN`, `WALL`,
`OPEN`. "Not looked yet" is different from "looked and it was open", and that
third state is what makes exploration work.

- **Optimistic** read (`UNKNOWN` = open) — the unknown always looks like the
  shortest way to the goal, so the flood-fill gradient pulls the robot into it.
- **Pessimistic** read (`UNKNOWN` = wall) — only verified passages count, which
  is what a speed run must commit to.

Optimism is a property of the object, not an argument, so `flood_fill` and
`planner` consume a `BeliefMaze` **unchanged** — they were already written
against `.n` / `.open_between` / `.neighbor`. The only change to `planner` was an
optional `prefer_unvisited` tie-break, which costs nothing because it only ever
breaks ties between equal-distance neighbours.

Updates return a new `BeliefMaze` (copy-on-write of just the touched cells).

## Phases

| Phase | Map | Target | Ends when |
|---|---|---|---|
| SEARCH | optimistic | goal | a goal cell is reached |
| RETURN | optimistic | start | back at the start, still sensing |
| SPEED  | pessimistic | goal | goal reached |

If the goal is unreachable on the pessimistic map, the robot has not learned
enough to commit, so it drops back to SEARCH rather than guessing. A fallback
counter bounds that loop.

## Lockstep — and why the old architecture had to invert

The baseline brain could run ahead of the robot because the answer was already
on disk. The explorer cannot know move *n+1* until the robot has arrived at cell
*n* and taken a reading there:

```
send one command → wait /maze/step_complete → wait for a settled scan
→ learn → re-flood → decide
```

**Waiting for the *next* scan is not sufficient, and this corrupts the map.**
The LiDAR publishes on its own clock, so a scan can be sent after arrival but
computed from the pose the robot had a moment earlier; the brain then files the
previous cell's walls against the current cell. The symptom is a belief that
looks plausible and is quietly wrong.

`settled_reading()` instead waits until *consecutive* scans agree on the walls.
The robot is stationary while it asks, so once the pose has propagated every
scan reads the same, and the in-flight stale one is rejected — without the
sensor having to report which pose it used, which matters because Gazebo's
`gpu_lidar` cannot.

## Motion controller

Two backends, chosen with `motion:=`.

**`velocity` (default)** — publishes `/cmd_vel`, reads back `/odom`, turns until
the heading error is inside tolerance, then drives until the wheels say a cell
pitch has passed. Position is earned, not asserted.

**`pose`** — the original `set_pose` teleport. Exact and needs no physics, but
the robot is being *placed*, not driven, so it demonstrates nothing about
localisation. Kept for recording on a slow host.

Commands: `moveForward`, `turnLeft`, `turnRight`, `resetToStart`. A FIFO worker
executes one at a time, so commands cannot be dropped or reordered.

### Three things that had to be right for closed-loop driving

All three were found by running it, and each produced a *downstream* symptom
rather than an obvious failure:

1. **Odom is not the world frame.** gz diff-drive anchors `/odom` at the robot's
   spawn — origin `(0,0)`, yaw `0`. The maze, the targets and the LiDAR speak
   world coordinates. The two differed by the start-cell offset *and* a 90°
   rotation, so every range was cast from the wrong place. It surfaced much
   later as a belief that had **walled off the start cell** (`dist = INF` home).
   Fixed with an explicit odom→world anchor, re-pinned after `resetToStart`,
   because the teleport moves the robot but not the wheels and `/odom` does not
   jump with it.

2. **Absolute targets, never chained.** `rotate_to(current_yaw + 90°)` lets every
   turn keep its own error. After a few moves the robot points far enough off
   that the LiDAR measures the neighbouring cell's walls. Every target is now
   recomputed from the ideal grid.

3. **Lateral drift is real.** Holding a heading *by turning mid-drive* curves the
   path, so a robot on a perfect cardinal still ends up off the centre line.
   `_correct_lateral()` measures the offset as half the difference between the
   two side walls and slides back. A differential drive cannot strafe, so this
   costs two turns — only paid when the offset exceeds threshold.

### Honest limitation of the raycast sensor

With `lidar:=raycast`, the simulated LiDAR casts from the *same* pose the
controller navigates by. Odometry error against the world is therefore invisible
to the sensor, and the re-centering — while real, and genuinely correcting
against walls — is not proving drift rejection. That only becomes a true
localisation correction under `lidar:=gz`, where the sensor is physically on the
robot and sees the world regardless of what odometry thinks.

## Visualization

`brain_viz` is a lightweight pygame window subscribed to `/maze/current_cell`. It does not render 3D; it shows the same distance field the brain computed, plus the live trail.

## Recording pipeline

`scripts/record_full_session.sh`:

1. Kills stale Gazebo/controller processes.
2. Regenerates maze with `--top-down-cam` for square overhead Gazebo view.
3. Launches simulation, runs `headless_mms_host.py | gazebo_sync_brain.py`.
4. `record_session_gifs.py` collects the full ordered trail, then renders brain + overhead GIFs.
5. `render_solve_gif.py` produces the offline mms animation for the README.

### Why the trail topic is `TRANSIENT_LOCAL`

The brain fires the whole 64-move path onto `/maze/current_cell` in a **sub-second
burst**. A freshly started recorder using the default `RELIABLE`+`VOLATILE` QoS
matches the brand-new publisher *after* the burst has already gone out, so every
message is dropped and the GIF is a single static start-cell frame.

Two changes make recording deterministic regardless of process start order:

- **`TRANSIENT_LOCAL` durability + deep history (1024)** on every `current_cell`
  publisher (brain and controller) and on the recorder's subscription. A late
  subscriber *replays* the entire buffered trail.
- The recorder **collects first, renders second**. Rendering is done offline
  (one frame per cell) so matplotlib never starves the rclpy executor, and the
  animation length is decoupled from how fast cells arrive over the wire.

The brain also waits for `pub.get_subscription_count() > 0` (command + cell
topics) before sending, closing the same first-message race on the command path.

## The Gazebo 3D viewport on WSL / GPU-less hosts

On WSL without a discrete GPU, the Gazebo Harmonic **GUI** logs:

```
Failed to load plugin [] : couldn't load library on path
  [/opt/ros/jazzy/opt/gz_rendering_vendor/lib/]
```

This is the Ogre 3D **render-engine** failing to initialise — it is a viewport
limitation, **not** a solver fault. The stack is intentionally **pose-driven**:
the physics server, `set_pose` service, motion controller, flood-fill brain, and
the synthetic overhead recorder all work without any 3D rendering. The robot
still traverses the maze; the demo GIFs are drawn from ROS topics, not from the
3D camera. If you need the 3D window, run on a host with OpenGL/GPU acceleration
or set `LIBGL_ALWAYS_SOFTWARE=1` for slow software rendering.

## Failure modes

| Symptom | Likely cause |
|---------|----------------|
| `stuck ... dist=1000000000` | Belief has enclosed the target — walls learned against the wrong cell. Check the odom→world anchor and that turns are completing. |
| Explorer stops with `no scan` | `raycast_lidar` is not running, or `maze_truth.json` is missing (it fails loudly by design). |
| `lidar:=gz` gives no `/scan` | World was not generated with `--sensors`, or the host has no GPU render engine. Use the default `lidar:=raycast`. |
| Belief looks plausible but wrong | Scans read before the pose propagated — `settled_reading` exists for exactly this. |
| Robot stuck mid-maze | Second `ros2 launch` still running — kill with pkill one-liner |
| mms "No such file" for brain | Run command must use `run_brain.sh` (sources ROS + workspace) |
| Robot not at start on Run | Missing `resetToStart` or duplicate controllers fighting set_pose |
| Maze/spec mismatch crash | Regenerate maze; reload `.num` in mms |
| `robot did 0/N moves` / empty GIF | First-message QoS race — fixed by `TRANSIENT_LOCAL` trail + subscriber wait |
| Gazebo GUI `engine []` / blank 3D view | No GPU render engine on WSL — cosmetic; solve + GIFs are unaffected |
