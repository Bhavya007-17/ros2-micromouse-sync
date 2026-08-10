"""Build a Gazebo (gz-sim / Harmonic) world SDF from a maze.

The world contains the required gz systems, a sun, a ground plane, and the maze
walls as a single static model. The robot is NOT in the world -- it is spawned
separately by the launch file so the world can be reused.

Cells are square: the cell pitch is `cell_size` in BOTH x and y, so the open
interior of every cell is (cell_size - wall_thickness) on each side. If the maze
*looks* rectangular in Gazebo it is the default perspective camera viewed from an
angle -- pass add_gui=True (generate_maze --top-down-cam) for a straight-down
view, or use the GUI's "View Angle -> top" button.

Each physical wall is emitted exactly once:
  * every cell's NORTH wall  -> interior horizontal walls + top border
  * row 0 SOUTH walls        -> bottom border
  * every cell's EAST wall   -> interior vertical walls + right border
  * col 0 WEST walls         -> left border
Segments are (cell_size + wall_thickness) long so they overlap at the posts.
"""
from .maze import Maze


def _box(name, x, y, sx, sy, sz, z):
    pose = "{:.5f} {:.5f} {:.5f} 0 0 0".format(x, y, z)
    size = "{:.5f} {:.5f} {:.5f}".format(sx, sy, sz)
    return """
      <collision name="col_{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="vis_{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>0.25 0.25 0.28 1</ambient>
          <diffuse>0.35 0.35 0.40 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>""".format(name=name, pose=pose, size=size)


def _wall_boxes(maze: Maze, cs: float, wt: float, wh: float):
    z = wh / 2.0
    out = []
    i = 0
    for x in range(maze.n):
        for y in range(maze.n):
            w = maze.walls[(x, y)]
            if w["N"]:
                cx, cy = x * cs + cs / 2.0, (y + 1) * cs
                out.append(_box(i, cx, cy, cs + wt, wt, wh, z)); i += 1
            if w["E"]:
                cx, cy = (x + 1) * cs, y * cs + cs / 2.0
                out.append(_box(i, cx, cy, wt, cs + wt, wh, z)); i += 1
            if y == 0 and w["S"]:
                cx, cy = x * cs + cs / 2.0, 0.0
                out.append(_box(i, cx, cy, cs + wt, wt, wh, z)); i += 1
            if x == 0 and w["W"]:
                cx, cy = 0.0, y * cs + cs / 2.0
                out.append(_box(i, cx, cy, wt, cs + wt, wh, z)); i += 1
    return "".join(out)


def _gui_block(span: float) -> str:
    """A straight-down camera so the square grid reads as squares.
    If this causes GUI trouble on your machine, just delete the <gui> block;
    the maze geometry is unaffected."""
    cx = cy = span / 2.0
    h = span * 1.3
    return """
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <property type="string" key="state">docked</property>
          <property type="bool" key="showTitleBar">false</property>
        </gz-gui>
        <!-- Without an explicit engine the pane reports
             "Engine [] is not supported" and renders nothing. ogre2 works
             under llvmpipe, which is what makes headless capture possible. -->
        <engine>ogre2</engine>
        <scene>scene</scene>
        <camera_pose>{cx:.3f} {cy:.3f} {h:.3f} 0 1.5708 0</camera_pose>
        <background_color>0.85 0.85 0.85</background_color>
      </plugin>
      <!-- Scene Manager only. The interactive-view and camera-tracking
           plugins each claim a docked panel and squeeze the 3D view down to a
           sliver, which matters when the window is being captured. -->
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">floating</property>
          <property type="double" key="width">0</property>
          <property type="double" key="height">0</property>
        </gz-gui>
      </plugin>
    </gui>""".format(cx=cx, cy=cy, h=h)


_SENSORS_PLUGIN = """
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
"""


def build_world(maze: Maze, cell_size: float, wall_thickness: float,
                wall_height: float, world_name: str = "micromouse",
                add_gui: bool = False, sensors: bool = False) -> str:
    """Build the world SDF.

    `sensors` adds the system that drives Gazebo's own gpu_lidar. It is off by
    default because that system initialises the Ogre render engine, which fails
    on a GPU-less host and takes the whole world down with it -- and the default
    setup does not need it, since the range data comes from `raycast_lidar`.
    """
    walls = _wall_boxes(maze, cell_size, wall_thickness, wall_height)
    span = maze.n * cell_size
    gui = _gui_block(span) if add_gui else ""
    sensor_plugin = _SENSORS_PLUGIN if sensors else ""
    return """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world_name}">
{gui}
    <physics name="default" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
{sensor_plugin}

    <gravity>0 0 -9.8</gravity>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>{cx} {cy} 6 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal>
            <size>{big} {big}</size></plane></geometry>
          <surface><friction><ode>
            <mu>1.0</mu><mu2>1.0</mu2>
          </ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal>
            <size>{big} {big}</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="maze_walls">
      <static>true</static>
      <link name="walls">{walls}
      </link>
    </model>

  </world>
</sdf>
""".format(world_name=world_name, walls=walls, gui=gui,
           sensor_plugin=sensor_plugin,
           big=max(20.0, span * 2.0), cx=span / 2.0, cy=span / 2.0)
