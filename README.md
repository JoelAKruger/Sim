# Regolith

A rover simulator for Nova's analogue lunar rover, built on Raylib and Box3D. It speaks
ROS 2 Jazzy and SocketCAN natively. It is one small native program that starts
in well under a second.

## Build and run

nixpkgs, nix-ros-overlay and Box3D are pinned in `flake.lock`. nixpkgs follows the
overlay's own pin, so ROS packages come prebuilt from `ros.cachix.org`.
`nix flake update` moves the pins forward.

```sh
nix build                           # builds; runs tests, style and naming checks → ./result
./result/bin/regolith               # window: M control mode, R reset, arrows drive, RMB orbit, wheel zoom
nix build .#without-ros             # the same sim with no ROS 2 at all
```

`nix-build` and `nix-shell` also work, reading the same pins from `flake.lock`
(`nix-build --arg withRos false` for the no-ROS build).

For development:

```sh
nix develop                         # or nix-shell
cmake -B build -G Ninja && cmake --build build
ctest --test-dir build
./build/regolith --config config/sim.yaml
```

## Settings

Every setting is declared once, in the schema table in `core/config.cpp`, with its key,
range, default and description. Settings are applied in this order:

1. built-in defaults
2. `--config FILE`: `config/sim.yaml` shows every setting
3. ROS 2 parameters (`--ros-args --params-file FILE`, `-p key:=value`), if built with ROS 2

The same YAML file works with `--config` and with `--params-file`. An unknown or misspelt
key is reported with its file and line; an out-of-range value is an error. `--dump-config`
prints the effective settings, and with ROS 2, `ros2 param dump /regolith` shows the same.

## Useful flags

| Flag | Effect |
|---|---|
| `--config F` | load settings from F |
| `--urdf F` | load this robot (overrides `robot.urdf`) |
| `--dump-config` | print the effective settings as YAML and exit |
| `--headless` | no window |
| `--rtf X` | real-time factor; `0` runs as fast as possible |
| `--steps N` | quit after N physics steps |
| `--hash` | print the final state hash; identical inputs give identical hashes |
| `--screenshot F` | save the final frame as a PNG (with `--steps`) |

The sim publishes `/clock`, so run other nodes with `use_sim_time:=true`.

## Robots

The robot is a URDF, taken from `/robot_description` (what `robot_state_publisher`
latches) or from a file (`--urdf`, or `robot.urdf`). The launch file does the usual thing:

```sh
ros2 launch regolith sim.launch.py description:=/path/to/rover.urdf.xacro   # headless:=true, config:=F
```

The robot is loaded once, at startup. Without a file, the sim waits for
`/robot_description` before it starts stepping, and ignores later descriptions. To change
the robot, restart the sim. A description that fails to load stops the sim with the
reason.

- Fixed joints are merged into single bodies.
- Motors are near-ideal: every driven joint can apply `robot.motor_torque` (10000 N·m by
  default). The URDF's effort and velocity limits are ignored; its position limits apply.
- Masses come from the URDF inertials; implausible ones are reported and repaired.
- Meshes must be STL (`file://`, `package://` or relative). A visual that can't be
  loaded is drawn as its link's collision shapes instead.

Exactly one source drives the robot, chosen with the CAN / KEYBOARD switch in the control panel (or **M**):

- **CAN** (green): the BLCMD commands on the bus, below. This is the default when the CAN
  interface is up. The status line under the switch is red if it isn't.
- **Keyboard** (orange): the arrow keys.
  - **Up/down** drive at `teleop.speed_mps`.
  - **Left/right on their own** spin the rover on the spot at `teleop.turn_rate`.
  - **With up or down held,** it follows an arc, and the steering points the same way
    forwards and backwards, as a car's does. Reversing with left held swings the nose right.
  - **Speed limit:** no wheel is ever driven faster than `teleop.speed_mps` over the ground,
    so an arc slows the rover rather than overspeeding its outer wheels.
  - **Wheels:** steered wheels point along the velocity they need; unsteered wheels
    skid-steer.
  - With no key held the robot stops.

  In this mode CAN commands are thrown away, and the panel says "Ignoring CAN commands"
  while they arrive. This is the default without CAN.

**M** switches between them. After a switch every joint holds until the new source
commands it; CAN commands from before the switch are forgotten.

**R** resets the robot if it flips or gets stuck. It goes back to its spawn pose, upright and
at rest with every joint at zero, and holds there the same way until it is commanded again.
The control mode doesn't change.

Position commands (the steering pivots) move the joint towards its target, clamped to its
URDF limits, at `robot.servo_speed` (2 rad/s by default; 0 makes it instant). It arrives
exactly, without overshoot. This applies to CAN and keyboard alike. Set it to the real
pivots' speed.

In keyboard mode a wheel whose pivot is still swinging drives only the part of its speed
along the way it points. So switching between straight and turning swings the pivots
first, instead of dragging the wheels sideways.

## Motor controllers and LEDs (CAN)

The sim plays the rover's BLCMD motor controllers and LED strip on a SocketCAN interface
(`can.interface`, default `can0` as in the URDF), so the drive stack talks to it as it does
to the hardware. Every URDF joint with a `<ros2_control>` block that gives a `canid` is a
BLCMD. Banksia has eight: wheels `flw`, `blw`, `brw`, `frw` are 1–4, and pivots `flp`,
`blp`, `brp`, `frp` are 5–8. The protocol is the one the previous (Unity) sim implemented.

- **Id:** `node << 4 | function`. The value is a big-endian 16-bit number in bytes 0–1.
- **Drive at Speed (3):** `rad/s = 30 · value / 32767` at the joint.
- **Drive to Position (4):** `rad = π · (value − zero_offset) / 65535`, with the value signed.
  `zero_offset` comes from the URDF (Banksia: 14717 = `0x397D`).
- **Stop (0):** ignored, as the Unity sim ignored it; the last command stands.
- **`reversed=true` in the URDF** negates a joint's values. Banksia's left motors are
  reversed, so for the rover to go straight their values on the wire are negated.
- **Not simulated:** the other functions (twitch, current, open loop, homing,
  configuration, reset) are reported once and ignored.
- **Nothing is sent back,** as with the previous sim.
- **LED strip (node 9, `can/led_strip.*`):** a separate device with its own protocol:
  brightness, red/green/blue, presets, all-in-one, pink and reset, handled exactly as the
  Unity sim did. It shows as the status light swatch in the control panel.
- **Commands persist** until replaced, as they did in the previous sim. Set
  `robot.command_timeout_ms` to make them expire (a watchdog in sim time).

```sh
sudo modprobe vcan && sudo ip link add dev can0 type vcan && sudo ip link set up can0
./result/bin/regolith --urdf tests/fixtures/banksia.urdf &
cansend can0 033#1000        # brw: 3.75 rad/s
cansend can0 013#F000        # flw (reversed): 3.75 rad/s forwards
cansend can0 084#51EF        # frp: 0.3 rad
cansend can0 095#02          # LED strip: green
```

## Sensors

Three sensors, published the way the rover's own drivers publish them. Each one is off
until the config turns it on, and each sits on a URDF link:
- **LiDAR and IMU:** the link named by `frame`, which is also their messages' frame_id.
- **Camera:** `<name>_link`, as the RealSense description names it.

If the link isn't in the URDF, the sim stops at startup and names the setting.

```yaml
lidar:
  enabled: true        # frame defaults to livox_frame
imu:
  enabled: true        # the Mid-360's own IMU, also on livox_frame
camera:
  name: "d415"         # realsense2_camera camera_name: mounted on d415_link
  color:
    enabled: true
  depth:
    enabled: true
```

- **LiDAR** (Livox Mid-360): `sensor_msgs/PointCloud2` on `/livox/lidar`, in
  `livox_ros_driver2`'s `xfer_format 0` layout.
  - **Fields:** `x y z intensity` (float32), `tag line` (uint8) and `timestamp` (float64,
    absolute ns), packed into 26 bytes. The header stamp is the first point's time.
  - **Timing:** it scans continuously at 200,000 rays per second and publishes 10 frames
    per second.
  - **Coverage:** 360° around, −7° to 52° up.
  - **Returns:** a hit between 0.1 and 40 m, with 2 cm of range noise.
  - **Pattern:** the Mid-360's rosette isn't published, so the rays follow a
    low-discrepancy sequence instead. It never repeats, and each frame covers the field of
    view evenly.
  - **Speed:** rays are cast on `lidar.threads` threads, 0.3 ms a step with 4. The output
    is the same whatever the thread count.
- **IMU** (the Mid-360's): `sensor_msgs/Imu` on `/livox/imu` at 200 Hz, without
  orientation.
  - **Units:** acceleration is **in g**, as the Livox driver sends it. Set
    `imu.acceleration_in_g: false` for m/s².
  - **What it measures:** the body's angular velocity, and the specific force at the
    sensor's own point.
  - **Noise:** white noise plus random-walk bias, using ICM-40609 figures by default.
- **RGB-D camera** (generic RealSense), on the realsense2_camera 4.x topics:
  - `rgb8` colour on `/camera/camera/color/image_raw`.
  - `16UC1` depth in mm on `/camera/camera/depth/image_rect_raw`, 0 where there is no
    data.
  - A `CameraInfo` for each, on the sibling `camera_info` topic.

  **Settings:** colour and depth are separate streams, as the driver's colour and depth
  profiles are. Each has its own section (`camera.color`, `camera.depth`) with:
  - `enabled`, `resolution`, `rate_hz`, `horizontal_fov_deg` and `topic`;
  - for colour, its sensor offset `offset_m`;
  - for depth, its `range_m`.

  They render independently, so they can differ in size and rate. The camera is on when
  either stream is.

  **Frames:** the sim works like `realsense2_camera`.
  - **Mount:** the camera sits on `<name>_link` (x forward, z up).
  - **Optical frames:** depth is at the link's origin and colour at `camera.color.offset_m` (15 mm
    to the left on a D4xx), each turned to z forward, x right, y down.
  - **Stamps:** the images carry `<name>_color_optical_frame` and
    `<name>_depth_optical_frame`.
  - **TF:** the driver's four static transforms go on `/tf_static`: `<name>_link` →
    `<name>_depth_frame` → `<name>_depth_optical_frame`, and likewise for colour.

  Nothing else goes on TF.

  **Window:** the camera renders with the window's GL context, so it is off in
  `--headless`.

Noise comes from `sensors.seed`, so a run repeats exactly. Sensors only read the physics,
so the state hash is the same with them on or off.

In the window, the LiDAR points switch (or **L**) shows the last frame as points coloured
by height. While the camera is on, its colour image appears in a card at the bottom
right.

## Terrain

Terrain is procedural by default: hills plus craters, set by `terrain.seed`. For a real
site, set `terrain.heightmap` to a binary PGM:

- north up, with one sample every `terrain.spacing_m` metres
- the full pixel range spans `terrain.height_m`
- use 16 bits: an 8-bit map loads with a warning, because its height steps are big
  enough to make wheels chatter

A DEM converts with, for example, `gdal_translate -of PNM -ot UInt16 -scale site.tif site.pgm`.

**Boulders:** `terrain.boulders` rocks (400 by default; 0 for none) are scattered across the
terrain and fixed in the ground.
- **Size:** `terrain.boulder_size_m` (0.1 to 1.2 m); most are small.
- **Shape:** irregular convex rocks, partly buried, only where the ground is level enough
  for them.
- **Spawn area:** the 6 m around `robot.spawn` is kept clear.
- **What they are:** static Box3D hulls, so the rover collides with them and the LiDAR and
  camera see them.
- **Layout:** it comes from `terrain.seed`, the same every run.

## Layout and modularity

- `core/`: the simulation: world, terrain, settings, and the lock-free shared state between
  threads. `core/sensors/` holds the IMU and LiDAR models and the camera's pinhole
  model. It has no ROS, no Raylib and no sockets, and is the only code that calls
  Box3D.
- `render/`: the Raylib viewer, and the sensor camera (`sensor_camera.cpp`), which draws the
  same scene.
- `ros/`: the optional ROS 2 adapter. `ros_bridge.cpp` is the only code that includes
  rclcpp. With `-DREGOLITH_WITH_ROS=OFF`, `ros_disabled.cpp` supplies the same functions
  as no-ops.
- `can/`: the optional SocketCAN adapter. `can_bridge.cpp` owns the socket and its
  thread; with `-DREGOLITH_WITH_CAN=OFF`, `can_disabled.cpp` replaces it.
  Each device's wire format is its own file (`blcmd.cpp`, `led_strip.cpp`), always built
  and tested.
- `app/main.cpp`: the fixed-step loop and thread wiring. It is the same code in every
  build configuration.
- `tests/`: shared-state concurrency, settings, terrain/physics agreement, actuators, the
  CAN codec, sensors, determinism.

Adapters talk to the core only through `core/shared_state.h` and plain structs, and they are
chosen at link time. `nix build .#minimal` has neither adapter. The physics doesn't
depend on which adapters are built: every build gives bit-identical state hashes.

Naming: `verb_noun` functions (`step_world`, `get_sim_time`), `Upper_Snake` structs,
`lower_case` variables, and `u32`/`f32`/`v3` types. `tools/check_naming.sh` and
`tools/check_format.sh` (layout, from `.clang-format`) check this in every Nix build.

## The viewer

The window has a control panel on the left, a status bar along the top and every shortcut
along the bottom. The 3D view fills the rest.

- **Control:** a CAN / KEYBOARD switch (or **M**). Below it is what the selected source
  is doing: listening on the CAN interface, no interface, or ignoring CAN commands while
  the keyboard drives. The LED strip's colour is shown here too.
- **Simulation:** Pause / Resume (**P**), Step (**N**, while paused) and Reset robot (**R**).
  **Space** still drops a large test box at the view centre.
- **View:** switches for following the robot (**F**), collision shapes (**C**), LiDAR
  points (**L**) and the camera preview.
- **Sensors:** each sensor's state, frame and rate.
- **Top bar:** sim time, real-time factor, step cost, frame rate and the rover's speed.

The world starts with four 10 cm cubes, red, white, green and blue, scattered 1.5 to 5 m
from the spawn point. Their layout comes from `terrain.seed`, so it is the same every run.

The mouse orbits (right button), pans (middle button, or WASD) and zooms (wheel) only over
the 3D view, never over the panel.

**Lighting:** a low, hard sun (25° up) with long shadows and only a faint fill, since the
Moon has no atmosphere. The ground and rocks get fine procedural grain, with darker crater
walls. The sky has stars, and the terrain fades to black at its edges. The sensor camera
sees the same lighting and shadows, but no stars, as a real camera exposed for sunlit
ground wouldn't.

**Graphics quality:** `viewer.quality`, or the LOW / MED / HIGH switch in the panel, trades
looks for speed:

| | shadows | surface grain |
|---|---|---|
| LOW | none | none |
| MED | four 1024² cascades, 2×2 smooth taps, rover shadow from its collision shapes | yes |
| HIGH | four 2048² cascades, 3×3 smooth taps, rover shadow from its full meshes | yes |

**Shadow cascades:** four shadow maps cover different areas, and each point uses the
finest one that covers it:
- 12 m around the rover, which keeps the rover's own shadow sharp;
- the near part of the view, out to 0.8 × the orbit distance;
- the rest of the view, out to 3 × the orbit distance;
- 100 m around the rover, so the sensor camera always has shadows.

They follow the rover and the view, snapped to their texels so edges don't shimmer, and
are filtered bilinearly so edges are smooth, not stepped.

Text is JetBrains Mono. The font is copied into the binary at build time
(`REGOLITH_FONT_REGULAR` and `REGOLITH_FONT_BOLD`, which Nix sets), so nothing is loaded
at run time. A build without it falls back to Raylib's font.

## Graphics

The viewer uses the system's GL driver (`/run/opengl-driver`). If the system's nixpkgs
is newer than the nixpkgs pinned in `flake.lock`, that driver can't load (a glibc
mismatch), and Regolith reports that it could not open a window. Keep the system no
newer than the pin, or use `--headless`.

The simulation keeps running while the window is minimised or hidden. Frames are paced at
the monitor's refresh rate by a timer rather than vsync, because on Wayland a vsync'd swap
waits for the compositor, which stops asking a hidden window for frames.
