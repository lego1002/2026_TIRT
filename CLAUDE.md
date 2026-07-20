# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Hardware design files and robot software for a maze-solving robot built for the 2026 TIRT competition
(`2026TIRT-迷宮機器人挑戰賽.pdf`). This is primarily a CAD/URDF repository, not an application codebase —
there is no build system, package manager, or test suite for most of the content. Treat SolidWorks/STEP/STL
files as opaque binary artifacts; only the URDF, launch files, config, and the OminiBotHV Python driver are
human-editable text.

## Repository layout

- `CAD_files/solidworks/` — SolidWorks source parts and assemblies (`.SLDPRT`/`.SLDASM`). `UGV_ass.SLDASM` is
  the top-level robot assembly; `UGV_chasis_ass.SLDASM`/`UGV_chasis.SLDASM` is the chassis sub-assembly.
- `CAD_files/step/` and `CAD_files/3DP/` — neutral STEP exports of individual parts (some, like the N20 motor
  bracket and mecanum connector, are custom 3D-printed parts; others like Lego Technic beams and the Pi/lidar
  are off-the-shelf reference geometry used for fitting).
- `car_assemble_description/` — a ROS 2 (`ament_cmake`) package auto-exported from SolidWorks via the SW2URDF
  exporter (originally ROS 1/catkin as `CAR_ASSEMBLE_URDF`; converted to ament_cmake on 2026-07-13, targeting
  ROS 2 Humble + Gazebo Classic `gazebo_ros`; renamed to the current lowercase name on 2026-07-13 to follow
  ROS 2 package-naming conventions — `_description` is the idiomatic suffix for a URDF/mesh-only package,
  e.g. `turtlebot3_description`). Do not hand-edit `meshes/*.STL` or regenerate them manually — they come
  from a SolidWorks export (`export.log` records the exporter run) and are tracked via **Git LFS**
  (`.gitattributes` covers `*.STL`/`*.STEP`/`*.SLDPRT`/`*.SLDASM`) — `git-lfs` must be installed before
  cloning or the checked-out mesh/CAD files will just be small LFS pointer stubs, not real geometry. The
  `.urdf` and config/launch files are safe to hand-edit.
  - `urdf/CAR_ASSEMBLE_URDF.urdf` — the robot description: `base_link` (fixed) with `lidar_link` (fixed) and
    four `continuous` wheel joints (`front_left_wheel_joint`, `front_right_wheel_joint`,
    `rear_left_wheel_joint`, `rear_right_wheel_joint`), each rotating about the Y axis (mecanum wheels). The
    file's own name and the `<robot name="CAR_ASSEMBLE_URDF">` model name were deliberately left as-is during
    the package rename — only the `package://car_assemble_description/meshes/...` URIs inside it (and in the
    sibling `.csv`) were updated, since those are what actually have to match the package name for RViz/Gazebo
    to resolve mesh files.
  - `urdf/CAR_ASSEMBLE_URDF.csv` — the exporter's intermediate per-link/joint data (inertials, origins,
    limits). Useful as a flat reference when cross-checking the URDF, but the `.urdf` is the source of truth.
  - `config/joint_names_CAR_ASSEMBLE_URDF.yaml` — leftover ROS 1 `ros_control` `controller_joint_names`
    format; `ros2_control` uses a different config format entirely, so this needs a rewrite (not a
    conversion) once `ros2_control` is wired up.
  - `launch/display.launch.py` and `launch/gazebo.launch.py` — ROS 2 Python launch files (RViz2 preview with
    `joint_state_publisher_gui`, and Gazebo Classic spawn via `gazebo_ros`/`spawn_entity.py`). See
    `urdf閱讀方法.md`'s "ROS 2 轉換" and "套件改名" sections for the full conversion/rename rationale and what
    was dropped (the ROS 1 `/calibrated` rostopic-pub step has no ROS 2 equivalent and was removed).
  - `launch/robot_bringup.launch.py` — the **real** on-robot entry point (headless, GUI-free), meant to run on
    the Raspberry Pi. It brings up `robot_state_publisher` + a non-GUI `joint_state_publisher` (zeros the four
    wheel joints so their TFs exist) + the RPLidar driver + SLAM Toolbox (async), plus a temporary fake
    `odom->base_link` static TF. Launch args: `use_slam` (default true), `use_fake_odom` (**default
    false** as of 2026-07-14 — the real `ominibot_driver` now runs by default; set `use_fake_odom:=true` for
    hardware-free model/lidar viewing), and `ominibot_port` (default `/dev/serial0`, the Pi GPIO UART).
    The `vx_sign`/`vy_sign`/`wz_sign` launch args default to the same
    hardware-verified signs as `driver_node.py` (`1.0`/`-1.0`/`-1.0`) — keep the two in sync, since the launch
    passes them explicitly and would otherwise override the node defaults. It `IncludeLaunchDescription`s
    `my_robot_lidar`'s `lidar_start.launch.py` and reads that package's `mapper_params_online_async.yaml` —
    so it depends on packages **outside this repo** (see "Runtime deployment" below).
  - `rviz/view_robot.rviz` — saved RViz2 config for the **PC-side viewer** in the two-machine setup (Fixed
    Frame `map`, RobotModel on `/robot_description`, LaserScan `/scan`, Map `/map` with Durability set to
    **Transient Local** to receive the latched map, Odometry with `Keep: 1` and small arrows — the previous
    `Keep: 50` + 0.4 m arrows visually buried the 0.15 m robot; temporarily set `Keep` back up to visualize
    odometry error as a breadcrumb trail when calibrating — and TF). Note this is distinct from
    `display.launch.py`, which still ships no saved config and needs its displays added by hand.
- `OminiBotHV-master/` — vendor (CircusPi) driver package for the OminiBotHV motor/IMU controller board.
  - `example/OminiBot_HV_Meca.py` — reference Python driver (`ominibothv` class) showing the serial protocol:
    frames are `\x7b <cmd> ... <bcc> \x7d` with a big-endian XOR checksum (`calculate_bcc`). Key methods:
    `robot_speed(lx, ly, az)` (mecanum body-frame velocity command), `motor_speed(m1..m4)` (per-wheel), and
    `read_robot_data()` (velocity + IMU quaternion + battery voltage feedback frame).
  - `firmware/` — prebuilt STM32F1 `.hex` firmware for the board (not built from source in this repo).
  - `communication/` — PDF spec for the serial protocol used by the driver above.
- `ominibot_driver/` — the actual ROS 2 (`ament_python`) driver node wrapping the board (written 2026-07-13;
  symlinked into `~/ros2_ws/src/` like `car_assemble_description`). `ominibot_driver/ominibot_hv.py` is a
  self-contained, de-duplicated copy of the vendor protocol class (so the package doesn't depend on the
  `OminiBotHV-master/example` path) — **keep the `time.sleep()` delays in its `__init__`**: without the 0.5s
  after `forced_stop` and 0.1s between config frames the firmware never starts streaming feedback (verified on
  hardware). `driver_node.py` subscribes `/cmd_vel` → `robot_speed(lx,ly,az)` (with a watchdog that zeros the
  base after `cmd_vel_timeout`), and a background read thread dead-reckons `/odom` + broadcasts
  `odom->base_link` TF from the board's body-velocity feedback, and publishes the IMU quaternion on `/imu`
  (accel/gyro layout unverified, so left out). Odom is integrated from velocity (not IMU heading) to keep the
  `odom` frame smooth for slam_toolbox. Default port is `/dev/serial0` — the board's USB (FTDI)
  terminal broke, so as of 2026-07-19 it is wired to the Pi's **GPIO UART** (TX/RX on GPIO14/15,
  pins 8/10 → `ttyAMA0`, of which `/dev/serial0` is the stable alias). The serial *protocol* is
  unchanged (raw UART is exactly what the FTDI used to bridge, same 115200 8N1); only the port
  moved. `udev/99-ominibot.rules` is now a deprecated no-op — a GPIO UART is a built-in platform
  device that USB enumeration can't steal, so no udev rule is needed. This requires `enable_uart=1`
  + `dtoverlay=disable-bt` in `/boot/firmware/config.txt` and no serial console on `ttyAMA0` in
  `cmdline.txt` (both already set on the Pi); the driver user must be in the `dialout` group.
  `driver_node.py` also has `linear_x_sign`/`linear_y_sign`/`angular_z_sign` params to correct the board's
  axis conventions relative to REP-103 (`linear_y_sign` and `angular_z_sign` default to `-1.0` — the board
  strafes and spins opposite REP-103, verified on hardware). Both `/cmd_vel` and `/odom` share these signs, so
  flip a sign here rather than in the teleop node to keep command and odometry consistent. It also has
  `wheel_diameter_mm` (default 48 — the real wheel; the firmware's factory default of 60 over-reported
  velocity by 1.25× and was the root cause of SLAM map drift), plus `wheel_space_mm`/`axle_space_mm`
  (default 110/110 — factory values, **not yet measured on the real robot**; they scale the yaw term).
  These are written into the board's firmware once at node startup (`\x7b\x24` config frame), so changing
  them requires restarting the bringup. Calibration procedures (drive 1 m / spin 720° and compare `/odom`)
  are in `SLAM_learning_note.md` §7. Note `ominibot_driver` is `ament_python`: unlike launch/config edits,
  editing any `.py` requires `colcon build --packages-select ominibot_driver --symlink-install` before
  `ros2 run`/`ros2 launch` pick it up.
  - `ominibot_driver/teleop_node.py` (`mecanum_teleop` console script) — keyboard teleop purpose-built for a
    holonomic base: the numeric-pad `u/i/o j/k/l m/,/.` keys are pure translation (including strafing, a
    first-class motion instead of Shift-hidden like `teleop_twist_keyboard`), `a`/`d` are pure spin, `w`/`s`
    scale linear speed, `q`/`e` scale turn speed (split into two key pairs on 2026-07-14 — they were coupled
    and turn speed "couldn't change"), `k`/space stop. Pad and turn keys are mutually exclusive (pressing one zeroes the other
    axis). It re-publishes the current `Twist` every loop (≥10 Hz) to keep the driver's `cmd_vel` watchdog
    fed. Run with `ros2 run ominibot_driver mecanum_teleop`.
- `dds/fastdds_lan.xml` — Fast DDS profile **template** that whitelists only the LAN interface + localhost
  (Fast DDS 2.6 on Humble whitelists by IP, not interface name, so the LAN IP must be filled in). Its
  `<address>` is the placeholder `@LAN_IP@` — do **not** point `FASTRTPS_DEFAULT_PROFILES_FILE` at this file
  directly (it won't parse). Instead `source dds/setup_dds.sh`, which auto-detects the machine's current LAN
  interface (excluding tailscale/loopback/virtual; override with `DDS_IFACE=`), renders the template to
  `$XDG_RUNTIME_DIR/fastdds_active.xml`, and exports `FASTRTPS_DEFAULT_PROFILES_FILE`. This is what makes the
  setup venue-portable: re-source (or open a new terminal) after switching networks instead of hand-editing
  the IP. Both machines source the same script. It's wired into `~/.bashrc` on the Pi.
- `udev/99-rplidar.rules` — udev rule binding the RPLidar C1 (CP2102N, VID 10c4 / PID ea60) to `/dev/rplidar`
  by USB serial, so a future chassis board on another CP210x adapter won't steal the port. Install per the
  header comment (`cp` to `/etc/udev/rules.d/`, reload, trigger).
- `run_robot.sh` / `run_rviz.sh` / `save_map.sh` — one-click entry points (see "Runtime deployment" below).
- `maps/` — saved SLAM maps (`.pgm` + `.yaml` pairs) produced by `save_map.sh`.
- `雙機RViz連線.md` — the definitive runbook (Chinese) for the two-machine visualization workflow; read it
  before touching bringup, DDS, or RViz-connectivity issues. Caveat: its 待辦 section's three 2026-07-14
  items (reversed turn, custom teleop, map drift) have all since been fixed in code — trust the code and
  `SLAM_learning_note.md` over that list.
- `SLAM_learning_note.md` — SLAM primer + this project's field-test debrief (Chinese): the
  `map->odom->base_link` TF split, symptom→cause table (map drift, model jump-back on stop, broken maps),
  odometry calibration procedures (§7: drive 1 m to verify `wheel_diameter_mm`, spin 720° to verify
  `wheel_space_mm`/`axle_space_mm`), and a quick-reference table of driver + slam_toolbox parameters. Read it
  before touching odometry, driver geometry params, or slam_toolbox config. Note the slam_toolbox config
  (`mapper_params_online_async.yaml`) lives in `my_robot_lidar` **outside this repo**.
- `command_note.md` — quick crib sheet (Chinese) of the start-to-finish SLAM session commands; overlaps the
  runbook, kept as the operator's cheat sheet.
- `urdf閱讀方法.md` — running notes (in Chinese) on how to validate/view the URDF and known open issues; check
  this file for the current TODO list before doing further URDF work (e.g. missing wheel `<limit>` tags, and
  a `rear_left_wheel_joint` origin RPY that differs from the other three wheels — harmless mathematically
  since it's about the wheel's own rotation axis, but worth visually confirming against SolidWorks).

## Working with the URDF

Validate structural changes with `liburdfdom-tools` (fast, no mesh loading required):

```bash
check_urdf car_assemble_description/urdf/CAR_ASSEMBLE_URDF.urdf
urdf_to_graphiz car_assemble_description/urdf/CAR_ASSEMBLE_URDF.urdf   # renders a link/joint tree PDF
```

To visualize with meshes in RViz, this package must be copied into a ROS 2 workspace (`colcon build`) so
`package://` mesh paths resolve — see `urdf閱讀方法.md` for the exact workflow, including that the
`display.launch.py` RViz view has no saved config (none shipped from the SolidWorks export), so you must add
the `RobotModel` display and set the fixed frame to `base_link` manually.

```bash
# From a ROS 2 workspace root, with this package under src/
colcon build --packages-select car_assemble_description
source install/setup.bash
ros2 launch car_assemble_description display.launch.py   # RViz2 + joint_state_publisher_gui
ros2 launch car_assemble_description gazebo.launch.py     # Gazebo Classic spawn
```

Because `git-lfs` must be installed *before* cloning, verify the meshes are real geometry and not LFS pointer
stubs before a `colcon build`/RViz session: `git lfs ls-files` should list the `meshes/*.STL` files, and each
`meshes/*.STL` should be tens/hundreds of KB, not ~130 bytes.

Before adding `ros2_control`/Gazebo joint dynamics, the four `continuous` wheel joints currently have no
`<limit effort="" velocity=""/>` — this needs to be filled in from the N20 motor's actual effort/velocity
figures, not left as a placeholder.

## Runtime deployment (two-machine setup)

The live robot runs **split across two machines** talking over ROS 2 DDS — `雙機RViz連線.md` is the full
runbook; the essentials:

- **Raspberry Pi = headless backend** (the robot). One-click: `./run_robot.sh` (repo root) — sources ROS +
  workspace + `dds/setup_dds.sh`, then runs `robot_bringup.launch.py` (real chassis + lidar `/scan` + SLAM
  `/map` + model TF). It never opens a GUI. Pass-through args work, e.g. `./run_robot.sh use_fake_odom:=true`.
- **Ubuntu PC = viewer only.** One-click: `./run_rviz.sh` — opens `rviz2` with `rviz/view_robot.rviz`
  (Grid, RobotModel, LaserScan, Map, Odometry, TF preconfigured). It must have `car_assemble_description`
  built locally (so `package://` mesh paths resolve for RobotModel) but does **not** need lidar/SLAM packages.
- The operator then opens two more terminals on the PC: `ros2 run ominibot_driver mecanum_teleop` to drive,
  and `./save_map.sh <name>` to save the map (wraps `map_saver_cli` with `save_map_timeout:=10.0` — the
  default ~2 s timeout often misses the latched `/map` and errors out; bare names land in `maps/`).

`car_assemble_description` is not self-contained at runtime: `robot_bringup.launch.py` depends on
`my_robot_lidar`, `sllidar_ros2`, and `slam_toolbox`, which live in the ROS 2 workspace (`~/ros2_ws/src/`),
**not in this git repo** — plus `ominibot_driver`, which *is* in this repo (symlinked into the workspace).
`robot_bringup.launch.py` launches `ominibot_driver` when `use_fake_odom:=false` and the fake static
`odom->base_link` TF otherwise; the two are mutually exclusive (both publish that same TF). The package is symlinked into `~/ros2_ws/src/` and built with
`colcon build --symlink-install`, so editing `launch/`, `rviz/`, `config/` needs no rebuild; only
`package.xml`/`CMakeLists.txt` changes do.

Both machines must share `ROS_DOMAIN_ID`, set `ROS_LOCALHOST_ONLY=0`, and point
`FASTRTPS_DEFAULT_PROFILES_FILE` at their own copy of `dds/fastdds_lan.xml`. **The DDS LAN whitelist is not
optional here:** the Pi has both `wlan0` and `tailscale0`, and without pinning DDS to the LAN, large samples
(`/robot_description`, `/tf`, `/map`) get routed over tailscale's 1280-MTU link and fragment-drop — `ros2
topic list` still shows the topics (small discovery packets get through) but RViz stays blank. When debugging
connectivity, "topic appears in `list`" ≠ "data is arriving"; confirm with `ros2 topic echo
/robot_description --once` actually printing. Set RViz Fixed Frame to `base_link` first (SLAM takes ~10-15s to
create the `map` frame; `map` before then reads as a blank "does not exist" screen).
