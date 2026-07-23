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
    wheel joints so their TFs exist) + the RPLidar driver (via `my_robot_lidar`'s `lidar_start.launch.py`,
    still an out-of-repo dependency) + the chassis driver, plus optionally SLAM Toolbox (async) on the Pi
    itself. Launch args: `use_slam` (**default false** as of 2026-07-21 — SLAM moved to the PC, see
    `slam_pc.launch.py` below; set `true` for a single-machine fallback), `use_fake_odom` (default false — the
    real `ominibot_driver` runs by default; set `true` for hardware-free model/lidar viewing), and
    `ominibot_port` (default `/dev/serial0`, the Pi GPIO UART). It also declares (and forwards to
    `ominibot_driver`) all of that node's calibration params — `vx_sign`/`vy_sign`/`wz_sign`,
    `wheel_diameter_mm`/`wheel_space_mm`/`axle_space_mm`, `encoder_ppr`/`gear_ratio`, the position/velocity PID
    gains (`pos_kp`/`pos_ki`/`pos_kd`/`vel_kp`/`vel_ki`), `odom_linear_scale`/`odom_angular_scale`, and
    `use_gyro_heading`/`gyro_z_sign`/`gyro_scale` — see the `ominibot_driver` section below for what each does.
    Keep the launch defaults in sync with `driver_node.py`'s, since passing a launch arg explicitly overrides
    the node's own default.
  - `launch/slam_pc.launch.py` — the SLAM entry point, run on the **PC**, not the Pi. `async_slam_toolbox_node`
    is CPU-bound and the Pi 4 couldn't keep up with the lidar's 10 Hz scan rate while also running
    `robot_state_publisher` + `joint_state_publisher` + the lidar driver + `ominibot_driver`: the scan queue
    filled up, scans got dropped, and the resulting bad scan matching combined with drifting odom produced a
    rotating "fan smear" map. Splitting SLAM onto the PC means the Pi only has to stream `/scan` and the
    `odom->base_link` TF over DDS; the PC does the scan matching and publishes `map->odom` + `/map` locally
    (so the large `/map` data never has to cross the network back to the Pi). Both launch files read the
    *same* `config/mapper_params_online_async.yaml`, which now lives **inside this repo**
    (`car_assemble_description/config/`) rather than in `my_robot_lidar` — copied in specifically so the PC
    can build just this one package and run SLAM without also installing `my_robot_lidar`/`sllidar_ros2`. Run
    via `./run_slam.sh` (repo root, PC side).
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
  (default 115/96 as of 2026-07-21 — **measured on the real robot**, replacing the earlier factory
  110/110 placeholder; they scale the yaw term) and `encoder_ppr`/`gear_ratio` (default 165/55 — CircusPi
  factory values for a different motor/gearbox; must be matched to the real N20 when known), plus the
  closed-loop PID gains `pos_kp`/`pos_ki`/`pos_kd`/`vel_kp`/`vel_ki` (factory-tuned for the heavier reference
  chassis; exposed so they can be lowered from the command line to fight vibration on the lighter N20 build
  without a rebuild). These are written into the board's firmware once at node startup (`\x7b\x24`/`\x7b\x23`
  config frames), so changing them requires restarting the bringup.
  **Hardware-verified caveat:** the board's feedback path ignores that geometry/motor config entirely and
  always reports body velocity using a fixed internal calibration for the CircusPi reference robot — a
  config readback confirms the values above are stored on the board, yet changing them does not move the
  reported odom at all. The only lever that actually corrects reported odom is `odom_linear_scale` (default
  0.16) and `odom_angular_scale` (default 0.195, wheel-derived yaw fallback only), which multiply the raw
  feedback back to real SI units before integration — the raw feedback over-reports distance ~5–6.5×.
  Separately, `use_gyro_heading` (default `true`) integrates the board's raw gyro-Z for odom heading instead
  of the wheel-derived yaw rate, because mecanum roller slip destroys the latter (a real 360° spin
  over-reports as ~2270° of wheel yaw, vs. ~350° from the gyro); the IMU quaternion itself can't substitute
  since it's 6-axis with no magnetometer, so yaw is frozen. `gyro_z_sign`/`gyro_scale` fine-tune that gyro
  integration. Calibration procedures for all of the above (drive 1 m / spin 720° and compare `/odom`) are in
  `SLAM_learning_note.md` §7. Note `ominibot_driver` is `ament_python`: unlike launch/config edits, editing
  any `.py` requires `colcon build --packages-select ominibot_driver --symlink-install` before `ros2
  run`/`ros2 launch` pick it up.
  - `ominibot_driver/teleop_node.py` (`mecanum_teleop` console script) — keyboard teleop purpose-built for a
    holonomic base: the numeric-pad `u/i/o j/k/l m/,/.` keys are pure translation (including strafing, a
    first-class motion instead of Shift-hidden like `teleop_twist_keyboard`), `a`/`d` are pure spin, `w`/`s`
    scale linear speed, `q`/`e` scale turn speed (split into two key pairs on 2026-07-14 — they were coupled
    and turn speed "couldn't change"), `k`/space stop. Pad and turn keys are mutually exclusive (pressing one zeroes the other
    axis). It re-publishes the current `Twist` every loop (≥10 Hz) to keep the driver's `cmd_vel` watchdog
    fed. Run with `ros2 run ominibot_driver mecanum_teleop`.
- `dds/fastdds_lan.xml` — Fast DDS profile **template** that does **two** things (as of 2026-07-24): (1)
  configures every participant as a **Discovery Server CLIENT** pointing at a fixed unicast rendezvous
  (`@SERVER_IP@:@SERVER_PORT@`, default port 11811, server GUID prefix `44.53.00.5f...` = `fastdds discovery
  -i 0`), and (2) whitelists only the LAN interface + localhost so bulk data stays off tailscale. Placeholders
  `@LAN_IP@`/`@SERVER_IP@`/`@SERVER_PORT@` mean you must **not** point `FASTRTPS_DEFAULT_PROFILES_FILE` at this
  file directly (it won't parse) — `source dds/setup_dds.sh` instead. **Why the Discovery Server (hard-won,
  2026-07-24):** the venue WiFi AP does **not forward multicast between wireless clients** (proven: `ros2
  multicast send`/`receive` between Pi and PC receives nothing, yet `ping` works), so DDS's default
  multicast-based discovery (SPDP) never links the two machines — `ros2 node list` on each side shows only its
  own local nodes even with matching `ROS_DOMAIN_ID`, correct whitelist, same subnet, and firewall off. The
  server gives discovery a unicast path that doesn't need multicast. The interfaceWhiteList is still needed on
  top (data-path fragmentation over tailscale is a separate problem). Verified end-to-end with the rendered
  profile via a talker/listener over the server.
- `dds/setup_dds.sh` — renders the template with this machine's current LAN IP (auto-detected, excluding
  tailscale/loopback/docker/virtual; override `DDS_IFACE=`) **and** the discovery-server address, then exports
  `FASTRTPS_DEFAULT_PROFILES_FILE` and clears the stale `ros2 daemon` (see below). The server IP comes from
  `DDS_SERVER` (default = this machine's own LAN IP): on the **Pi** (which hosts the server) that default is
  correct with no extra config; on the **PC** you must run `DDS_SERVER=<pi_ip> source dds/setup_dds.sh` (or
  export `DDS_SERVER` in `~/.bashrc`) or the client points at itself and never connects — `run_slam.sh`/
  `run_rviz.sh` warn when `DDS_SERVER` is unset. Venue-portable: re-source after switching networks. Wired
  into `~/.bashrc` on the Pi. **ros2 daemon caching gotcha:** the `ros2` CLI daemon caches discovery config
  from whenever it first started, so a daemon spawned before this profile existed silently ignores it (`ros2
  topic list` shows nothing / `echo` reports "could not determine type") — `setup_dds.sh` now runs `ros2
  daemon stop` so the next command respawns it with the right profile; do the same by hand if a debug terminal
  acts stale.
- `dds/run_discovery_server.sh` — starts the Fast DDS Discovery Server (`fastdds discovery -i 0 -p 11811`,
  bound to `0.0.0.0` so it survives the Pi's DHCP IP changing) on the **Pi**. `run_robot.sh` auto-starts it in
  the background (logs to `/tmp/dds_discovery_server.log`); run it by hand only to host the server without the
  full bringup. Leave server-id 0 — its GUID prefix is hard-coded as the `RemoteServer` prefix in the template.
- `udev/99-rplidar.rules` — udev rule binding the RPLidar C1 (CP2102N, VID 10c4 / PID ea60) to `/dev/rplidar`
  by USB serial, so a future chassis board on another CP210x adapter won't steal the port. Install per the
  header comment (`cp` to `/etc/udev/rules.d/`, reload, trigger).
- `run_robot.sh` / `run_slam.sh` / `run_rviz.sh` / `save_map.sh` — one-click entry points (see "Runtime
  deployment" below).
- `maps/` — saved SLAM maps (`.pgm` + `.yaml` pairs) produced by `save_map.sh`.
- `雙機RViz連線.md` — the definitive runbook (Chinese) for the two-machine visualization workflow; read it
  before touching bringup, DDS, or RViz-connectivity issues. Caveat: its 待辦 section's three 2026-07-14
  items (reversed turn, custom teleop, map drift) have all since been fixed in code — trust the code and
  `SLAM_learning_note.md` over that list.
- `SLAM_learning_note.md` — SLAM primer + this project's field-test debrief (Chinese): the
  `map->odom->base_link` TF split, symptom→cause table (map drift, model jump-back on stop, broken maps),
  odometry calibration procedures (§7: drive 1 m to verify `odom_linear_scale`, spin 720° to verify
  `use_gyro_heading`/`gyro_scale`), and a quick-reference table of driver + slam_toolbox parameters. Read it
  before touching odometry, driver geometry params, or slam_toolbox config. As of 2026-07-21 the slam_toolbox
  config (`mapper_params_online_async.yaml`) has been copied **into this repo**
  (`car_assemble_description/config/`) so the PC can run SLAM without installing `my_robot_lidar`; see
  `launch/slam_pc.launch.py` above.
- `command_note.md` — quick crib sheet (Chinese) of the start-to-finish SLAM session commands; overlaps the
  runbook, kept as the operator's cheat sheet.
- `networkplan.md` — in-progress notes (Chinese) on bringing self-hosted Wi-Fi (phone or laptop hotspot) to
  the competition venue instead of relying on venue Wi-Fi, so the Pi/PC DDS link stays on a network the team
  controls; covers why venue Wi-Fi is risky for DDS (congestion, AP client isolation, blocked multicast,
  captive portals) and phone-hotspot vs. laptop-hotspot tradeoffs. Not yet finalized into netplan config.
- `0721_net_issue_plan.md` — the field-test debrief (Chinese) that closed out the `slam_toolbox` "queue is
  full" symptom: source-side telemetry (`/scan` 10 Hz, `/odom` 20 Hz, `/tf` ~60 Hz, full TF chain, correct
  DDS whitelist) is all clean, so the dropped scans are **WiFi transport jitter**, not a ROS config or
  source-data fault — a WiFi stall delays the `odom->base_link` TF so SLAM's message filter queues the scan
  until the queue overflows (`transform_timeout:=1.0` only masks it). Also records that the Pi's `wlan0` is
  managed by **netplan → systemd-networkd** (NetworkManager shows it `unmanaged`), so the venue-hotspot fix
  from `networkplan.md` must be written as multiple `access-points` with `priority` under `wlan0` in netplan,
  **not** via `nmcli`. Read this (with `networkplan.md`) before touching Pi networking or re-diagnosing
  dropped scans.
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
runbook (though its "two-machine" description predates the 2026-07-21 SLAM move and should be read alongside
`slam_pc.launch.py`'s docstring, which explains the current split); the essentials:

- **Raspberry Pi = headless backend** (the robot). One-click: `./run_robot.sh` (repo root) — sources ROS +
  workspace + `dds/setup_dds.sh`, kills any stale bringup/driver from a previous run (orphaned launch children
  otherwise hold the GPIO-UART port open and a second driver instance fights over it, corrupting reads),
  starts the **Fast DDS Discovery Server** in the background (`dds/run_discovery_server.sh` — the unicast
  rendezvous both machines' nodes connect to; see the DDS section), then runs `robot_bringup.launch.py` with
  `use_slam:=false` by default (real chassis + lidar `/scan` + model TF; no SLAM on the Pi). Pass-through args
  work, e.g. `./run_robot.sh use_fake_odom:=true` or `./run_robot.sh use_slam:=true` for a single-machine
  fallback.
- **Ubuntu PC = SLAM + viewer.** Two one-click scripts: `./run_slam.sh` runs `slam_pc.launch.py`
  (`async_slam_toolbox_node`, killing any stale instance first so `map->odom` isn't published twice) — this
  is where scan matching now happens, moved off the Pi 4 because it couldn't keep up with 10 Hz scans
  alongside everything else running there (dropped scans + drifting odom produced a rotating "fan smear"
  map). `./run_rviz.sh` opens `rviz2` with `rviz/view_robot.rviz` (Grid, RobotModel, LaserScan, Map, Odometry,
  TF preconfigured; Fixed Frame `map`). Both need `car_assemble_description` built locally on the PC (for
  `package://` mesh paths and the in-repo `mapper_params_online_async.yaml`) plus `ros-humble-slam-toolbox`
  installed — but not `my_robot_lidar`/`sllidar_ros2`, which stay Pi-only.
- The operator then opens two more terminals on the PC: `ros2 run ominibot_driver mecanum_teleop` to drive,
  and `./save_map.sh <name>` to save the map (wraps `map_saver_cli` with `save_map_timeout:=10.0` — the
  default ~2 s timeout often misses the latched `/map` and errors out; bare names land in `maps/`).

`car_assemble_description` is not self-contained at runtime: `robot_bringup.launch.py` still depends on
`my_robot_lidar`/`sllidar_ros2` for the lidar driver, which live in the ROS 2 workspace (`~/ros2_ws/src/`),
**not in this git repo** — plus `ominibot_driver`, which *is* in this repo (symlinked into the workspace).
`slam_toolbox` itself must be installed on whichever machine runs SLAM (PC by default, or the Pi if
`use_slam:=true`), but its config now ships inside this repo. `robot_bringup.launch.py` launches
`ominibot_driver` when `use_fake_odom:=false` and the fake static `odom->base_link` TF otherwise; the two are
mutually exclusive (both publish that same TF). The package is symlinked into `~/ros2_ws/src/` and built with
`colcon build --symlink-install`, so editing `launch/`, `rviz/`, `config/` needs no rebuild; only
`package.xml`/`CMakeLists.txt` changes do.

Both machines must share `ROS_DOMAIN_ID`, set `ROS_LOCALHOST_ONLY=0`, and `source dds/setup_dds.sh` (the PC
with `DDS_SERVER=<pi_ip>` — see the `dds/` bullets above). This gives them **two** independent fixes that the
two-machine link needs, and both are mandatory:
- **Discovery** goes through the Pi's Fast DDS **Discovery Server** (unicast), because the venue WiFi AP does
  not forward multicast between clients and DDS's default discovery is multicast-based. Symptom when this is
  the problem: `ros2 node list` on each machine shows only its **own** nodes even though `ping` works,
  `ROS_DOMAIN_ID` matches, the whitelist is correct, and the firewall is off. Confirm multicast is the culprit
  with `ros2 multicast receive` (PC) + `ros2 multicast send` (Pi) — no datagram arrives.
- **Data path** is pinned to the LAN by the interfaceWhiteList, because the Pi/PC both also have `tailscale0`
  (+ `docker0`); without it, large samples (`/robot_description`, `/tf`, `/map`) route over tailscale's
  1280-MTU link and fragment-drop while small discovery packets still get through.

When debugging connectivity, "topic appears in `list`" ≠ "data is arriving" (and now, *before* that, "node
appears in `node list`" is itself the thing that fails first) — confirm with `ros2 topic echo
/robot_description --once` actually printing. Remember the **ros2 daemon caches discovery config**: if things
look wrong right after re-sourcing, `ros2 daemon stop` and retry (`setup_dds.sh` does this automatically).
Set RViz Fixed Frame to `base_link` first (SLAM takes ~10-15s to create the `map` frame; `map` before then
reads as a blank "does not exist" screen).
