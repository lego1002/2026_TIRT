# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Hardware design files and robot software for a maze-solving robot built for the 2026 TIRT competition
(`docs/2026TIRT-迷宮機器人挑戰賽.pdf`). This is primarily a CAD/URDF repository, not an application codebase —
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
  - The package has **no `config/` directory of its own** as of 2026-07-28 — all YAML lives in the repo-root
    `config/`, see that bullet below. Launch files still resolve params through
    `get_package_share_directory('car_assemble_description') + '/config'` and need no path knowledge, because
    `CMakeLists.txt` installs `../config/` into the package share.
  - `launch/display.launch.py` and `launch/gazebo.launch.py` — ROS 2 Python launch files (RViz2 preview with
    `joint_state_publisher_gui`, and Gazebo Classic spawn via `gazebo_ros`/`spawn_entity.py`). See
    `notes/urdf閱讀方法.md`'s "ROS 2 轉換" and "套件改名" sections for the full conversion/rename rationale and what
    was dropped (the ROS 1 `/calibrated` rostopic-pub step has no ROS 2 equivalent and was removed).
  - `launch/robot_bringup.launch.py` — the **real** on-robot entry point (headless, GUI-free), meant to run on
    the Raspberry Pi. It brings up `robot_state_publisher` + a non-GUI `joint_state_publisher` (zeros the four
    wheel joints so their TFs exist) + the RPLidar driver (`sllidar_ros2`'s `sllidar_node`, launched
    **directly by this file** as of 2026-07-25 — it used to `IncludeLaunchDescription` out-of-repo
    `my_robot_lidar`'s `lidar_start.launch.py`; see the `laser_*` args below for why that moved in) + the
    chassis driver, plus optionally SLAM Toolbox (async) on the Pi
    itself. Launch args: `use_slam` (**default false** as of 2026-07-21 — SLAM moved to the PC, see
    `slam_pc.launch.py` below; set `true` for a single-machine fallback), `use_fake_odom` (default false — the
    real `ominibot_driver` runs by default; set `true` for hardware-free model/lidar viewing), and
    `ominibot_port` (default `/dev/serial0`, the Pi GPIO UART). It also declares (and forwards to
    `ominibot_driver`) all of that node's calibration params — `vx_sign`/`vy_sign`/`wz_sign`,
    `wheel_diameter_mm`/`wheel_space_mm`/`axle_space_mm`, `encoder_ppr`/`gear_ratio`, the position/velocity PID
    gains (`pos_kp`/`pos_ki`/`pos_kd`/`vel_kp`/`vel_ki`), `odom_linear_scale`/`odom_angular_scale`, and
    `use_gyro_heading`/`gyro_z_sign`/`gyro_scale` — see the `ominibot_driver` section below for what each does.
    Keep the launch defaults in sync with `driver_node.py`'s, since passing a launch arg explicitly overrides
    the node's own default. It also owns the lidar extrinsic — `laser_x`/`laser_y`/`laser_z`/`laser_yaw`
    (defaults `0.014`/`-0.014`/`0.109`/`0.0`), fed to the `base_link -> laser_frame`
    `static_transform_publisher`. **These are un-calibrated CAD estimates, not measured values.** The old
    `my_robot_lidar` version of this TF was all zeros with a comment admitting it "assumed the lidar is
    mounted at the robot's center", but the URDF's `lidar_joint` puts the lidar at
    `(0.0088, -0.061, 0.0427)` and the wheel-joint centroid (the point `odom` actually tracks) sits at
    `(-0.0047, -0.047)` in `base_link` — so the lidar is ~1.4 cm off the rotation center in each axis, not 0.
    A zero here makes the lidar orbit a small circle during in-place spins while SLAM thinks it is fixed,
    which smears walls into double lines on every turn — the prime suspect for the broken maps. Calibrate
    with the overlay procedure in `notes/SLAM_learning_note.md` §7.3 (no rebuild needed: pass
    `./run_robot.sh laser_x:=... laser_y:=...`), then write the converged values back as the defaults here.
  - `launch/slam_pc.launch.py` — the SLAM entry point, run on the **PC**, not the Pi. `async_slam_toolbox_node`
    is CPU-bound and the Pi 4 couldn't keep up with the lidar's 10 Hz scan rate while also running
    `robot_state_publisher` + `joint_state_publisher` + the lidar driver + `ominibot_driver`: the scan queue
    filled up, scans got dropped, and the resulting bad scan matching combined with drifting odom produced a
    rotating "fan smear" map. Splitting SLAM onto the PC means the Pi only has to stream `/scan` and the
    `odom->base_link` TF over DDS; the PC does the scan matching and publishes `map->odom` + `/map` locally
    (so the large `/map` data never has to cross the network back to the Pi). Both launch files read the
    *same* `mapper_params_online_async.yaml`, which now lives **inside this repo** (the repo-root `config/`)
    rather than in `my_robot_lidar` — copied in specifically so the PC can build just this one package and run
    SLAM without also installing `my_robot_lidar`/`sllidar_ros2`. Run via `./run_slam.sh` (repo root, PC side).
  - `launch/nav2_pc.launch.py` (+ the repo-root `config/nav2_params.yaml`) — the **Nav2 autonomous-navigation**
    entry point
    (added 2026-07-28), also PC-side, and **mutually exclusive with `slam_pc.launch.py`**: slam_toolbox and
    AMCL both publish `map->odom`, so running both shreds the TF tree. Run via `./run_nav2.sh [map_name]`
    (repo root, PC side), which pkills a stale slam_toolbox first. The launch file is a thin wrapper around
    nav2_bringup's `bringup_launch.py` (`slam:=False`, so localization is AMCL against a saved map) plus RViz;
    all the customization is in `config/nav2_params.yaml`. `map:` has **no default** and must be an absolute
    path — maps deliberately stay in the repo's `maps/` (outside the package share) so saving a new one needs
    no `colcon build`; `run_nav2.sh` resolves a bare name like `201_self_test` to `maps/201_self_test.yaml`.
    What `nav2_params.yaml` changes versus nav2_bringup's stock file, and why (all four matter):
    `use_sim_time` false throughout; `base_frame_id`/`robot_base_frame` is **`base_link`**, not
    `base_footprint` (this URDF has no such link); everything switched to **holonomic** (AMCL
    `nav2_amcl::OmniMotionModel`, non-zero `max_vel_y`/`vy_samples`, and `min_y_velocity_threshold` 0.5 →
    0.001 — the stock 0.5 exists to discard strafe as noise on a diff-drive base and silently kills mecanum
    strafing); and every dimension scaled down one size class for a 0.15 m robot in a maze
    (`robot_radius` 0.22 → **0.11**, `inflation_radius` 0.55 → **0.18**, `xy_goal_tolerance` 0.25 → **0.10**,
    NavFn `tolerance` 0.5 → 0.15, `allow_unknown` **false** so it won't plan through unmapped cells).
    Local-costmap `width`/`height` are declared **integer metres** in `nav2_costmap_2d` — a `2.5` there makes
    `controller_server`'s constructor throw and the whole `nav2_container` fail to load (hit on hardware).
    The local planner is **DWB with the y axis opened up**, not MPPI: MPPI's `motion_model: "Omni"` is the
    better fit for mecanum in tight corridors, but the arm64 `nav2_mppi_controller` binary **SIGILLs on the
    Pi 4's Cortex-A72** the instant it loads (`nav2_container` dies with exit code -4, right after "Created
    controller : FollowPath") — it uses instructions that CPU lacks, and no parameter change helps. Nav2
    normally runs on the x86 PC where this doesn't apply, but a default that hard-crashes on one of the two
    machines is a bad default; the full MPPI block is kept commented at the end of `nav2_params.yaml` as a
    one-edit swap. Smoke-tested 2026-07-28 on the Pi against live `/scan` + `/odom`: all lifecycle nodes
    reach active, map loads, AMCL processes scans, zero warnings.
  - `rviz/view_nav2.rviz` — RViz config for navigation, copied from `nav2_bringup`'s `nav2_default_view.rviz`
    (so it has the Navigation 2 panel, the **Nav2 Goal** tool, costmaps, plans and the AMCL particle cloud)
    with two local fixes: RobotModel's `/robot_description` durability Volatile → **Transient Local** (that
    topic is latched, so a Volatile subscriber joining after `robot_state_publisher` gets no model at all)
    and enabled by default, plus the TB3-only "Bumper Hit" display disabled.
  - `rviz/view_robot.rviz` — saved RViz2 config for the **PC-side viewer** in the two-machine setup (Fixed
    Frame `map`, RobotModel on `/robot_description`, LaserScan `/scan`, Map `/map` with Durability set to
    **Transient Local** to receive the latched map, Odometry with `Keep: 1` and small arrows — the previous
    `Keep: 50` + 0.4 m arrows visually buried the 0.15 m robot; temporarily set `Keep` back up to visualize
    odometry error as a breadcrumb trail when calibrating — and TF). Note this is distinct from
    `display.launch.py`, which still ships no saved config and needs its displays added by hand.
- `config/` — **the single home for every tunable YAML** (consolidated here 2026-07-28 at the operator's
  request: params, like `maps/`, are what actually gets edited between field runs, so they should not be
  buried inside a package). `nav2_params.yaml`, `mapper_params_online_async.yaml`, and the leftover ROS 1
  `joint_names_CAR_ASSEMBLE_URDF.yaml` (dead — `ros2_control` uses a different format entirely, so it needs a
  rewrite rather than a conversion once `ros2_control` is wired up). `car_assemble_description/CMakeLists.txt`
  installs `../config/` into the package share, so launch files keep resolving params via
  `get_package_share_directory(...)` with no knowledge of where the repo is, and `--symlink-install` means
  editing these files needs no rebuild (a **restart** of the affected node is still required — nav2 and
  slam_toolbox read these at configure time). Trade-off accepted: `car_assemble_description` can no longer be
  copied to another workspace on its own. **Caution:** before this consolidation, root `config/` held a
  *stale upstream-default* copy of `mapper_params_online_async.yaml` (`base_frame: base_footprint`,
  `transform_timeout: 0.2`, `max_laser_range: 20`) that nothing read while the tuned copy lived in the
  package — if an older branch or backup is ever merged, make sure the tuned version wins.
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
  base after `cmd_vel_timeout` — **1.0 s and BEST_EFFORT depth-1 as of 2026-07-27**, see "Where teleop runs,
  and why" under Runtime deployment for the WiFi-stutter reasoning), and a background read thread dead-reckons `/odom` + broadcasts
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
  `notes/SLAM_learning_note.md` §7. Note `ominibot_driver` is `ament_python`: unlike launch/config edits, editing
  any `.py` requires `colcon build --packages-select ominibot_driver --symlink-install` before `ros2
  run`/`ros2 launch` pick it up.
  - `ominibot_driver/teleop_node.py` (`mecanum_teleop` console script) — keyboard teleop purpose-built for a
    holonomic base: the numeric-pad `u/i/o j/k/l m/,/.` keys are pure translation (including strafing, a
    first-class motion instead of Shift-hidden like `teleop_twist_keyboard`), `a`/`d` are pure spin, `w`/`s`
    scale linear speed, `q`/`e` scale turn speed (split into two key pairs on 2026-07-14 — they were coupled
    and turn speed "couldn't change"), `k`/space stop. Pad and turn keys are mutually exclusive (pressing one zeroes the other
    axis). It re-publishes the current `Twist` every loop (≥10 Hz) to keep the driver's `cmd_vel` watchdog
    fed. Run with `ros2 run ominibot_driver mecanum_teleop`.
- `net/` — the **venue-portability layer** (added 2026-07-27). The whole point: stop chasing DHCP IPs.
  - `net/tirt_net.conf` — the single source of truth for the two fixed alias IPs (`TIRT_PI_IP=10.77.0.2`,
    `TIRT_PC_IP=10.77.0.1`), the ssh user/repo path, DDS port, and the Pi tmux session name. Every value is
    written `${VAR:-default}` so it can be overridden from the environment for a one-off test without editing
    the file. Sourced by `setup_dds.sh`, `robotctl`, `gcs.sh`, `pi/robot_tmux.sh`, and both installers.
  - **Why fixed alias IPs.** Each machine keeps DHCP *and* additionally holds a static `10.77.0.x/24` address
    on its wireless interface. Venue IPs still come from DHCP (internet, tailscale), but the robot link only
    ever uses the constants — so `DDS_SERVER=<pi_ip>` is gone, `ssh` has a fixed target, and switching networks
    requires editing nothing. It also avoids mDNS, which would be the obvious alternative but travels by
    multicast — exactly what the venue AP was proven to block (see the Discovery Server note below). The one
    prerequisite is that the AP forwards *unicast* between clients; if it has full client isolation nothing
    works anyway, which is what the self-hosted-hotspot plan in `networkplan.md` is for.
  - `net/60-tirt-wifi.yaml.example` → copy to `net/60-tirt-wifi.yaml`, fill in SSIDs/passwords, install with
    `sudo ./net/install_pi_network.sh`. Lists **all** venues' `access-points` at once (phone hotspot, RMML_2G,
    …) so wpa_supplicant auto-joins whichever is present, plus `addresses: [10.77.0.2/24]` alongside
    `dhcp4: true`. Note **netplan 0.107.1 has no per-AP `priority` key** (verified) — with two networks in
    range the choice is by signal strength; that's fine since they don't co-occur in practice. The installer
    uses **`netplan try`** (auto-rollback after 120 s), never `netplan apply` — a bad wifi block otherwise
    kills SSH permanently. The real file is gitignored; only the `.example` is tracked (it holds passwords).
  - `net/install_pc_alias.sh` — laptop side. Adds `10.77.0.1/24` immediately *and* persists it at the
    **interface** level, not per-SSID (a per-connection setting would need redoing for every new venue):
    a NetworkManager dispatcher script `/etc/NetworkManager/dispatcher.d/90-tirt-alias`, or a
    systemd-networkd `.network.d/tirt-alias.conf` drop-in, whichever the laptop actually uses.
  - `net/alias_now.sh pi|pc [--off]` — adds the alias **non-persistently** (`ip addr add`, gone on reboot).
    The escape hatch for "I want to drive today and haven't filled in the WiFi passwords yet", and for
    debugging. Once the two installers above have run, this is never needed.
- `dds/fastdds_pi.xml` / `dds/fastdds_pc.xml` — the two Fast DDS profiles, one per machine. They replaced the
  old `fastdds_lan.xml` **template** on 2026-07-27: because both endpoints now have constant IPs, the
  `@LAN_IP@`/`@SERVER_IP@` placeholders and the whole `sed`-render-per-venue mechanism became unnecessary.
  Each profile (1) makes every participant a Discovery Server **SUPER_CLIENT** of `10.77.0.2:11811` (server
  GUID prefix `44.53.00.5f...` = `fastdds discovery -i 0`; do not change it), and (2) whitelists only that
  machine's own alias + `127.0.0.1`. The full rationale for both lives in `fastdds_pi.xml`'s header comment;
  `fastdds_pc.xml` deliberately doesn't repeat it. **Why the Discovery Server (hard-won, 2026-07-24):** the
  venue WiFi AP does **not forward multicast between wireless clients** (proven: `ros2 multicast
  send`/`receive` between Pi and PC receives nothing, yet `ping` works), so DDS's default multicast-based
  discovery (SPDP) never links the two machines — `ros2 node list` on each side shows only its own local nodes
  even with matching `ROS_DOMAIN_ID`, correct whitelist, same subnet, and firewall off. The server gives
  discovery a unicast path that doesn't need multicast. The interfaceWhiteList is still needed on top
  (data-path fragmentation over tailscale is a separate problem).
- `dds/setup_dds.sh` — `source` it (both machines, same command, no arguments). Picks `fastdds_pi.xml` or
  `fastdds_pc.xml` by checking **which alias IP this machine holds** (force with `TIRT_ROLE=pi|pc`), exports
  `FASTRTPS_DEFAULT_PROFILES_FILE`, and clears the stale `ros2 daemon`. Two behaviours worth knowing:
  - **No alias → it errors, unsets `FASTRTPS_DEFAULT_PROFILES_FILE`, and configures nothing.** The unset is
    not cosmetic: `~/.bashrc` sources this file, so a shell can inherit the *old* rendered
    `/run/user/1000/fastdds_active.xml` path; every launcher's guard is `[ -z "$FASTRTPS_..." ]`, and a stale
    value would sail through it and start a robot that silently talks to nobody. All of `run_robot.sh`,
    `run_slam.sh`, `run_rviz.sh`, `save_map.sh`, `gcs.sh` now abort on an empty value rather than start.
  - Re-sourcing when it is already correct is a **silent no-op** — `~/.bashrc` plus each tmux window's rc file
    means it gets sourced 2–3 times per window, and without this every window opened with duplicate banners.
  - **ros2 daemon caching gotcha:** the `ros2` CLI daemon caches discovery config from whenever it first
    started, so a daemon spawned before this profile existed silently ignores it (`ros2 topic list` shows
    nothing / `echo` reports "could not determine type") — `setup_dds.sh` runs `ros2 daemon stop` so the next
    command respawns it correctly; do the same by hand if a debug terminal acts stale.
- `dds/run_discovery_server.sh` — starts the Fast DDS Discovery Server (`fastdds discovery -i 0 -p 11811`,
  bound to `0.0.0.0`) on the **Pi**. It is **not** started by `run_robot.sh` any more (it used to be, in the
  background, logging to `/tmp`): `pi/robot_tmux.sh` gives it its own `dds` tmux window so its output is
  visible, and — more importantly — so restarting the bringup doesn't restart the server underneath it, which
  would force every PC-side node to re-discover. Leave server-id 0; its GUID prefix is hard-coded in both
  `fastdds_*.xml`.
- `udev/99-rplidar.rules` — udev rule binding the RPLidar C1 (CP2102N, VID 10c4 / PID ea60) to `/dev/rplidar`
  by USB serial, so a future chassis board on another CP210x adapter won't steal the port. Install per the
  header comment (`cp` to `/etc/udev/rules.d/`, reload, trigger).
- `gcs.sh` / `robotctl` / `pi/robot_tmux.sh` — the operator entry points (see "Runtime deployment" below).
  `run_robot.sh` / `run_slam.sh` / `run_rviz.sh` / `save_map.sh` are still there and still work standalone,
  but are now mostly called *by* those three. `run_nav2.sh` is **not** called by `gcs.sh` — it is the
  alternative to `run_slam.sh` (map-building vs. driving on a finished map), so which one you want is a
  per-session decision; run it in `gcs.sh`'s `shell` window, or standalone.
- `maps/` — saved SLAM maps (`.pgm` + `.yaml` pairs) produced by `save_map.sh`, and consumed by
  `run_nav2.sh` / Nav2's `map_server`. `201_self_test` (2026-07-28) is 168×104 cells @ 0.05 m = 8.4 × 5.2 m.
- `tools/` — standalone diagnostic scripts (plain `python3 foo.py`, no colcon package, no rebuild).
  `odom_check.py` prints live cumulative displacement/heading from `/odom` in metres and **degrees**
  (far more readable than echoing quaternions) and, on Ctrl-C, computes the `odom_linear_scale` /
  `gyro_scale` you should set — run the 1 m and 360° tests with both scales forced to `1.0` so the
  numbers it prints are absolute. `map_check.py` counts `/map`'s unknown/free/occupied cells per
  update, to distinguish "SLAM is republishing a dead map on `map_update_interval`" from "the map is
  actually growing" — `ros2 topic hz /map` cannot tell those apart. It subscribes with
  `TRANSIENT_LOCAL` durability to match slam_toolbox's latched publisher.
  `motor_diag.py --label air|floor` is the entry point for **chassis control / stutter** problems
  (2026-07-26). It measures three independent axes in one run — **serial link health**
  (`good`/`bcc_fail`/`desync`/`timeout` from `OminiBotHV.stats`), **body-velocity dropout**
  (the quantification of "一頓一頓": what fraction of samples collapse toward zero while the
  command is held constant), and **four-wheel sync** (per-wheel encoder rates via the `0x36`
  readback). Run it **twice** (`--label air`, then `--label floor`); a single run proves nothing,
  the air↔floor difference is the diagnosis. Interpretation is built into its output. It needs the
  driver stopped (`pkill -f ominibot_driver`) since the port is exclusive.
  `cmd_vel_check.py` is the **network-side** counterpart to `motor_diag.py` (2026-07-27): run it on the
  Pi while driving and it reports the `/cmd_vel` inter-arrival p50/p95/p99/max and, crucially, **how many
  gaps exceeded `cmd_vel_timeout`** — each one is a watchdog trip that zeroes the base, which is what
  "一頓一頓" feels like. `ros2 topic hz` cannot show this: a stream averaging a healthy 20 Hz can still
  stall 0.8 s every few seconds. Use it to decide *which* tool to reach for next — clean arrival with a
  still-stuttering robot means the problem is mechanical/electrical, so go to `motor_diag.py`.
  `record_diag.py <out_dir>` **replaces `ros2 bag record` on the Pi**, which silently produces an empty
  bag there: `ros2 bag record` finds its publishers through the same CLI graph query that is blind under
  the Discovery Server, so it subscribes to nothing and reports no error (verified 2026-07-25 — a 60 s
  recording yielded 0 topics / 0 messages). This script subscribes with plain rclpy (endpoint matching by
  topic name, which works) and writes a standard rosbag2 via `rosbag2_py`, so `ros2 bag play` still works
  on it. It subscribes `/tf_static` with `TRANSIENT_LOCAL` — miss that and the recorded bag has no
  `base_link->laser_frame`, breaking the TF chain on replay.
  `analyze_bag.py <bag_dir>` treats `/scan` as ground truth to audit `/odom`: it auto-segments the
  recording on odom twist, recovers the true rotation over a spin by circular cross-correlation of
  consecutive scans (→ `gyro_scale`, and catches a flipped `gyro_z_sign`), and recovers the true heading
  offset by least-squares fitting `Δr(θ) ≈ -d·cos(θ-φ)` over a straight run (→ `laser_yaw`). Record with
  `record_diag.py` following the still → spin 360° → still → drive 1 m → still routine; the stationary
  gaps are what the segmenter keys on.
- `docs/` — reference documents: the competition rulebook PDF (`2026TIRT-迷宮機器人挑戰賽.pdf`), the OminiBotHV
  serial-protocol/kinematics spec PDF (a copy of the one in `OminiBotHV-master/communication/`), and field-test
  screenshots. As of the 2026-07-25 "reorganize the structure" commit, all the Chinese design/field-test notes
  moved from the repo root into `notes/` (`SLAM_learning_note.md`, `command_note.md`, `urdf閱讀方法.md`,
  `雙機RViz連線.md`); `networkplan.md` and `0721_net_issue_plan.md` stayed at the root.
- `build/`, `install/`, `log/` — **gone, and must stay gone.** These were colcon output accidentally committed
  in the 2026-07-25 reorg; `be73e1f` untracked them, `.gitignore` covers `/build` `/install` `/log`, and the
  leftover local copies were deleted on 2026-07-25. **Never `colcon build` from the repo root.** That in-tree
  `install/` was a real-file (non-`--symlink-install`) copy that froze whatever the source looked like at build
  time, and because it shadows the real workspace on `AMENT_PREFIX_PATH` it silently runs stale code: on
  2026-07-25 it cost a debugging session, serving `odom_linear_scale = 0.16` and a `robot_bringup.launch.py`
  with no `laser_*` args while the live source had both — the running `ominibot_driver` was the stale copy, so
  none of that session's fixes were actually being exercised. The editable sources live in
  `car_assemble_description/` and `ominibot_driver/`, both symlinked into `~/ros2_ws/src/`, which is the **only**
  build workspace. **Always `source ~/ros2_ws/install/setup.bash`, never `source install/setup.bash` from the
  repo root** (`run_robot.sh` already gets this right; a manual source in the same shell can shadow it).
- `notes/雙機RViz連線.md` — the definitive runbook (Chinese) for the two-machine visualization workflow; read it
  before touching bringup, DDS, or RViz-connectivity issues. Caveat: its 待辦 section's three 2026-07-14
  items (reversed turn, custom teleop, map drift) have all since been fixed in code — trust the code and
  `notes/SLAM_learning_note.md` over that list.
- `notes/SLAM_learning_note.md` — SLAM primer + this project's field-test debrief (Chinese): the
  `map->odom->base_link` TF split, symptom→cause table (map drift, model jump-back on stop, broken maps),
  odometry calibration procedures (§7: drive 1 m to verify `odom_linear_scale`, spin 720° to verify
  `use_gyro_heading`/`gyro_scale`), and a quick-reference table of driver + slam_toolbox parameters. Read it
  before touching odometry, driver geometry params, or slam_toolbox config. As of 2026-07-21 the slam_toolbox
  config (`mapper_params_online_async.yaml`) has been copied **into this repo** (repo-root `config/` since
  2026-07-28) so the PC can run SLAM without installing `my_robot_lidar`; see `launch/slam_pc.launch.py` above.
- `notes/nav2_tuning.md` — the **Nav2 tuning manual** (Chinese, added 2026-07-28 after the first real drive).
  Read this rather than re-deriving parameter effects: a symptom→parameter index, the three places speed must
  be changed in lockstep, the slip-vs-stall distinction (accel limits vs. `min_speed_*`), why "no path" is
  almost always costmap geometry rather than the planner, what each DWB critic actually controls, and
  swap-in tables for the other installed planners/controllers. It also draws the line for maze-solving
  algorithms: right-hand-rule / DFS / flood-fill belong **above** Nav2 as a `NavigateToPose` client
  (`nav2_simple_commander`), not as a planner plugin.
- `notes/command_note.md` — quick crib sheet (Chinese) of the start-to-finish SLAM session commands; overlaps the
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
- `notes/urdf閱讀方法.md` — running notes (in Chinese) on how to validate/view the URDF and known open issues; check
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
`package://` mesh paths resolve — see `notes/urdf閱讀方法.md` for the exact workflow, including that the
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

The live robot runs **split across two machines** talking over ROS 2 DDS. As of **2026-07-27 the operator
workflow is one command on the laptop and zero commands on the Pi**; `notes/雙機RViz連線.md` is the full
runbook. The previous flow (ssh into the Pi for `./run_robot.sh`, then four more laptop terminals, each
needing `DDS_SERVER=<pi_ip>` looked up by hand) is gone — read this section, not older descriptions of it.

### The one command

```bash
./gcs.sh                       # laptop, repo root. That's the whole startup.
./gcs.sh use_fake_odom:=true   # any robot_bringup.launch.py arg passes through to the Pi
./gcs.sh --down                # shut everything down, both machines
```

`gcs.sh` sources the local DDS profile, calls `robotctl up` to start the Pi over ssh, opens a laptop tmux
session (`robot` = live Pi output + keyboard teleop, `slam` = `run_slam.sh`, `shell` = scratch, with
`Ctrl-b m` bound to save a timestamped map), and launches RViz detached as a GUI window.

### Pi side: a tmux session, not a service

`pi/robot_tmux.sh` (invoked over ssh by `robotctl`, runnable directly on the Pi) owns a tmux session `tirt`
with one window per concern: `dds` (discovery server), `bringup` (`run_robot.sh`), `teleop`, `shell`. Design
notes that matter:

- **Deliberately manual, deliberately not systemd.** The operator wants to see each piece start and to be able
  to isolate a failure. Attaching shows exactly what an interactive ssh session would.
- Commands are delivered with `tmux send-keys` into an interactive bash rather than being the window's
  process, so each command **lands in that shell's history** — Ctrl-C then ↑ Enter re-runs just that piece.
- Each window's bash starts from a generated rc file that sources ROS + `~/ros2_ws` + `dds/setup_dds.sh`, so
  every window is ready for ad-hoc `ros2` commands.
- The session's tmux prefix is **`Ctrl-a`**, because the laptop session (`Ctrl-b`) nests it.
- `robot_tmux.sh up` waits up to 30 s for the `10.77.0.2` alias before starting anything — without it the
  robot would come up looking healthy and be unreachable.
- Bringup args live in `~/.tirt_robot_args` on the Pi (`./robotctl args "..."`), so they survive restarts and
  don't require editing anything.

### `robotctl` — laptop-side remote control (no ssh session needed)

```
./robotctl up [dds|bringup|teleop|shell]   # default all; already-running windows untouched
./robotctl restart bringup                 # restart ONE piece; dds keeps running so the PC never re-discovers
./robotctl down [window|all]
./robotctl attach                          # live Pi output + teleop keyboard (Ctrl-a d to leave)
./robotctl status                          # per-window state + alias IPs on BOTH sides + PC-visible topics
./robotctl log bringup [lines]             # capture-pane, no attach needed
./robotctl args "use_fake_odom:=true"      # then: ./robotctl restart bringup
./robotctl shell
```

It targets `lego@10.77.0.2` — a constant, so there is never an IP to look up. It requires passwordless ssh
(`ssh-copy-id lego@10.77.0.2`) and diagnoses the link itself before every command (ping → ssh → alias
present on each side), which is why `status` reports Pi state and laptop state separately: "the Pi says the
node is alive" and "the laptop can actually see its topics" are different failures.

### Where teleop runs, and why

**Keyboard teleop runs on the Pi**, and you type into it over ssh from the laptop's `robot` window. This is
the fix for the "cmd_vel 資料怪怪的 / 一頓一頓" symptom: previously teleop ran on the laptop and `/cmd_vel`
crossed WiFi, so an ordinary transport stall longer than the driver's `cmd_vel_timeout` tripped the watchdog,
zeroed the base, and then resumed — a stutter with no mechanical cause. With teleop on the Pi, `/cmd_vel`
never leaves the machine; keystrokes travel over ssh (TCP, reliable, tiny), so a WiFi hiccup delays a
keypress instead of stopping the robot. Two belt-and-braces changes back this up for future PC-side
publishers (nav2): `cmd_vel_timeout` 0.5 → 1.0 s, and `/cmd_vel` QoS is now **BEST_EFFORT depth-1** on both
the driver's subscription and the teleop publisher (`cmd_vel_best_effort:=false` to force RELIABLE).
Quantify any of this with `tools/cmd_vel_check.py` run on the Pi.

### PC side pieces (still usable standalone)

`./run_slam.sh` runs `slam_pc.launch.py`, which as of 2026-07-27 starts **`async_slam_toolbox_node` *and*
RViz together** (`use_rviz`, default true) — they were never useful separately, and splitting them cost an
extra terminal. RViz loads `rviz/view_robot.rviz` (Grid, RobotModel, LaserScan, Map with Durability
**Transient Local** so the latched map arrives, Odometry, TF; Fixed Frame `map`), so no displays need adding
by hand. Closing the RViz window does **not** stop SLAM — reopen with `./run_rviz.sh`, which still exists for
exactly that. Scan matching lives on the PC because the Pi 4 couldn't keep up with 10 Hz scans alongside
everything else (dropped scans + drifting odom produced a rotating "fan smear" map); `run_slam.sh` kills a
stale instance first so `map->odom` isn't published twice. `./save_map.sh <name>` wraps `map_saver_cli` with
`save_map_timeout:=10.0` — the default ~2 s often misses the latched `/map`; bare names land in `maps/`.
All three need `car_assemble_description` built locally on the PC (for `package://` mesh paths and the in-repo
`mapper_params_online_async.yaml`) plus `ros-humble-slam-toolbox` — but not `sllidar_ros2`, which stays
Pi-only. Each aborts if `dds/setup_dds.sh` didn't configure a profile.

### Autonomous navigation (Nav2) — the other PC-side mode

`./run_nav2.sh [map] [launch args…]` (PC side; default map `201_self_test`, bare names resolve under `maps/`)
is the **alternative to `./run_slam.sh`**, not an addition to it: SLAM builds a map, Nav2 drives on a finished
one, and both publish `map->odom`, so the script pkills a stale `async_slam_toolbox_node` (and a stale
`nav2_container`) before starting. Needs `ros-humble-navigation2` + `ros-humble-nav2-bringup` on the PC on top
of the usual `car_assemble_description` build. Operating it: RViz comes up with `rviz/view_nav2.rviz`; if the
robot isn't where the map says, fix it with **2D Pose Estimate**, then click **Nav2 Goal** and drag a heading.
`amcl`'s `set_initial_pose` is on with pose `(0,0,0)`, which is correct *only* if the robot starts where the
map's origin was recorded (i.e. where SLAM was started) — put it back on that spot and the pose-estimate step
is unnecessary.

Three operational gotchas, all of which look like something other than what they are:

- **Stop the Pi's keyboard teleop first** (`./robotctl down teleop`). `teleop_node` re-publishes its current
  `Twist` every loop to keep the driver's watchdog fed, so while idle it streams zeros at 20 Hz; interleaved
  with Nav2's commands the robot twitches and barely moves. `run_nav2.sh` prints this reminder rather than
  killing the remote window itself.
- **`/cmd_vel` now crosses WiFi in the PC→Pi direction** — the exact path that moving teleop onto the Pi was
  meant to avoid (see "Where teleop runs, and why"). The existing mitigations carry over (`cmd_vel_timeout`
  1.0 s, BEST_EFFORT depth-1) and `velocity_smoother` publishes a steady 20 Hz, but if it stutters, measure
  with `tools/cmd_vel_check.py` on the Pi instead of guessing. Useful topic detail: `controller_server`
  actually publishes `/cmd_vel_nav`, and nav2_bringup remaps `velocity_smoother`'s output to `/cmd_vel` — so
  `/cmd_vel` is what reaches the base, `/cmd_vel_nav` is the raw planner output.
- **A SLAM instance on the *other* machine is invisible to the local pkill.** DDS is network-wide: hit on
  2026-07-28 while testing, with `run_slam.sh` still up on the PC — its live growing `/map` (168×142) and
  `map_server`'s saved map (168×104) were both being fed to AMCL, and both nodes were publishing `map->odom`.
  Nav2 logs nothing at all; localization is just inexplicably bad. `run_nav2.sh` now asks
  `ros2 topic info /map` *before* starting its own `map_server` — any publisher at that moment is somebody
  else — and warns. Also note nav2's `component_container_isolated` **ignores SIGTERM**: an orphan survived
  ~20 min after its launch was killed, ate the Pi's CPU, and made the next start sit silent for 46 s with no
  log output, which reads exactly like a broken config. `run_nav2.sh` escalates to SIGKILL for this reason,
  and its kill pattern is `component_container_isolated.*nav2_container`, not the bare `nav2_container` —
  the short pattern matches any command line containing that string, including the operator's own shell.

Tuning order when something goes wrong: "planner says no path" is almost always costmap geometry, not the
planner — drop `inflation_radius` toward 0.13, then `robot_radius` toward 0.09, before touching anything in
`planner_server`. `allow_unknown: false` also means a goal in an unmapped pocket is unreachable by design.
**Full tuning guide: `notes/nav2_tuning.md`** — read it before changing parameters. The two things it exists to
stop you re-discovering: (1) **speed lives in three places** — `FollowPath.max_vel_*`,
`velocity_smoother.max_velocity`, and `behavior_server.max_rotational_vel` — and `velocity_smoother` clamps
the final `/cmd_vel` **silently**, so raising only `FollowPath` changes nothing at all and looks like the
params aren't loading (diff `/cmd_vel` against `/cmd_vel_nav` to catch it); (2) DWB's `min_speed_xy` is
**AND**-ed with `min_speed_theta` in `isValidSpeed()`, so with `min_speed_theta: 0.0` (the stock value)
`min_speed_xy` is dead code no matter what you set it to.

Post-first-drive tuning, 2026-07-28 (user reported directions correct but too slow for maze-solving, and
stalling/slipping at low speed): `max_vel_x` 0.20 → **0.35**, `max_vel_y` 0.15 → **0.25**, `max_vel_theta`
1.0 → **1.5**, accel 1.0 → 1.5, with `velocity_smoother` and `behavior_server` moved in lockstep. The stall
was addressed with `min_speed_xy` **0.06** + `min_speed_theta` **0.15** (DWB kept selecting 0.02–0.03 m/s
trajectories the N20 base can't actually execute — wheels slip, Nav2 believes it is moving, then
`progress_checker` fires) and `RotateToGoal.slowing_factor` 5.0 → **2.0** (the stock value turns the final
approach into a long crawl in exactly that dead band). `vx_samples` 10 → 12 since the velocity range widened.
Not yet re-tested on hardware at these values — if it now slips on launch/braking, lower `acc_lim_*` first,
not `max_vel_x`; mecanum roller slip is caused by acceleration and it destroys odometry.

`car_assemble_description` is not fully self-contained at runtime: `robot_bringup.launch.py` still depends on
`sllidar_ros2` for the lidar driver node, which lives in the ROS 2 workspace (`~/ros2_ws/src/`), **not in this
git repo** — plus `ominibot_driver`, which *is* in this repo (symlinked into the workspace). The
`my_robot_lidar` dependency is **gone** as of 2026-07-25: its `lidar_start.launch.py` was inlined into
`robot_bringup.launch.py` (see the `laser_*` args) so the lidar extrinsic lives under version control here
instead of in an out-of-repo file.
`slam_toolbox` itself must be installed on whichever machine runs SLAM (PC by default, or the Pi if
`use_slam:=true`), but its config now ships inside this repo. `robot_bringup.launch.py` launches
`ominibot_driver` when `use_fake_odom:=false` and the fake static `odom->base_link` TF otherwise; the two are
mutually exclusive (both publish that same TF). The package is symlinked into `~/ros2_ws/src/` and built with
`colcon build --symlink-install`, so editing `launch/`, `rviz/`, `config/` needs no rebuild; only
`package.xml`/`CMakeLists.txt` changes do.

Both machines must share `ROS_DOMAIN_ID` and `source dds/setup_dds.sh` — the **same command on both**, with no
`DDS_SERVER` argument, since 2026-07-27 (it exports `ROS_LOCALHOST_ONLY=0` and `RMW_IMPLEMENTATION` too). The
prerequisite is that each machine holds its fixed alias IP; see the `net/` bullets above. This gives them
**two** independent fixes that the two-machine link needs, and both are mandatory:
- **Discovery** goes through the Pi's Fast DDS **Discovery Server** (unicast), because the venue WiFi AP does
  not forward multicast between clients and DDS's default discovery is multicast-based. Symptom when this is
  the problem: `ros2 node list` on each machine shows only its **own** nodes even though `ping` works,
  `ROS_DOMAIN_ID` matches, the whitelist is correct, and the firewall is off. Confirm multicast is the culprit
  with `ros2 multicast receive` (PC) + `ros2 multicast send` (Pi) — no datagram arrives.
- **Data path** is pinned to the LAN by the interfaceWhiteList, because the Pi/PC both also have `tailscale0`
  (+ `docker0`); without it, large samples (`/robot_description`, `/tf`, `/map`) route over tailscale's
  1280-MTU link and fragment-drop while small discovery packets still get through.

When debugging connectivity, "topic appears in `list`" ≠ "data is arriving" — confirm with `ros2 topic echo
/robot_description --once` actually printing. Remember the **ros2 daemon caches discovery config**: if things
look wrong right after re-sourcing, `ros2 daemon stop` and retry (`setup_dds.sh` does this automatically).

**`ros2 topic list` used to be blind on the Pi; that is fixed as of 2026-07-26** — the cause was
`<discoveryProtocol>CLIENT</discoveryProtocol>` in the DDS profile. A Discovery Server only forwards to a
CLIENT the discovery data that *matches that client's own endpoints*; `ros2 topic list` subscribes to nothing,
so it was told nothing and listed only its own `/parameter_events` and `/rosout` — while a plain rclpy
subscriber in the same shell happily received 62 `/odom` messages in 5 s. The profile now uses
**`SUPER_CLIENT`**, which receives the server's *complete* discovery database; `ros2 topic list` immediately
returns all 11 topics. The extra discovery traffic is negligible at this scale. Note the earlier conclusion
recorded here — "CLI graph introspection is simply broken under the Discovery Server, do not treat an empty
list as a fault" — was **wrong**, and cost several debugging sessions of flying blind on the Pi.
Set RViz Fixed Frame to `base_link` first (SLAM takes ~10-15s to create the `map` frame; `map` before then
reads as a blank "does not exist" screen).
