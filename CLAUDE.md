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
- `OminiBotHV-master/` — vendor (CircusPi) driver package for the OminiBotHV motor/IMU controller board.
  - `example/OminiBot_HV_Meca.py` — reference Python driver (`ominibothv` class) showing the serial protocol:
    frames are `\x7b <cmd> ... <bcc> \x7d` with a big-endian XOR checksum (`calculate_bcc`). Key methods:
    `robot_speed(lx, ly, az)` (mecanum body-frame velocity command), `motor_speed(m1..m4)` (per-wheel), and
    `read_robot_data()` (velocity + IMU quaternion + battery voltage feedback frame).
  - `firmware/` — prebuilt STM32F1 `.hex` firmware for the board (not built from source in this repo).
  - `communication/` — PDF spec for the serial protocol used by the driver above.
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
