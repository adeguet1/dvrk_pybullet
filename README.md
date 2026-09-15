# dvrk_pybullet

PyBullet implementation of the backend contracts defined by
`dvrk_simulator_base`. The package will load robot assets from `dvrk_model` and
must not depend on SurRoL.

PyBullet is intentionally loaded only when the backend starts. Install it in a
system-site-packages virtual environment:

```shell
./src/dvrk/dvrk_pybullet/scripts/bootstrap_venv.sh
source .venv/bin/activate
hash -r
command -v colcon
```

The bootstrap creates or reuses `<workspace>/.venv`, installs PyBullet, and
places a `colcon` wrapper inside the venv. Consequently, bare `colcon build`
uses the venv interpreter and generated ROS executables retain access to
PyBullet. It is safe to run repeatedly. Use `--python PATH` to select a
different bootstrap interpreter.

The simulator currently supports shared-world kinematic PSM and ECM models,
CRTK ROS interfaces, and an ECM optical camera exported through GStreamer's
Unix-FD transport.

## Model generation cache

`dvrk_model` is located only through the ROS 2 ament index. The Virtual PSM
Xacro is expanded and its `package://dvrk_model/...` resources are converted to
validated absolute paths for PyBullet. Generated URDF and metadata files live
outside `src`:

```text
<workspace>/.generated/pybullet/<content-hash>/
├── model.urdf
└── metadata.json
```

The content-addressed entry is reused while the expanded model, instrument,
parent link, and materializer version remain unchanged.

## Preview PSM1

Build and source the two packages from the workspace, activate the environment
containing PyBullet, and open the GUI:

```shell
cd ~/wss/dvrk
colcon build --symlink-install \
  --packages-select dvrk_simulator_base dvrk_pybullet
source install/setup.bash
ros2 run dvrk_pybullet dvrk_pybullet_preview --model PSM1 --instrument 420006
```

The preview loads the Virtual PSM at its configured home position and remains
open until the window is closed or Ctrl-C is pressed. Use `--duration 10` for
an automatically closing ten-second preview.

## ROS simulator node

Run the PyBullet owner loop and expose the initial CRTK state and joint-command
topics:

```shell
ros2 run dvrk_pybullet simulator \
  --model PSM1 \
  --instrument 420006 \
  --gui true
```

## Multi-arm scenes

Load PSM1, PSM2, and the ECM in one shared PyBullet world:

```shell
ros2 run dvrk_pybullet simulator \
  --scene ECM_PSM1_PSM2.yaml \
  --gui true
```

Run `ros2 run dvrk_pybullet simulator --help` to see the deliberately small
set of command-line selectors. ROS 2 launch files can configure runtime
parameters such as rates and queue capacity.

## ECM camera

Camera settings live in each scene's `camera` mapping and use the same core
field names as `dvrk_isaac_sim`: `mode`, `owner`, `frame`, `width`, `height`,
`horizontal_fov_deg`, `near_clip_m`, `far_clip_m`, `encoding`,
`baseline_m`, `publish_rate_hz`, and `transports`. PyBullet currently supports
mono `rgba8` output and adds this transport-specific section:

```yaml
transports: [unixfd]
unixfd: {socket_path: /tmp/dvrk-pybullet-ECM.sock}
```

Start a scene containing an ECM, then connect a GStreamer viewer from another
terminal:

```shell
ros2 run dvrk_pybullet simulator --scene ECM_PSM1_PSM2.yaml --gui true

gst-launch-1.0 unixfdsrc socket-path=/tmp/dvrk-pybullet-ECM.sock \
  ! queue leaky=downstream max-size-buffers=1 \
  ! videoconvert ! autovideosink sync=false
```

The producer uses one Linux `memfd` per frame and a one-frame leaky queue, so a
slow or disconnected viewer cannot build an image backlog. Headless operation
uses PyBullet's EGL renderer. Set `renderer: tiny` in `pybullet.yaml` for a
CPU-rendered diagnostic run.

`ECM_PSM1_PSM2_PSM3.yaml` adds PSM3. Scene files select each robot asset and
set non-overlapping world base poses. All arms have independent command
mailboxes and CRTK interfaces, while the simulation itself advances once per
world tick.

The Virtual ECM is always controlled kinematically; no mass, motor, or PID
tuning is used. When it is present, each PSM's top-level Cartesian state and
commands use the live `ECM_view` frame. The corresponding
`/<PSM>/local/measured_cp` and `/<PSM>/local/setpoint_cp` topics remain in the
fixed PSM base frame. An explicitly `world`-framed Cartesian command remains
available for diagnostics.

`dvrk_arm_test.py` includes the selected arm in its ROS node name, so tests for
different arms can run concurrently without a manual node-name remap:

```shell
ros2 run dvrk_python dvrk_arm_test.py -a PSM1

ros2 run dvrk_python dvrk_arm_test.py -a PSM2
```

Inspect the state from another sourced terminal:

```shell
ros2 topic list | grep PSM1
ros2 topic echo /PSM1/measured_js --once
ros2 topic echo /PSM1/measured_cp --once
```

Send one direct joint setpoint (positions are radians except insertion, which is
metres):

```shell
ros2 topic pub --once /PSM1/servo_jp sensor_msgs/msg/JointState \
  "{name: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw], position: [0.2, 0.1, 0.14, 0.0, 0.2, -0.2]}"
```

Send a velocity-limited joint move and open the jaw:

```shell
ros2 topic pub --once /PSM1/move_jp sensor_msgs/msg/JointState \
  "{name: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw], position: [-0.2, 0.15, 0.10, 0.3, -0.2, 0.2]}"

ros2 topic pub --once /PSM1/jaw/move_jp sensor_msgs/msg/JointState \
  "{position: [0.7]}"
```

Cartesian servo and move commands use world-frame poses when running a single
arm without an ECM reference:

```shell
ros2 topic pub --once /PSM1/servo_cp geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.01, y: 0.0, z: -0.1237}, orientation: {x: 0.7071, y: 0.7071, z: 0.0, w: 0.0}}}"
```

The PyBullet backend solves all six pose coordinates numerically using its own
forward kinematics. This keeps the URDF mimic joints constrained by the logical
PSM joints during IK.

Motion is accepted while the arm is enabled and homed. The node starts in that
state. Operating-state commands can be tested with:

```shell
ros2 topic pub --once /PSM1/state_command crtk_msgs/msg/StringStamped \
  "{string: pause}"
ros2 topic pub --once /PSM1/state_command crtk_msgs/msg/StringStamped \
  "{string: resume}"
```

`servo_jp` commands supersede older pending servo commands; `move_jp` and state
commands use a bounded ordered queue. Joint and jaw limits are checked before a
command is applied. Moves are synchronized linear trajectories using the
configured velocity limits, and the operating-state `is_busy` flag covers the
move. The jaw command drives the URDF's named mimic joints.

Control is deliberately kinematic at this milestone: setpoints are applied with
PyBullet joint resets. No link masses, motor gains, or PID tuning are required
until the backend advances to dynamic control. ROS callbacks only validate and
enqueue commands; all PyBullet calls remain on the owner thread.
