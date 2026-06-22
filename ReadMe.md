# ReachyController

A Python SDK wrapper and scripting framework for the [Reachy 2021](https://pollen-robotics.github.io/reachy-2021-docs/) humanoid robot by [Pollen Robotics](https://www.pollen-robotics.com/). It provides high-level control over every actuated part of the robot (arms, head, antennas, fans), a collision-detection system that prevents arm-arm and arm-torso/table collisions, a motion recording and playback pipeline, and a YAML-based scripting language that lets you write robot behaviours without any Python.

## Table of Contents

1. [What Is This?](#what-is-this)
2. [Target Robot: Reachy 2021](#target-robot-reachy-2021)
3. [Project Summary](#project-summary)
4. [Getting Started](#getting-started)
   - [Prerequisites](#prerequisites)
   - [Installation](#installation)
   - [Connecting to the Robot](#connecting-to-the-robot)
5. [Quick-Start Guide](#quick-start-guide)
   - [Basic Arm Movement](#basic-arm-movement)
   - [Head Control](#head-control)
   - [Recording and Playback](#recording-and-playback)
   - [Recording with Parallel Threads](#recording-with-parallel-threads)
   - [Running a YAML Script](#running-a-yaml-script)
6. [File Tree](#file-tree)
7. [Collision System](#collision-system)
   - [How It Works](#how-it-works)
   - [Arm Capsule Model](#arm-capsule-model)
   - [Table Collision](#table-collision)
   - [What Happens on Collision Detection](#what-happens-on-collision-detection)
   - [Handling a Blocked Arm](#handling-a-blocked-arm)
8. [TimeSeries Format](#timeseries-format)
   - [JSON Format](#json-format)
   - [CSV Format](#csv-format)
9. [YAML Scripting Language (`.ryi`)](#yaml-scripting-language-ryi)
   - [File Format](#file-format)
   - [Expressions](#expressions)
   - [Variables and Capture](#variables-and-capture)
   - [Control Flow](#control-flow)
   - [Recording Example in YAML](#recording-example-in-yaml)
   - [Full Action Reference](#full-action-reference)
10. [Complete API Reference](#complete-api-reference)
    - [ReachyController](#reachycontroller-1)
    - [ReachyArm](#reachyarm-1)
    - [ReachyHead](#reachyhead-1)
    - [ReachyAntenna](#reachyantenna-1)
    - [ReachyTorso](#reachytorso-1)
    - [ReachyFan](#reachyfan-1)
    - [CollisionManager](#collisionmanager-1)
    - [CapsuleCollider](#capsulecollider-1)
    - [TableCollider](#tablecollider-1)
    - [TimeSeries](#timeseries-1)
    - [Instructor / Executor](#instructor--executor)
    - [Validator](#validator)
    - [ExpressionEvaluator](#expressionevaluator)
    - [ActionRegistry](#actionregistry)
    - [consoleManager](#consolemanager)
    - [config](#config)
11. [Acknowledgements](#acknowledgements)

---

## What Is This?

ReachyController is a robotics application framework that sits on top of the official `reachy-sdk`. It solves three problems that the bare SDK leaves to the developer:

**Safety** — Every arm movement is broken into configurable sub-steps and checked against a capsule-based collision model before each step executes. The robot will refuse to move if the planned pose would cause the arm to hit the other arm, the torso, or a user-defined table volume.

**Expressiveness** — Motion sequences can be written entirely in YAML (`.ryi` files), including arithmetic expressions, variables, conditions, loops, parallel execution, and recording/playback, with no Python required.

**Reusability** — Recordings are stored as `TimeSeries` objects that support concatenation, reversal, speed scaling, smoothing, noise injection, and DBA-based averaging across multiple takes.

---

## Target Robot: Reachy 2021

This framework targets the **Reachy 2021 Full Kit** by [Pollen Robotics](https://www.pollen-robotics.com/). The full documentation for the robot is available at [pollen-robotics.github.io/reachy-2021-docs](https://pollen-robotics.github.io/reachy-2021-docs/).

**Hardware overview:**

| Part | Description |
|---|---|
| Arms | 2 × 7-DOF arms (shoulder pitch/roll, arm yaw, elbow pitch, forearm yaw, wrist pitch/roll) + gripper |
| Head | 3-DOF Orbita neck (roll, pitch, yaw) + 2 motorised cameras |
| Antennas | 2 × 1-DOF decorative antennas on the head |
| Compute | Intel NUC embedded computer running Ubuntu + ROS 2 Foxy |
| SDK | `reachy-sdk` Python library communicating over gRPC |

**Coordinate frame (robot frame):**

```
X = forward (in front of the robot)
Y = left
Z = up

Origin = torso centre at shoulder height
```

**SDK version targeted:** `reachy-sdk==0.5.4`

For full mechanical specifications, refer to the [Reachy 2021 arm specs](https://pollen-robotics.github.io/reachy-2021-docs/advanced/specifications/arm-specs/) and [head specs](https://pollen-robotics.github.io/reachy-2021-docs/advanced/specifications/head-specs/).

---

## Project Summary

| Module | Purpose |
|---|---|
| `reachyController.py` | Top-level facade; instantiates all subsystems |
| `armController.py` | Per-arm motion, FK, collision, record/play |
| `headController.py` | Head orientation, look-at, record/play |
| `antennaController.py` | Individual antenna positioning and vibration |
| `torsoController.py` | Torso collision shape |
| `fanController.py` | Fan state and temperature-triggered auto-control |
| `collisionManager.py` | Arm-arm and arm-torso collision arbitration |
| `capsuleCollider.py` | Capsule geometry primitive |
| `tableCollider.py` | AABB table geometry primitive |
| `timeSeries.py` | Motion recording container and manipulation |
| `DBA_multivariate.py` | DTW Barycenter Averaging for motion averaging |
| `instructor.py` | YAML script parser, executor, and expression engine |
| `action.py` | All built-in YAML actions |
| `actionRegistry.py` | Decorator-based action registration |
| `reachyPart.py` | Base class for robot parts |
| `consoleManager.py` | Coloured timestamped logging |
| `config.py` | Motor names, geometry constants, thresholds |

---

## Getting Started

### Prerequisites

- Python 3.8+
- A Reachy 2021 robot accessible on the network (or `localhost`)
- The packages listed in `requirements.txt`

### Installation

```bash
# Clone or copy this package into your project
git clone <your-repo-url> reachycontroller

# Install dependencies
pip install -r reachycontroller/requirements.txt
```

`requirements.txt` contents:

```
matplotlib==3.5.1
numpy==1.21.0
pandas==1.4.1
PyYAML==5.3.1
reachy-sdk==0.5.4
scipy==1.7.3
```

### Connecting to the Robot

```python
import reachycontroller as rc

# Connect to a robot at a specific IP address
reachy = rc.ReachyController.instanciate("192.168.1.42")

# Or connect to localhost (simulation / robot with direct access)
reachy = rc.ReachyController.instanciate()
```

---

## Quick-Start Guide

### Basic Arm Movement

```python
import reachycontroller as rc

reachy = rc.ReachyController.instanciate()
reachy.turnOnSafe()   # sync goal positions before enabling torque

# Move the right arm by joint angles (8 values, degrees)
# Order: shoulder_pitch, shoulder_roll, arm_yaw,
#        elbow_pitch, forearm_yaw, wrist_pitch, wrist_roll, gripper
joints = [0, -10, 0, -90, 0, 0, 0, 0]
target = {j: a for j, a in zip(reachy.armRight.getJointsInOrder(), joints)}
reachy.armRight.safeGoto(target, duration=2.0)

# Move to a Cartesian position [x, y, z] with Euler rotation [rx, ry, rz] (degrees)
reachy.armLeft.gotoCartesianPoint([0.3, 0.2, -0.3], [0, 90, 0], duration=1.5)

# Open / close gripper
reachy.armRight.openHand(duration=0.5)
reachy.armLeft.closeHand(duration=0.5)

reachy.turnOffSmooth()
```

### Head Control

```python
# Look at a world-space point [x, y, z]
reachy.head.lookAt([1.0, 0.0, 0.0], duration=1.0)   # look straight ahead

# Get the point the head is currently looking at (1 metre away)
point = reachy.head.forwardKinematic(distance=1.0)
```

### Recording and Playback

```python
# Record all parts for 5 seconds at 30 fps
record = reachy.record(recordDurationSeconds=5.0, samplingFrequencyHertz=30.0)

# Save and reload
record.saveToJson("my_motion.json")
record.saveToCSV("my_motion.csv")

loaded = rc.TimeSeries.loadFromJson("my_motion.json")

# Manipulate before playback
slowed    = loaded.speed(0.5)       # half speed
looped    = loaded * 3              # repeat 3 times
reversed_ = loaded.reverse()        # play backwards
smoothed  = loaded.smooth(window=7) # smooth joint trajectories

# Play back
reachy.playRecord(loaded, startDuration=3.0)
```

### Recording with Parallel Threads

`ReachyController.record()` internally launches one thread per body part so that arms and head are captured simultaneously. If you need to record a single arm *while* the other arm executes a motion, you can orchestrate this yourself:

```python
from concurrent.futures import ThreadPoolExecutor
import reachycontroller as rc

reachy = rc.ReachyController.instanciate()
reachy.turnOnSafe()

def move_left():
    """Move left arm through a sequence while the right arm records."""
    joints = [0, 10, 0, -60, 0, 0, 0, 0]
    target = {j: a for j, a in zip(reachy.armLeft.getJointsInOrder(), joints)}
    reachy.armLeft.safeGoto(target, duration=3.0)

def record_right():
    """Record the right arm for 5 seconds."""
    return reachy.armRight.recordArm(
        recordDurationSeconds=5.0,
        samplingFrequencyHertz=30.0
    )

with ThreadPoolExecutor(max_workers=2) as pool:
    future_move   = pool.submit(move_left)
    future_record = pool.submit(record_right)
    future_move.result()
    record = future_record.result()

record.saveToJson("right_arm_capture.json")
reachy.turnOffSmooth()
```

> The same pattern is what `parallel:` does in YAML scripts (see below).

### Running a YAML Script

```python
instructor = rc.Instructor.loadFromPath("my_script.ryi", reachy)
instructor.execute()
```

Example `my_script.ryi`:

```yaml
format: reachy_instruction
version: 1.0
extension: .ryi

reachy:
  - reachy_on

  - look_at:
      target: [1.0, 0.0, 0.0]
      duration: 1.0

  - move_joints:
      arm: right
      joints: [0, -10, 0, -90, 0, 0, 0, 0]
      duration: 2.0

  - open_hand:
      arm: right

  - reachy_off
```

---

## File Tree

```
reachycontroller/
│
├── __init__.py               # Package entry point, re-exports public API
├── config.py                 # Constants: motor names, geometry, thresholds
│
├── reachyController.py       # ReachyController  — top-level facade
├── reachyPart.py             # ReachyPart        — base class
│
├── armController.py          # ReachyArm         — arm motion + FK + collision
├── headController.py         # ReachyHead        — head orientation + record
├── antennaController.py      # ReachyAntenna     — antenna positioning
├── torsoController.py        # ReachyTorso       — torso collision shape
├── fanController.py          # ReachyFan         — fan management
│
├── collisionManager.py       # CollisionManager  — inter-part collision
├── capsuleCollider.py        # CapsuleCollider   — capsule geometry
├── tableCollider.py          # TableCollider     — AABB table geometry
│
├── timeSeries.py             # TimeSeries        — motion data container
├── DBA_multivariate.py       # performDBA        — motion averaging
│
├── instructor.py             # Instructor, Executor, Validator, ExpressionEvaluator
├── action.py                 # All built-in YAML actions
├── actionRegistry.py         # ACTION_REGISTRY, decorators
│
├── consoleManager.py         # Coloured logging utilities
└── requirements.txt
```

---

## Collision System

### How It Works

Every call to `safeGoto()` divides the motion into `steps` equal sub-steps (default: 5). Before executing each sub-step, the system computes the arm's forward kinematics at the interpolated target and checks the resulting geometry against:

1. The **other arm's current or predicted position**
2. The **torso**
3. The **table volume** (if enabled)

If any check fails, the motion stops immediately at the last safe position and `canMove` is set to `False`. Subsequent calls to `safeGoto()` will do nothing until the arm is manually moved to a safe pose and `resetCanMove()` is called.

For **parallel moves** (two arms moving at the same time), each arm publishes its interpolated target as `_pendingJointDict` *before* the check runs, so the other arm can read it and verify against the predicted future state rather than the live state. This avoids false positives where two arms moving towards each other appear safe frame-by-frame but would collide mid-motion.

### Arm Capsule Model

Each arm is modelled as **3 capsules** computed from the analytical forward kinematics:

```
Capsule 0 — upper arm:  shoulder → elbow
Capsule 1 — forearm:    elbow    → wrist
Capsule 2 — hand:       wrist    → end-effector
```

A capsule is a cylinder with hemispherical end-caps, parameterised by two endpoint positions and a radius (`CAPSULE_COLLISION_RADIUS = 0.04 m`). Two capsules intersect when the minimum distance between their centre segments is less than the sum of their radii.

**Torso exclusion rule:** capsule 0 (upper arm) is excluded from arm-torso checks. When the arm hangs at its side in a neutral pose, the shoulder-to-elbow segment passes very close to the torso axis, which would generate constant false positives. Only capsules 1 and 2 (forearm and hand) are checked against the torso.

### Table Collision

The table is modelled as an **Axis-Aligned Bounding Box (AABB)**. Four key points of the arm are checked against this volume on every sub-step: elbow, forearm-yaw frame, wrist, and end-effector. The upper arm is excluded to avoid false positives when the arm is folded in a resting pose behind the table's edge.

The table geometry is defined in the robot's coordinate frame:

```
X = forward   (positive = in front of Reachy)
Y = left
Z = up        (negative = below shoulder height)
```

Default values in `config.py` assume the robot is centred on a 120 × 80 cm table positioned directly behind it:

```python
TABLE_X_MIN = -0.80   # 80 cm behind Reachy
TABLE_X_MAX =  0.00   # flush with Reachy's torso front
TABLE_Y_MIN = -0.60
TABLE_Y_MAX =  0.60
TABLE_Z_MAX = -0.40   # table surface
TABLE_Z_MIN = -1.18   # table underside
```

**Adjust these values in `config.py` to match your actual table before enabling table collision.**

Table collision is disabled by default. Enable it per arm:

```python
reachy.armRight.activateCollisionWithTable()
reachy.armLeft.activateCollisionWithTable()
```

Or dynamically set a custom table:

```python
from reachycontroller.tableCollider import TableCollider

table = TableCollider.fromSurface(
    x_min=-0.70, x_max=0.00,
    y_min=-0.50, y_max=0.50,
    z_surface=-0.42,   # surface of the table in robot frame
    thickness=0.08
)
reachy.armRight.activateCollisionWithTable(table)
reachy.armLeft.activateCollisionWithTable(table)
```

### What Happens on Collision Detection

When a collision is detected during `safeGoto()`:

1. The motion stops at the last safe sub-step.
2. A `[SAFETY]` message is printed to the console (red).
3. `arm.canMove` is set to `False`.
4. All subsequent `safeGoto()` calls on that arm are silently ignored until `resetCanMove()` is called.

### Handling a Blocked Arm

> **There is no automatic recovery.** The arm must be unblocked manually.

If an arm stops due to a detected collision:

1. **Check the console output** to identify which collision was detected (arm-arm, arm-torso, or arm-table).
2. **Turn off the motors** with `reachy.turnOffSmooth()` so the arm can be moved passively.
3. **Physically move the arm** to a safe, clear position away from the obstacle.
4. **Turn the motors back on** with `reachy.turnOnSafe()`.
5. **Reset the flag** on the affected arm(s):

```python
reachy.armRight.resetCanMove()
reachy.armLeft.resetCanMove()
```

6. You can now send motion commands again.

> Do not call `resetCanMove()` without first manually moving the arm to a safe pose. Doing so will cause the arm to immediately re-trigger the collision on the next motion command, or worse, execute a motion that damages the robot.

---

## TimeSeries Format

`TimeSeries` objects can be serialised to JSON or CSV. Both formats store the same data; JSON is preferable for programmatic use, CSV for inspection in a spreadsheet.

### JSON Format

```json
{
    "samplingFrequency": 30.0,
    "recordDuration": 5.0,
    "flags": [true, true, true],
    "jointPosition": [
        {
            "r_shoulder_pitch": -2.1,
            "r_shoulder_roll": -8.5,
            "r_arm_yaw": 0.3,
            "r_elbow_pitch": -88.0,
            "r_forearm_yaw": 1.2,
            "r_wrist_pitch": 0.0,
            "r_wrist_roll": -0.5,
            "r_gripper": 5.0,
            "l_shoulder_pitch": -1.0,
            "l_shoulder_roll": 9.0,
            "l_arm_yaw": -0.2,
            "l_elbow_pitch": -91.0,
            "l_forearm_yaw": 0.0,
            "l_wrist_pitch": 0.1,
            "l_wrist_roll": 0.3,
            "l_gripper": -4.0,
            "head_x": 0.98,
            "head_y": 0.05,
            "head_z": -0.12
        },
        ...
    ]
}
```

**Fields:**

| Field | Type | Description |
|---|---|---|
| `samplingFrequency` | float | Frames per second at which the recording was captured |
| `recordDuration` | float | Total duration in seconds |
| `flags` | `[bool, bool, bool]` | Which parts are present: `[arm_right, arm_left, head]` |
| `jointPosition` | list of dicts | One dict per frame; keys are motor names |

**Motor name conventions:**

- Right arm: `r_<motor>` (e.g. `r_shoulder_pitch`)
- Left arm: `l_<motor>` (e.g. `l_elbow_pitch`)
- Head gaze point: `head_x`, `head_y`, `head_z` (3D look-at point, not disk angles)

### CSV Format

The CSV has a fixed header row followed by one row per frame:

```
frame,timestamp,r_shoulder_pitch,r_shoulder_roll,r_arm_yaw,r_elbow_pitch,r_forearm_yaw,r_wrist_pitch,r_wrist_roll,r_gripper,l_shoulder_pitch,...,head_x,head_y,head_z
0,0.0,-2.1,-8.5,0.3,-88.0,1.2,0.0,-0.5,5.0,-1.0,...,0.98,0.05,-0.12
1,0.033,-2.0,-8.4,0.3,-88.1,1.2,0.0,-0.5,5.0,-1.0,...,0.98,0.05,-0.12
...
```

**Columns:**

| Column | Description |
|---|---|
| `frame` | Integer frame index starting at 0 |
| `timestamp` | Time in seconds (`frame / samplingFrequency`) |
| Motor columns | Angle in degrees for each joint, in the order defined by `TimeSeries.jointLabel` |

Missing joints (when only some parts were recorded) are written as `0`. The `flags` field is not stored in CSV; it is inferred as `[True, True, True]` on load.

> When loading from CSV, `samplingFrequency` and `recordDuration` are recovered from the `frame` and `timestamp` columns of the last row.

---

## YAML Scripting Language (`.ryi`)

### File Format

A `.ryi` file is a YAML file with a required header and a `reachy` key whose value is a list of instructions:

```yaml
format: reachy_instruction
version: 1.0
extension: .ryi

reachy:
  - reachy_on
  - <action_name>:
      <param>: <value>
  - reachy_off
```

Plain `.yml` lists (without the header) are also accepted.

### Expressions

Any parameter value can be replaced with an arithmetic or vector expression:

```yaml
duration:
  add: [1, 2]           # → 3.0

joints:
  - mul: [5, 4]         # → 20
  - 0
  - 0

position:
  vec_add:
    - [0.2, 0.1, -0.3]
    - [0.0, 0.1,  0.0]  # → [0.2, 0.2, -0.3]
```

**Scalar operators:** `add`, `sub`, `mul`, `div`, `mod`, `pow`, `abs`, `min`, `max`, `clamp`

**Comparison operators:** `eq`, `neq`, `gt`, `gte`, `lt`, `lte`

**Logical operators:** `and`, `or`, `not`

**Vector operators:** `vec_add`, `vec_sub`, `distance`, `length`, `normalize`

### Variables and Capture

```yaml
- capture:
    as: my_var
    value:
      mul: [3, 4]        # stores 12 in $my_var

- capture:
    as: record
    action:
      record_all:
        duration: 5
        fps: 30           # stores a TimeSeries in $record

- print:
    message: $my_var     # reference a variable with $
```

### Control Flow

```yaml
# Loop N times
- do:
    times: 3
    actions:
      - look_at:
          target: [1, 0, 0]

# Conditional
- if:
    condition:
      gt: [$speed, 2]
    actions:
      - print:
          message: "fast!"

# If / else
- if_else:
    condition:
      eq: [$mode, 1]
    then:
      - open_hand:
          arm: right
    else:
      - close_hand:
          arm: right

# While loop
- while:
    condition:
      lt: [$counter, 10]
    actions:
      - <...>

# Wait
- wait:
    duration: 2.0

# Break / continue (inside do or while)
- break
- continue
```

### Recording Example in YAML

The following script records both arms and the head simultaneously while the left arm executes a motion, then saves, reloads, and plays the result back:

```yaml
format: reachy_instruction
version: 1.0
extension: .ryi

reachy:
  - reachy_on

  # Run recording and arm motion in parallel threads
  - parallel:
      # Thread 1 — capture all parts for 5 seconds
      - capture:
          as: demo_record
          action:
            record_all:
              duration: 5
              fps: 30
              arm_right: true
              arm_left: true
              head: true

      # Thread 2 — move the right arm during the recording
      - move_joints:
          arm: right
          joints: [0, -10, 0, -90, 0, 0, 0, 0]
          duration: 4.0

      # Thread 3 — look somewhere interesting
      - look_at:
          target: [0.5, 0.3, 0.0]
          duration: 2.0

  # Save to JSON
  - save_record_as_JSON:
      file_name: demo.json
      record: $demo_record

  # Reload from disk
  - capture:
      as: loaded
      action:
        load_record_from_JSON:
          file_name: demo.json

  # Play back (3 seconds to reach first pose, then real-time)
  - play_record_all:
      record: $loaded
      start_duration: 3.0

  - reachy_off
```

### Full Action Reference

| Action | Required params | Optional params | Returns |
|---|---|---|---|
| `reachy_on` | — | — | — |
| `reachy_off` | — | — | — |
| `look_at` | `target` (vec3) | `duration` | — |
| `move_hand` | `arm`, `position` (vec3), `orientation` (vec3 Euler °) | `duration`, `interpolation` | — |
| `move_joints` | `arm`, `joints` (8 floats °), `duration` | `collision_check_number`, `interpolation` | — |
| `move_hand_sequence` | `arm`, `positions` (list of vec3), `duration` | `step_duration`, `orientation` | — |
| `place_hand_on_table` | `arm` | `duration` | — |
| `open_hand` | `arm` | `duration` | — |
| `close_hand` | `arm` | `duration` | — |
| `get_joint_positions` | `arm` | — | joint values |
| `get_hand_position` | `arm` | — | vec3 |
| `get_look_position` | — | — | vec3 |
| `set_antenna` | `antenna` (`left`/`right`), `angle` (°) | `duration` | — |
| `vibrate_antenna` | `antenna` | `amplitude`, `cycles`, `speed` | — |
| `record_all` | `duration`, `fps` | `head`, `arm_right`, `arm_left` (bool) | TimeSeries |
| `record_arm` | `arm`, `duration`, `fps` | — | TimeSeries |
| `record_head` | `duration`, `fps` | — | TimeSeries |
| `play_record_all` | `record` | `start_duration` | — |
| `play_record_arm` | `record`, `arm` | `start_duration`, `collision_check_number` | — |
| `play_record_head` | `record` | `start_duration` | — |
| `save_record_as_CSV` | `file_name`, `record` | — | — |
| `load_record_from_CSV` | `file_name` | — | TimeSeries |
| `save_record_as_JSON` | `file_name`, `record` | — | — |
| `load_record_from_JSON` | `file_name` | — | TimeSeries |
| `enable_table_collision` | `arm` | — | — |
| `disable_table_collision` | `arm` | — | — |
| `set_table` | `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max` | `arm` | — |
| `set_table_from_surface` | `x_min`, `x_max`, `y_min`, `y_max`, `z_surface` | `thickness`, `arm` | — |
| `capture` | `as` | `value`, `action` | stored value |
| `condition` | (expression dict) | — | bool |
| `print` | `message` | `type` (`debug`/`safety`/`warning`) | — |
| `do` | `times`, `actions` | — | — |
| `if` | `condition`, `actions` | — | — |
| `if_else` | `condition`, `then`, `else` | — | — |
| `while` | `condition`, `actions` | — | — |
| `wait` | `duration` | — | — |
| `parallel` | (list of actions) | — | — |
| `break` | — | — | — |
| `continue` | — | — | — |

---

## Complete API Reference

### ReachyController

Top-level facade. Instantiates all subsystems and exposes them as attributes.

**Attributes**

| Attribute | Type | Description |
|---|---|---|
| `armLeft` | `ReachyArm` | Left arm controller |
| `armRight` | `ReachyArm` | Right arm controller |
| `head` | `ReachyHead` | Head controller |
| `torso` | `ReachyTorso` | Torso collision shape |
| `fans` | `ReachyFan` | Fan controller |
| `collision` | `CollisionManager` | Collision arbitration |

---

#### `ReachyController.instanciate(ip="localhost") -> ReachyController`

Class method. Connects to a Reachy robot and returns a fully initialised `ReachyController`.

```python
reachy = ReachyController.instanciate("192.168.1.10")
```

---

#### `ReachyController.__init__(reachy: ReachySDK)`

Direct constructor. Prefer `instanciate()` in normal usage.

---

#### `ReachyController.turnOn() -> None`

Turns on all Reachy motors. May cause snapping if goal positions are not synced — prefer `turnOnSafe()`.

---

#### `ReachyController.turnOffSmooth() -> None`

Smoothly turns off all Reachy motors and switches all fans off.

---

#### `ReachyController.turnOnSafe() -> None`

Turns on motors and syncs all `goal_position` to `present_position` before enabling torque, preventing sudden snapping.

---

#### `ReachyController.record(recordDurationSeconds, samplingFrequencyHertz, recordArmLeft=True, recordArmRight=True, recordHead=True) -> TimeSeries`

Records all selected parts in parallel using a `ThreadPoolExecutor`. Returns a merged `TimeSeries`.

```python
record = reachy.record(5.0, 30.0)                        # all parts
record = reachy.record(5.0, 30.0, recordHead=False)      # arms only
```

---

#### `ReachyController.playRecord(records: TimeSeries, startDuration=3.0) -> None`

Plays a `TimeSeries` on all parts simultaneously using a `ThreadPoolExecutor`. `startDuration` is the time in seconds to interpolate from the current pose to the first frame.

---

### ReachyArm

Controls one arm. Accessed via `reachy.armLeft` or `reachy.armRight`.

**Joint order** (used by `safeGoto`, `move_joints`, etc.):

```
shoulder_pitch → shoulder_roll → arm_yaw →
elbow_pitch → forearm_yaw →
wrist_pitch → wrist_roll → gripper
```

**Joint limits (right arm)**

| Joint | Min (°) | Max (°) |
|---|---|---|
| shoulder_pitch | −150 | 90 |
| shoulder_roll | −180 | 10 |
| arm_yaw | −90 | 90 |
| elbow_pitch | −125 | 0 |
| forearm_yaw | −100 | 100 |
| wrist_pitch | −45 | 45 |
| wrist_roll | −55 | 35 |
| gripper | −69 | 20 |

Left arm has mirrored limits for `shoulder_roll` (−10 / 180), `wrist_roll` (−35 / 55), and `gripper` (−20 / 69).

---

#### `ReachyArm.safeGoto(joint_dict, duration, interpolation=linear, steps=5) -> None`

Moves to the target joint configuration with collision checking at each sub-step. `joint_dict` maps SDK joint objects to target angles in degrees. Angles are clamped to physical limits automatically.

```python
joints = reachy.armRight.getJointsInOrder()
target = {j: a for j, a in zip(joints, [0, -10, 0, -90, 0, 0, 0, 0])}
reachy.armRight.safeGoto(target, duration=2.0, steps=10)
```

---

#### `ReachyArm.gotoCartesianPoint(goalPosition, goalRotation, duration=0.1, interpolation=linear) -> None`

Moves the end-effector to a Cartesian position using the SDK's inverse kinematics, then executes the resulting joint configuration with `safeGoto`.

- `goalPosition`: `[x, y, z]` in metres, robot frame
- `goalRotation`: `[rx, ry, rz]` Euler angles in degrees (XYZ convention)

---

#### `ReachyArm.openHand(duration=0.5) -> None`

Opens the gripper fully.

---

#### `ReachyArm.closeHand(duration=0.5) -> None`

Closes the gripper fully.

---

#### `ReachyArm.changeHandAngle(angleEuler, duration) -> None`

Sets the gripper to a specific angle (degrees). Clamped to joint limits.

---

#### `ReachyArm.getJointsInOrder() -> list`

Returns the 8 SDK joint objects in chain order. Use this to build `joint_dict` for `safeGoto`.

---

#### `ReachyArm.getArmId() -> str`

Returns `"r"` for right arm, `"l"` for left arm.

---

#### `ReachyArm.resetCanMove() -> None`

Clears the `canMove = False` flag that is set when a collision is detected. Must only be called after manually moving the arm to a safe pose. See [Handling a Blocked Arm](#handling-a-blocked-arm).

---

#### `ReachyArm.activateCollisionWithTable(table=None) -> None`

Enables table collision checking. If `table` is `None`, uses the AABB from `config.py`. Pass a `TableCollider` instance to use a custom table.

---

#### `ReachyArm.desactivateCollisionWithTable() -> None`

Disables table collision checking without discarding the current `TableCollider`.

---

#### `ReachyArm.setTable(table: TableCollider) -> None`

Replaces the current table without changing whether collision checking is active.

---

#### `ReachyArm.getTable() -> TableCollider | None`

Returns the current `TableCollider`.

---

#### `ReachyArm.computeFK(joint_angles_deg=None) -> list[np.ndarray]`

Runs the analytical forward kinematics. Returns 9 world-space positions:

| Index | Meaning |
|---|---|
| 0 | World origin |
| 1 | Shoulder |
| 2 | After shoulder_roll |
| 3 | After arm_yaw |
| 4 | Elbow |
| 5 | After forearm_yaw |
| 6 | Wrist |
| 7 | After wrist_roll |
| 8 | End-effector (gripper tip) |

If `joint_angles_deg` is `None`, uses the current live joint positions.

---

#### `ReachyArm.getHandPosition() -> list`

Returns the current end-effector position `[x, y, z]` from analytical FK.

---

#### `ReachyArm.getShoulderPosition() -> list`

Returns the current shoulder position `[x, y, z]` from analytical FK.

---

#### `ReachyArm.getCollision() -> list[CapsuleCollider]`

Returns 3 capsules representing the current arm pose: upper arm, forearm, hand.

---

#### `ReachyArm.recordArm(recordDurationSeconds, samplingFrequencyHertz) -> TimeSeries`

Records this arm's joint angles for the given duration at the given frequency. Frames where all joints read exactly `0.0` are replaced with the last valid frame to handle SDK read glitches.

---

#### `ReachyArm.playArmRecord(record, startDuration=3.0, collisionCheckInterval=5) -> None`

Plays a `TimeSeries` on this arm. Collision-checks every `collisionCheckInterval` frames.

---

#### `ReachyArm.setCollisionManager(collisionManager) -> None`

Attaches a `CollisionManager`. Called automatically by `ReachyController.__init__`.

---

#### `ReachyArm.getInterpoaltionByName(name) -> trajectory.interpolation`

Looks up a `reachy_sdk.trajectory.interpolation` enum value by name (e.g. `"linear"`, `"minimum_jerk"`).

---

### ReachyHead

Controls the head orientation. Accessed via `reachy.head`.

**Attributes**

| Attribute | Type | Description |
|---|---|---|
| `antennaLeft` | `ReachyAntenna` | Left antenna |
| `antennaRight` | `ReachyAntenna` | Right antenna |
| `cameraLeft` | SDK camera | Left camera |
| `cameraRight` | SDK camera | Right camera |

---

#### `ReachyHead.lookAt(degAngles, duration=1.0) -> None`

Points the head at a 3D world-space point.

- `degAngles`: `[x, y, z]` — the point to look at

---

#### `ReachyHead.forwardKinematic(distance=1.0) -> list`

Returns the 3D world-space point the head is currently looking at, projected `distance` metres along the gaze direction.

---

#### `ReachyHead.invertKinematic(x, y, z) -> list`

Returns `[roll, pitch, yaw]` disk angles (degrees) needed to look at `[x, y, z]`.

---

#### `ReachyHead.recordHead(recordDurationSeconds, samplingFrequencyHertz) -> TimeSeries`

Records the head gaze direction (as a 3D look-at point) for the given duration.

---

#### `ReachyHead.playHeadRecord(record, startDuration=3.0) -> None`

Plays a head `TimeSeries`. Interpolates to the first frame over `startDuration` seconds, then replays each frame by writing disk goal positions directly.

---

#### `ReachyHead.setCameraZoomLevel(zoomLevel) -> None`

Sets both cameras to the given `ZoomLevel` and enables autofocus.

---

### ReachyAntenna

Controls one antenna. Accessed via `reachy.head.antennaLeft` or `reachy.head.antennaRight`.

---

#### `ReachyAntenna.setAntenna(angle, duration=0.5) -> None`

Moves the antenna to the given angle (degrees) over `duration` seconds.

---

#### `ReachyAntenna.vibrateAntenna(amplitude=15.0, cycles=3, speed=0.08) -> None`

Vibrates the antenna by oscillating ±`amplitude` degrees around its current position, for `cycles` full cycles at `speed` seconds per half-swing.

---

### ReachyTorso

Provides the torso collision shape. Accessed via `reachy.torso`. No motion methods.

---

#### `ReachyTorso.getCollision() -> list[CapsuleCollider]`

Returns a single vertical capsule centred on the torso, `TORSO_SIZE` metres tall and `TORSO_COLLISION_RADIUS` metres wide.

---

### ReachyFan

Manages cooling fans. Accessed via `reachy.fans`.

---

#### `ReachyFan.setMode(mode) -> None`

Sets all fans to `FanMode.ON` or `FanMode.OFF`.

---

#### `ReachyFan.setFan(fanName, mode) -> None`

Sets a single fan by name. Available names can be retrieved with `getFanNames()`.

---

#### `ReachyFan.turnOnAll() / turnOffAll() -> None`

Convenience wrappers around `setMode`.

---

#### `ReachyFan.enableAuto() / disableAuto() -> None`

Enables or disables automatic temperature-based fan control. When enabled, `tick()` must be called periodically.

---

#### `ReachyFan.tick() -> None`

Reads all motor temperatures and switches fans on or off based on `config.FAN_THRESHOLD`. Call this in a background loop when auto mode is active.

---

#### `ReachyFan.updateFromTemperature(temperatures, threshold=FAN_THRESHOLD) -> None`

Manually feeds a temperature dict (`{motor_name: float}`) and updates fan state.

---

#### `ReachyFan.printState() -> None`

Prints current fan states and associated motor temperatures to the console.

---

#### `ReachyFan.getFanNames() -> list`

Returns the names of all registered fans.

---

### CollisionManager

Arbitrates collisions between both arms and the torso. Instantiated automatically by `ReachyController`.

---

#### `CollisionManager.askValidMovement(armId, joint_dict) -> bool`

Checks whether a single arm's proposed pose (described by `joint_dict`) collides with the live position of the other arm or the torso. Use for single-arm moves.

---

#### `CollisionManager.askValidMovementBoth(joint_dict_right, joint_dict_left) -> tuple[bool, bool]`

Checks both arms simultaneously against each other's predicted future poses. The correct method to use for parallel moves. Either argument can be `None` to use the arm's live position. Returns `(right_ok, left_ok)`.

---

### CapsuleCollider

Geometric primitive: a capsule defined by two endpoints and a radius.

---

#### `CapsuleCollider(pointA, pointB, radius)`

Constructor. `pointA` and `pointB` are `[x, y, z]` lists or arrays; `radius` is a float in metres.

---

#### `CapsuleCollider.intersects(other) -> bool`

Returns `True` if this capsule overlaps with `other` (segment-segment distance ≤ sum of radii).

---

#### `CapsuleCollider.distanceToPoint(point) -> float`

Signed distance from `point` to the capsule surface. Negative means inside the capsule.

---

#### `CapsuleCollider.closestPoint(point) -> np.ndarray`

Closest point on the axis segment to `point`.

---

#### `CapsuleCollider.segmentSegmentDistance(A, B, C, D) -> float` *(static)*

Minimum distance between segments AB and CD.

---

### TableCollider

Axis-Aligned Bounding Box defining a table volume in the robot's coordinate frame.

---

#### `TableCollider(x_min, x_max, y_min, y_max, z_min, z_max)`

Constructor. Raises `ValueError` if any `min >= max`.

---

#### `TableCollider.fromSurface(x_min, x_max, y_min, y_max, z_surface, thickness=0.10)` *(classmethod)*

Convenience constructor. Sets `z_max = z_surface`, `z_min = z_surface - thickness`.

---

#### `TableCollider.containsPoint(point) -> bool`

Returns `True` if the point is inside the AABB.

---

#### `TableCollider.distanceToPoint(point) -> float`

Signed distance: positive = outside, negative = inside.

---

#### `TableCollider.toDict() / fromDict(d)` *(classmethod)*

Serialise/deserialise to/from a plain dict with keys `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`.

---

### TimeSeries

Container for robot motion data. Stores a list of joint-angle frames alongside sampling metadata. See [TimeSeries Format](#timeseries-format) for file formats.

**Flags** control which parts are present: `[arm_right, arm_left, head]`.

---

#### `TimeSeries(samplingFrequency, recordDurationSeconds, jointPosition=None, flags=[True,True,True])`

Constructor.

---

#### Operator overloads

| Operator | Description |
|---|---|
| `a + b` | Merge two recordings time-aligned (parallel blend) |
| `a >> b` | Concatenate: `a` then `b` |
| `a << b` | Concatenate: `b` then `a` |
| `a * n` | Repeat `a` exactly `n` times |
| `a[start:stop]` | Slice by frame index |
| `a \| b` | DBA average of `a` and `b` |
| `len(a)` | Number of frames |

---

#### `TimeSeries.reverse() -> TimeSeries`

Returns a time-reversed copy.

---

#### `TimeSeries.speed(factor) -> TimeSeries`

Resamples to a different playback speed. `factor > 1` speeds up; `factor < 1` slows down.

---

#### `TimeSeries.smooth(window=5) -> TimeSeries`

Applies a sliding-window average to all joint channels.

---

#### `TimeSeries.addWhiteNoise(amplitude=0.1) -> TimeSeries`

Adds Gaussian noise (mean 0, std `amplitude`) to every joint angle.

---

#### `TimeSeries.saveToJson(fileName) / loadFromJson(fileName)` *(classmethod)*

JSON serialisation and deserialisation.

---

#### `TimeSeries.saveToCSV(fileName) / loadFromCSV(fileName)` *(classmethod)*

CSV serialisation and deserialisation.

---

#### `TimeSeries.plot() -> None`

Opens a 3×3 matplotlib figure showing all joint channels grouped by body part.

---

#### `TimeSeries.dba(seriesList, nIterations=10) -> TimeSeries` *(static)*

Computes the DTW Barycenter Average of a list of `TimeSeries`. All series must share the same sampling frequency.

---

### Instructor / Executor

The YAML scripting engine.

---

#### `Instructor(data, reachyController)`

Constructor. `data` is a parsed list of instruction dicts.

---

#### `Instructor.loadFromPath(path, reachyController) -> Instructor` *(classmethod)*

Loads a `.ryi` or `.yml` file and returns an `Instructor`.

---

#### `Instructor.loadFromString(yamlString, reachyController) -> Instructor` *(classmethod)*

Parses a YAML string directly. Prepends a `reachy:` wrapper if absent.

---

#### `Instructor.execute() -> None`

Runs all instructions in sequence.

---

#### `Executor`

Internal class used by `Instructor`. Holds the `ReachyController` reference, the `ACTION_REGISTRY`, and the variable store.

Key method: `Executor.executeInstruction(instruction)` dispatches a single instruction dict and returns its result (if any).

---

### Validator

Fluent parameter validator used inside action handlers.

```python
if not Validator(params, "my_action").require("arm").require("duration").validate():
    return
```

#### `Validator.require(field) -> Validator`

Marks `field` as required. Prints a warning and marks the validator invalid if the field is absent.

#### `Validator.isAList() -> Validator`

Checks that the params object is a list.

#### `Validator.validate() -> bool`

Returns `True` if all checks passed.

---

### ExpressionEvaluator

Recursively evaluates expression dicts, variable references (`$name`), and literal values.

#### `ExpressionEvaluator.evaluate(executor, value) -> Any` *(static)*

Entry point. Resolves the full expression tree rooted at `value` against the executor's variable store. See [Expressions](#expressions) for supported operators.

---

### ActionRegistry

#### `@register_action("name")`

Decorator. Registers a function in `ACTION_REGISTRY` under the given name. The function must have the signature `fn(executor)` or `fn(executor, params)`.

```python
@register_action("my_action")
def my_action(executor: Executor, params: dict):
    arm = params.get("arm")
    ...
```

#### `@register_control_action("name")`

Like `register_action` but also adds the name to `CONTROL_ACTIONS`, causing the executor to skip expression evaluation on the parameters. Required for actions like `do`, `while`, and `if` that receive and evaluate their own sub-expressions.

---

### consoleManager

Timestamped, coloured console output.

#### `MKprint(printStr, instrName="default", colorID=Color.DEFAULT) -> None`

Prints a timestamped, indented message.

#### `MKprintSafety(printStr, instrName, colorID) -> None`

Prints with a red `[SAFETY]` prefix.

#### `MKprintDebug(printStr, instrName, colorID) -> None`

Prints with a yellow `[DEBUG]` prefix.

#### `MKprintWarning(printStr, instrName, colorID) -> None`

Prints with a yellow `[WARNING]` prefix.

#### `addIntentation(n=1) / removeIntentation(n=1) -> None`

Increases or decreases the global indentation level for log output.

#### `Color`

Class of ANSI escape-code constants: `RED`, `GREEN`, `YELLOW`, `BLUE`, `CYAN`, `MAGENTA`, `WHITE`, `BRIGHT_*` variants, `BOLD`, `RESET`, `SAFETY`, `DEBUG`, `WARNING`.

---

### config

Central constants file. Edit this file to match your physical setup.

| Constant | Default | Description |
|---|---|---|
| `ARM_LEFT_ID` | `"l"` | Left arm identifier prefix |
| `ARM_RIGHT_ID` | `"r"` | Right arm identifier prefix |
| `ORIGIN_TO_SHOULDER` | `0.19 m` | Lateral distance from torso centre to shoulder |
| `SHOULDER_TO_ELBOW` | `0.28 m` | Upper-arm length |
| `ELBOW_TO_WRIST` | `0.25 m` | Forearm length |
| `CAPSULE_COLLISION_RADIUS` | `0.04 m` | Arm capsule radius |
| `TORSO_SIZE` | `0.25 m` | Torso capsule height |
| `TORSO_COLLISION_RADIUS` | `0.12 m` | Torso capsule radius |
| `TABLE_X_MIN/MAX` | `−0.80 / 0.00 m` | Default table depth bounds |
| `TABLE_Y_MIN/MAX` | `−0.60 / 0.60 m` | Default table width bounds |
| `TABLE_Z_MIN/MAX` | `−1.18 / −0.40 m` | Default table height bounds |
| `SAFE_GOTO_STEPS` | `5` | Default sub-steps in `safeGoto` |
| `FAN_THRESHOLD` | `40.0 °C` | Temperature above which fans activate |

---

## Acknowledgements

**Author:** Benoit Baudry

This project was developed as part of a research internship at the [MSD-IRIMAS](https://msd-irimas.github.io/) team (Modélisation, Simulation et Décision — Informatique, Robotique, Image, Automatique, Signal), University of Haute-Alsace.

Special thanks to:

- **Maxime Devanne** — internship supervisor. Associate Professor at the University of Haute-Alsace, member of the MSD-IRIMAS team. [maxime-devanne.com](https://maxime-devanne.com/)
- **Nicolò Gavassa** — colleague and fellow contributor during the internship. [github.com/NickGava](https://github.com/NickGava)
- The **MSD-IRIMAS team** — [msd-irimas.github.io](https://msd-irimas.github.io/)
- **François Petitjean** — author of the DTW Barycenter Averaging algorithm used in `DBA_multivariate.py` (GPL-3.0)
- **Yoshman29** for the inspiration for the console manager [github](https://github.com/YoshiCrafter29)
- **Pollen Robotics** — creators of the Reachy 2021 robot and the `reachy-sdk`. [pollen-robotics.com](https://www.pollen-robotics.com/)
