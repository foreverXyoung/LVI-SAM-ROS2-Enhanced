# Localization status interface

This document defines the first, non-invasive localization-state interface for
the enhanced LVI-SAM package. It is intentionally an **observational adapter**:
it reports the state that the existing LiDAR localization path already computes
and does not replace the matcher, factor graph, TF publication, odometry, or
relocalization decisions.

## Scope and baseline

The baseline for this phase is commit `9cfeb11` (`fix OpenCV 4 visual loop color
conversion`). The implementation is staged in small increments so that mapping
and localization behavior can be compared against this commit at every step.

This phase adds only:

- the independent `lvi_sam_msgs` ROS 2 interface package;
- a small, side-effect-free state projection header with unit coverage;
- a structured status topic from `lvi_sam_mapOptimization`;
- a compatibility mapping from the existing `LocInitSta` enum and legacy string
  topic; and
- this contract and validation documentation.

It does **not** change point-cloud filtering, scan matching, graph optimization,
loop closure, TF, odometry values, or the existing force-relocalization service.

The projection header preserves the four legacy decision paths exactly. The
`VERIFYING`/`DEGRADED` values are now emitted as observational quality states
through that seam; they do not feed back into the legacy algorithm. Runtime
regression of all four original paths is still required before the reset/IMU/
VINS coupling work begins.

## Topic contract

| Item | Value |
|---|---|
| Package | `lvi_sam_msgs` |
| Message | `lvi_sam_msgs/msg/LocalizationStatus` |
| Topic | `/lio_sam/localization/status` |
| QoS | reliable, depth 1, transient local |
| Producer | `lvi_sam_mapOptimization` |
| Update | state transitions plus a low-rate heartbeat |

The existing `/lio_sam/localization/state` (`std_msgs/msg/String`) topic remains
available and unchanged for current consumers. New system integrations should
use the structured topic and treat `state_name` as display text only; branch on
the numeric `state` and `mode` constants.

## State mapping in phase 1

| Existing condition | `mode` | `state` | Legacy string | Meaning |
|---|---:|---:|---|---|
| `Loc.EnableFlag=false` | `MODE_MAPPING` | `MAPPING` | unchanged/ not forced | Mapping path is active; the new structured topic is authoritative and the legacy string publisher is not changed to add a mapping heartbeat. |
| `LocInitSta=NonInitialized` | `MODE_LOCALIZATION` | `RELOCALIZING` | `RELOCALIZING` | No valid prior-map initialization has been accepted yet. |
| `LocInitSta=Initializing` | `MODE_LOCALIZATION` | `RELOCALIZING` | `RELOCALIZING` | The existing RTK/Scan Context/configured-guess initialization path is running. |
| `LocInitSta=Initialized`, no successful scan-to-map frame yet | `MODE_LOCALIZATION` | `VERIFYING` | `LOCALIZED` | Initialization was accepted, but the adapter is waiting for the first successful scan-to-map frame. |
| `LocInitSta=Initialized`, current bad-match count > 0 | `MODE_LOCALIZATION` | `DEGRADED` | `LOCALIZED` | The existing algorithm remains initialized, but the latest scan-to-map quality is degraded. |
| `LocInitSta=Initialized`, current scan-to-map succeeds | `MODE_LOCALIZATION` | `TRACKING` | `LOCALIZED` | Existing initialization and the current scan-to-map frame are valid. |
| `LocInitSta=MayLost` | `MODE_LOCALIZATION` | `LOST` | `LOST` | Existing bad-match threshold was reached. The adapter latches this transient event for one status publication, while the algorithm still immediately returns to `NonInitialized`. |

`WAITING_FOR_SENSORS` and `ERROR` remain reserved for later phases. `MAPPING`
is an explicit structured state so an upper-layer state machine does not need to
infer mapping from a localization state. `VERIFYING` and `DEGRADED` are
observational quality states only; they do not change the underlying matcher or
relocalization decisions.

## Field semantics

- `pose_valid` is true when the current legacy state is `Initialized` in
  localization mode, including `VERIFYING` and `DEGRADED`. It is not a
  covariance or accuracy guarantee; upper layers should gate normal motion on
  `TRACKING` instead.
- `odometry_valid` is a conservative availability flag for the active LiDAR
  processing path; in localization mode it becomes true only after a
  scan-to-map frame has succeeded. It is not a replacement for
  `/lio_sam/mapping/odometry`.
- `sensors_ready` reports that the adapter has observed the required LiDAR,
  IMU, and incremental-odometry inputs. A true value only means messages have
  arrived, not that calibration or timing is correct.
- `map_ready` is true after a prior map has been loaded successfully in
  localization mode and false in mapping mode.
- `relocalization_active` is true for `RELOCALIZING` and `LOST`.
- `quality_degraded` is true only in `DEGRADED`.
- `match_score`, `confidence`, and the three `*_age` fields are `-1.0` while
  their computation is not part of the frozen behavior. Consumers must not use
  these fields for control until a later contract revision.
- `transition_sequence` increases only when the structured state changes.
  `previous_state` and `previous_state_name` identify the preceding structured
  state. The first message uses `STATE_UNKNOWN` as the previous state.
- `loss_count` increases when the existing `MayLost` state is observed.
- `LOST` is emitted for at least one heartbeat after the existing bad-match
  threshold, even though the frozen algorithm immediately continues in
  `NonInitialized`/`RELOCALIZING`.

`reason` is diagnostic text and is deliberately non-normative. Upper-layer
logic must not parse it.

## Consumer guidance

The safe first integration is to gate navigation on the numeric state:

- allow normal operation only in `MODE_LOCALIZATION + TRACKING`;
- stop or hold motion in `RELOCALIZING`, `VERIFYING`, `DEGRADED`, `LOST`, or
  `ERROR`;
- treat `MAPPING` as a separate commissioning mode.

Do not use the new status message to reset LVI-SAM, rewrite TF, or modify an
odometry topic in this phase. Those actions belong to the later controlled
relocalization work after the status contract is validated on real bags and the
robot.

## Validation

From a ROS 2 workspace containing this repository:

```bash
colcon list | grep -E '^(lvi_sam_msgs|lvi_sam)[[:space:]]'
colcon build --symlink-install --packages-up-to lvi_sam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_VISUAL=OFF
source install/setup.bash
ros2 interface show lvi_sam_msgs/msg/LocalizationStatus
ros2 topic echo /lio_sam/localization/status --once
```

The repository also installs a runtime contract checker. Run one case per
launch session:

```bash
# mapping launch
ros2 run lvi_sam verify_localization_status --case mapping

# localization launch; wait for the prior-map initialization to succeed
ros2 run lvi_sam verify_localization_status --case localization

# optional quality-state observations during a controlled test
ros2 run lvi_sam verify_localization_status --case verifying
ros2 run lvi_sam verify_localization_status --case degraded

# localization launch; call the existing service and verify the state output
ros2 run lvi_sam verify_localization_status --case force_relocalize

# while deliberately reproducing the configured bad-match/lost condition
ros2 run lvi_sam verify_localization_status --case lost --timeout 60
```

The `lost` case must be run while the transient loss is occurring; it does not
inject faults. Run the existing mapping, localization, lost, and
`/lio_sam/localization/force_relocalize` checks before enabling any later state
machine behavior. The expected legacy topics and odometry must remain unchanged.
