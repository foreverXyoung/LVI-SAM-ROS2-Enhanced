# Localization reset-event interface

This document defines the second, event-only seam for controlled reset
propagation. It is deliberately separate from the existing LiDAR matcher,
factor graph, TF, and odometry decisions. Adding the topic does not itself
force a relocalization; each producer explicitly declares which downstream
component should clear state.

## Message and topic

| Item | Value |
|---|---|
| Message | `lvi_sam_msgs/msg/LocalizationReset` |
| Topic | `/lio_sam/localization/reset` |
| QoS | reliable, depth 10, volatile |
| Map/LiDAR owner of `reset_id` | `lvi_sam_mapOptimization` |

The launch file exposes the topic as a single `localization_reset_topic`
argument. The default must remain `/lio_sam/localization/reset`; a custom
absolute topic may be used when multiple LVI-SAM instances share a ROS graph.

## Identifier semantics

- `reset_id` is the LiDAR/map continuity generation. It is the same generation
  that is carried in the existing internal covariance metadata consumed by
  the IMU preintegration and visual initializer.
- `event_id` identifies one event from one source. It is not globally unique
  across processes, so receivers deduplicate by `(source, event_id)` when they
  need deduplication.
- `LocalizationStatus.reset_id` reports the most recently owned LiDAR/map
  generation. It does not mean that every downstream node has already
  completed its reset.

## Reasons and expected actions

| Reason | Typical source | `reset_imu` | `restart_visual` | Meaning |
|---|---|---:|---:|---|
| `RELOCALIZATION` | map optimization | true | true | A new prior-map pose was accepted. |
| `FORCE_RELOCALIZATION` | map optimization | true | true | Existing force service requested a fresh initialization. |
| `MAP_CORRECTION` | map optimization | true | true | A loop/graph correction changed the map pose basis. |
| `VINS_FAILURE` | visual estimator | false | true | VINS cleared its own state; LiDAR/IMU map state is not replaced. |
| `IMU_FAILURE` | IMU preintegration | true | true | IMU propagation rejected its optimized state. |
| `IMAGE_STREAM_RESET` | feature tracker | false | true | Camera timestamps or image stream continuity failed. |
| `MANUAL` | integration/test tool | explicit | explicit | Reserved for a controlled external test. |

The flags are requests for the named downstream action, not a guarantee that
the receiver has completed it. A receiver must log the event and apply only
the actions whose flags are true. In particular, a `VINS_FAILURE` event must
not reset the LiDAR map or increment the LiDAR-owned `reset_id`.

## Compatibility rules

1. The old covariance reset metadata remains published unchanged. Existing
   ROS 2 consumers that do not know this message continue to work.
2. The reset topic is advisory during the staged rollout. No old algorithm
   branch may depend on a reset subscriber being present.
3. A reset event is ordered only relative to messages from the same publisher;
   consumers must use the event timestamp and generation checks to reject
   stale data.
4. A receiver must clear queued data from the previous generation before
   accepting new data. This is the important distinction from merely resetting
   an estimator flag.

## Staged rollout

The interface is introduced before any subscriber is enabled. The intended
order is:

1. publish map-owned events while preserving covariance reset metadata;
2. make IMU propagation clear its queues and integration state on events;
3. make visual feature/estimator nodes restart on the explicit visual flag;
4. emit VINS/IMU failure events and verify de-duplication on bags and on the
   robot.

Until those runtime checks pass, the status topic remains the upper-layer
control interface and this reset topic must not be used by navigation as a
direct motion command.
