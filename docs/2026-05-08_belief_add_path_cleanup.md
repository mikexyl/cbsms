# Belief Add Path Cleanup

Date: 2026-05-08

## Context

The CBS odometry bridge was still rejecting or retrying some locally matched
beliefs in the receiver layers before they reached the CBS add interface. This
made the runtime counters hard to reason about because outside-window decisions
could be split between LiORF/Kimera receiver code and BPSAM/IFLS.

The intended ownership is:

- Timestamp matching is receiver-side transport logic because it maps sender
  stamps to receiver keys.
- Once receiver keys are available, the belief should be passed to
  `addOdometryBeliefsDetailed`.
- Active-window rejection should be reported by CBS/IFLS add detail rows.

## Changes

### LiORF

Commit: `64d322dcb929c851c0110c12cc239b2d21181cad`

- Switched the mapping backend from raw `cbs::BPSAM` to
  `cbs::IncrementalFixedLagBpsamSmoother`.
- Uses local pose index as the IFLS timestamp, preserving the existing
  `cbsBeliefExchangeWindowSize` count-based active window.
- Removed LiORF's receiver-side `receiver_window_or_order` rejection/retry gate.
- Matched K2L odometry beliefs now call `addOdometryBeliefsDetailed`; outside
  active-window decisions are emitted as
  `target_edge_outside_IFLS_active_local_window`.

### Kimera-VIO

Commit: `3b05fac2ded0cd4f9eceb69d5f0dacee5394d2b4`

- Removed Kimera's pre-BPSAM receiver-order and missing-receiver-state gates.
- Timestamp resolution now searches all known current/past keyframe timestamps,
  not only the active frame span.
- Resolved L2K odometry beliefs are handed to the IFLS-backed
  `addOdometryBeliefsDetailed`, which owns inactive-window rejection.

## Validation

Build:

```bash
catkin build liorf --no-status --summarize -j12 -p1
```

Runtime:

```bash
roslaunch kimera_vio_ros s3e_alpha_liorf_kimera_experiment.launch \
  bag_duration:=60.0 \
  enable_cbs_bridge:=true \
  rerun_visualizer_enable:=true \
  kimera_visualize:=true \
  use_kimera_rviz:=true \
  use_liorf_rviz:=false \
  shutdown_on_bag_finish:=true
```

Validation log: `logs/s3e_liorf_ifls_cleanup_validation.log`

Observed:

- 60 second S3E run completed with clean shutdown.
- Old receiver-side decision labels were absent:
  `retry_receiver_window_or_order`, `dropped_receiver_window_or_order`,
  `retry_receiver_order`, `dropped_receiver_order`, `retry_missing_state`,
  `dropped_missing_state`, `missing_receiver_state`.
- Outside-window decisions appeared in CBS add rows:
  - K2L `CBS_BPSAM_ODOM_ADD_ROW`: 5357 total.
  - K2L `target_edge_outside_IFLS_active_local_window`: 1799.
  - L2K `target_edge_outside_IFLS_active_local_window`: 13.

## Remaining Follow-Up

Repeated odometry edges still reach BPSAM and are reported as
`temporary_linear_odom_already_applied` or
`temporary_linear_odom_already_pending`. That is separate from receiver-side
rejection cleanup. The next behavior question is whether a repeated edge should
update/replace the temporary-linear record when the incoming belief changed
enough, instead of always being skipped by the already-applied gate.
