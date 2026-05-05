# Belief Odometry Factor Mode

Date: 2026-05-04

## Goal

Move CBS receiver injection from absolute pose priors toward relative belief
odometry constraints. The motivation is that a unary received pose prior can
over-anchor the receiver trajectory and forces iSAM2 to reconcile absolute
poses across the whole connected local chain. A relative constraint instead
says: the receiver's motion between two matched local poses should agree with
the sender's believed motion over the same interval.

This is additive. The previous unary prior path is still available.

## New mode

The shared launch argument is:

```bash
cbs_belief_factor_mode:=prior
cbs_belief_factor_mode:=odom_between
```

`prior` preserves the existing behavior.

`odom_between` converts accepted incoming pose beliefs from the same sender into
`gtsam::BetweenFactor<Pose3>` constraints. The first accepted pose belief for a
sender initializes receiver-side odometry bookkeeping. Later accepted beliefs
look up the previous accepted receiver key from the same pose stream and queue a
relative factor:

```text
measurement = T_sender_previous^-1 * T_sender_current
factor      = BetweenFactor(receiver_previous_key, receiver_current_key,
                            measurement, odom_covariance)
```

The receiver-side GBP gates still run before the odometry factor is queued.

## Current covariance approximation

The current implementation does not yet send sender-side pairwise joint
marginals. It builds a conservative first-pass odometry covariance from the two
available pose marginals:

```text
odom_covariance = Sigma_previous + Sigma_current
```

Then the existing per-source covariance scale is applied. This is deliberately
simple so we can test the factor semantics without changing the ROS belief
message. The better follow-up is to export sender-side pairwise conditional or
joint-marginal information and use that directly for the relative factor noise.

## Sender-computed odometry update

Date: 2026-05-05

The `odom_between` mode now has a sender-computed transport path. The sender
exports relative motion beliefs directly instead of sending only absolute pose
beliefs and asking the receiver to reconstruct the edge.

New ROS messages:

- `liorf/pose_odom_belief.msg`
- `liorf/pose_odom_belief_array.msg`

The outgoing sender path is:

```text
active local pose window
  -> BPSAM::getOdometryBeliefs(keys, peer_agent)
  -> consecutive same-stream pose pairs
  -> measured_from_to = T_from^-1 * T_to
  -> joint marginal information over [from,to]
  -> Lambda_jj, Lambda_ji
  -> Q_to_given_from = Lambda_jj^-1
  -> A_to_from = -Lambda_jj^-1 * Lambda_ji
  -> pose_odom_belief_array
```

The receiver path is:

```text
pose_odom_belief_array
  -> timestamp-match from/to stamps to receiver local keyframes
  -> convert relative motion/covariance through known camera-lidar extrinsics
  -> BPSAM::addOdometryBeliefsDetailed(...)
  -> sender-conditional residual or BetweenFactor fallback
```

The old receiver-reconstructed path remains in `BPSAM::addBeliefsDetailed` as a
fallback when absolute pose beliefs are received in `odom_between` mode. The
normal launch now wires dedicated odometry belief topics:

```text
/kimera/cbs/odom_belief_out -> /kimera/cbs/odom_belief_in
```

where LiORF subscribes to Kimera's odometry-belief output and publishes to
Kimera's odometry-belief input, matching the existing absolute belief topic
direction.

The strict path is enabled by:

```bash
cbs_use_sender_conditional_odom_factors:=true
```

When `odom_between`, temporary-linear CBS, and sender-computed odometry are all
enabled, the receiver injects a fixed linearized sender conditional residual:

```text
r = delta_to - A_to_from * delta_from
Q = Lambda_jj^-1
```

The implementation uses `SenderConditionalPose3Factor`, with anchors frozen at
the receiver's current `from` estimate and the incoming relative-motion
prediction for `to`. This approximates Mike's requested posterior conditional
edge without adding it permanently to the nonlinear graph. If
`cbs_use_sender_conditional_odom_factors:=false`, the same sender-computed
message falls back to:

```text
BetweenFactor<Pose3>(receiver_from, receiver_to, measured_from_to, Q)
```

## Temporary-linear interaction

`odom_between` works with the temporary-linear iSAM2 path. In that case the
relative factor is linearized for the current delta solve and is not inserted
into the persistent nonlinear graph.

The already-applied temporary-linear gate still operates per sender/key. When an
odometry factor is applied, BPSAM records the endpoint beliefs as applied so
near-identical repeated window messages can be skipped.

## Diagnostics

New log rows:

- `CBS_BELIEF_ODOM_ROW`: queued or waiting odometry edges, with endpoint
  covariance traces and odometry covariance trace.
- `CBS_ODOM_PREINJECTION_RESIDUAL_ROW`: receiver relative-motion residual just
  before factor injection.

The experiment report parser writes:

- `parsed/cbs_belief_odom.csv`
- `parsed/belief_odom_summary.csv`

`summary.md` now includes a "Belief Odometry Factors" section when these rows
exist.

## Strict sender-conditional validation

Date: 2026-05-05

The full sender-conditional path was implemented and the S3E alpha 60 s
experiment was rerun:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL \
  --name s3e-alpha-60s-sender-conditional-odom \
  --duration 60 \
  --timeout-padding 90 \
  --no-use-kimera-rviz \
  --no-use-liorf-rviz \
  --no-kimera-visualize \
  --no-rerun-visualizer-enable \
  --extra-arg cbs_belief_factor_mode:=odom_between
```

Run directory:

```text
/home/yeranis/repos/V4RL/runs/20260505-162204_s3e-alpha-60s-sender-conditional-odom
```

The topic wiring bug was fixed first. The run now receives and injects
sender-computed odometry beliefs in both directions:

- `CBS_BELIEF_ODOM_ROW`: `L2K sender_odom_queued=2098`,
  `K2L sender_odom_queued=1581`.
- `CBS_ODOM_PREINJECTION_RESIDUAL_ROW`: `L2K temporary_linear_odom_applied=2046`,
  `K2L temporary_linear_odom_applied=1008`.

The strict conditional path is active, but this run is not numerically healthy:

- Kimera APE RMSE: `3.1020 m`.
- LiORF APE RMSE: `24.5274 m`.
- LiORF RPE RMSE: `4.9218 m`.
- K2L odometry residual p95 before injection: `0.2997`.
- L2K odometry residual p95 before injection: `0.4215`.
- K2L odometry covariance trace mean: `7.002e-04`.
- L2K odometry covariance trace mean: `3.030e-04`.

Interpretation: the full conditional implementation is wired and exercised, but
the strict `A,Q` factors are very strong and currently destabilize LiORF.

The same sender-computed odometry transport was then rerun with:

```bash
--extra-arg cbs_use_sender_conditional_odom_factors:=false
```

Run directory:

```text
/home/yeranis/repos/V4RL/runs/20260505-164424_s3e-alpha-60s-sender-odom-between-no-conditional
```

That comparison kept LiORF healthy but degraded Kimera:

- Kimera APE RMSE: `3.2997 m`.
- LiORF APE RMSE: `0.7545 m`.
- K2L temporary-linear odometry applied: `1074`.
- L2K temporary-linear odometry applied: `1231`.
- K2L odometry residual p95 before injection: `0.4582`.
- L2K odometry residual p95 before injection: `0.4878`.

Current conclusion: the sender-computed odometry transport and topic wiring are
working. The instability is now localized to factor semantics/tuning: strict
posterior conditional factors are beneficial for Kimera but unsafe for LiORF,
while plain `BetweenFactor` odometry keeps LiORF stable but does not help
Kimera. The next engineering target is not basic transport; it is the
coordinate/frame semantics and covariance scale of the received odometry
constraint in each direction.

## Validation

Built the workspace packages touched by this change:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash && cd /workspace && \
   catkin build cbs liorf kimera_vio kimera_vio_ros cbsms'
```

Added and ran a focused BPSAM smoke executable:

```bash
docker exec cbsms_ws bash -lc \
  'source /workspace/devel/setup.bash && \
   /workspace/build/cbs/examples/bpsam_belief_odom_smoke'
```

The smoke test creates a two-pose local BPSAM graph, receives two sequential
beliefs in `odom_between` mode, and verifies:

- the first belief initializes the odometry stream and logs
  `waiting_for_previous`;
- the second belief queues a `BetweenFactor<Pose3>` and logs `queued`;
- the following BPSAM update tracks one CBS odometry factor and logs
  `CBS_ODOM_PREINJECTION_RESIDUAL_ROW`.

Kimera's CBS ROS pub/sub bridge was then moved out of the `RosVisualizer`
dependency path by adding a headless bridge in `KimeraVioRos`. This lets
Kimera publish and receive CBS beliefs when `--no-kimera-visualize` is used,
avoiding the VTK/OpenGL display dependency in the container.

The headless `/kimera_vio_ros/odometry` publisher was also added to
`KimeraVioRos`, using the same backend-output pose, velocity, and covariance
mapping as `RosVisualizer::publishState`. Without this, CBS could run headless
but the report could not compute Kimera trajectory metrics because the
experiment recorder subscribes to `/kimera_vio_ros/odometry`.

The full headless odometry-factor run command was:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL \
  --name s3e-alpha-60s-odom-between-full-summary-v2 \
  --duration 60 \
  --timeout-padding 90 \
  --no-use-kimera-rviz \
  --no-use-liorf-rviz \
  --no-kimera-visualize \
  --no-rerun-visualizer-enable \
  --extra-arg cbs_belief_factor_mode:=odom_between
```

Run directory:

```text
/home/yeranis/repos/V4RL/runs/20260504-222820_s3e-alpha-60s-odom-between-full-summary-v2
```

Key validation results:

- roslaunch return code was `0`.
- CBS transport rows were present in both directions:
  `K2L=11770`, `L2K=4059`.
- Belief odometry rows were present in both directions:
  `K2L queued=6622`, `L2K queued=3953`.
- Temporary-linear odometry updates were applied in both directions:
  `K2L=4049`, `L2K=3950`.
- Pre-injection relative-motion residuals were much smaller than the previous
  bad unary prior residuals:
  `K2L p95=0.3880`, `L2K p95=0.3595`.
- Kimera trajectory CSV was populated with 289 rows and LiORF with 225 rows.
- Evo metrics were present for both estimators:
  `Kimera APE RMSE=0.4209 m`, `Kimera RPE RMSE=0.4772 m`,
  `LiORF APE RMSE=0.8573 m`, `LiORF RPE RMSE=0.6432 m`.

## Changed files

- `src/cbs/include/cbs/bpsam/bpsam.h`
- `src/cbs/src/bpsam/bpsam.cpp`
- `src/cbs/examples/CMakeLists.txt`
- `src/cbs/examples/bpsam_belief_odom_smoke.cpp`
- `src/liorf/src/mapOptmization.cpp`
- `src/liorf/launch/run_lio_sam_s3e_alpha.launch`
- `src/Kimera-VIO/include/kimera-vio/pipeline/Pipeline.h`
- `src/Kimera-VIO/src/backend/VioBackend.cpp`
- `src/Kimera-VIO-ROS/include/kimera_vio_ros/KimeraVioRos.h`
- `src/Kimera-VIO-ROS/src/KimeraVioRos.cpp`
- `src/Kimera-VIO-ROS/launch/kimera_vio_ros.launch`
- `src/Kimera-VIO-ROS/launch/kimera_vio_ros_s3e_alpha_mono.launch`
- `src/Kimera-VIO-ROS/launch/s3e_alpha_liorf_kimera_experiment.launch`
- `src/cbsms/launch/s3e_alpha_liorf_kimera_topics.launch`
- `src/cbsms/tools/cbsms_experiment.py`

## Known limitations

- The old receiver-reconstructed path still exists for absolute pose beliefs,
  but the main experiment path now uses sender-computed odometry belief
  messages.
- The strict sender-conditional factor now uses `conditional_A`, but the frame
  semantics need validation against a plain sender-computed `BetweenFactor`
  run because the first strict 60 s run diverged.
- The previous accepted receiver key is chosen from the same receiver pose
  stream by index order, which avoids accidental backwards edges when repeated
  sliding-window messages arrive.
