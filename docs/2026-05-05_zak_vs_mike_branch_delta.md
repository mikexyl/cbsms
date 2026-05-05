# Zak Branch Delta vs Mike Branches

Date: 2026-05-05

This document is for Mike's Codex. It summarizes what Zak's branch set adds on
top of Mike's current CBSMS branches, so the changes can be reviewed or ported
without relying on chat history.

## Branch Set

Zak branch used in all modified repositories:

```text
zak/belief-odom-conditional-20260505
```

Manifest in the cbsms meta package:

```text
src/cbsms/zak_belief_odom_20260505.repos
```

Primary handoff document:

```text
src/cbsms/docs/2026-05-05_mike_handoff_belief_odometry.md
```

## Baseline Comparison

These are the branch comparisons used to produce this document.

| repo | Mike base ref | Mike base commit | Zak head ref | Zak head commit before this doc |
| --- | --- | --- | --- | --- |
| `cbsms` | `mikexyl/cbsms:main` | `b20e41e` | `Zakyera/cbsms:zak/belief-odom-conditional-20260505` | `d2ae58e` |
| `cbs` | `mikexyl/cbs:prior-belief-factors` | `cdcfd5b` | `Zakyera/cbs:zak/belief-odom-conditional-20260505` | `9803a67` |
| `liorf` | `mikexyl/liorf:dev/cbsms` | `ab0a6a9` | `Zakyera/liorf:zak/belief-odom-conditional-20260505` | `28d0ccc` |
| `Kimera-VIO` | `mikexyl/Kimera-VIO:dev/cbsms` | `1310e4b` | `Zakyera/Kimera-VIO:zak/belief-odom-conditional-20260505` | `77ccc72` |
| `Kimera-VIO-ROS` | `mikexyl/Kimera-VIO-ROS:dev/cbsms` | `7dd6151` | `Zakyera/Kimera-VIO-ROS:zak/belief-odom-conditional-20260505` | `d829f1f` |

## High-Level Change

Mike's branches already contain the CBSMS base integration. Zak's branches add
the experimental receiver/sender machinery needed to study CBS belief
odometry:

1. Receiver-side diagnostics for why beliefs are accepted or rejected.
2. Temporary-linear CBS injection so external factors affect the current solve
   but are removed before fixed-lag marginalization.
3. Sender-computed belief odometry messages carrying relative pose,
   conditional covariance `Q`, and conditional matrix `A`.
4. Receiver-side odometry factor injection, either as a plain
   `BetweenFactor<Pose3>` fallback or as a strict sender-conditional residual.
5. Headless Kimera CBS bridge and experiment-report tooling so 60 s runs can be
   reproduced and summarized.

## Repo: `cbs`

Comparison:

```bash
git log --oneline upstream/prior-belief-factors..zak/belief-odom-conditional-20260505
git diff --stat upstream/prior-belief-factors..zak/belief-odom-conditional-20260505
```

Ahead commits:

```text
9803a67 Add sender conditional CBS odometry factors
b71b4bc Add temporary-linear CBS belief accounting
f9652a1 Add CBS belief gate diagnostics
```

Changed files:

```text
M examples/CMakeLists.txt
A examples/bpsam_belief_odom_smoke.cpp
M include/cbs/bpsam/bpsam.h
M include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h
M src/bpsam/bpsam.cpp
```

What changed:

- Adds `CbsBeliefFactorMode` with `prior` and `odom_between`.
- Adds `CbsOdometryBelief` transport structures and
  `BPSAM::addOdometryBeliefsDetailed(...)`.
- Adds `BPSAM::getOdometryBeliefs(...)`, which computes sender-side consecutive
  pose-pair odometry beliefs from the current local belief window.
- Computes sender pair conditional information from joint marginal information:

```text
Q = Lambda_jj^-1
A = -Lambda_jj^-1 * Lambda_ji
```

- Adds `SenderConditionalPose3Factor` with residual:

```text
r = delta_to - A * delta_from
```

- Keeps a fallback path to a normal `BetweenFactor<Pose3>` using the sender
  relative motion and `Q`.
- Adds temporary-linear accounting so applied CBS factors are recorded and
  near-identical repeated window messages can be skipped.
- Adds diagnostics rows:
  `CBS_RECEIVER_DIAGNOSTIC_ROW`, `CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW`,
  `CBS_PREINJECTION_RESIDUAL_ROW`, `CBS_BELIEF_ODOM_ROW`,
  `CBS_ODOM_PREINJECTION_RESIDUAL_ROW`.
- Adds a focused smoke executable for belief odometry.

Important math note:

- Odometry covariance is extracted from `jointMarginalInformation([from,to])`.
- `setMarginalizationGraph(LOCAL)` builds `tmp_marginals_` with CBS/external
  belief factors removed before `getOdometryBeliefs(...)` is called.
- Therefore the intended covariance source is local-only, excluding temporary
  external CBS factors.

## Repo: `liorf`

Comparison:

```bash
git log --oneline upstream/dev/cbsms..zak/belief-odom-conditional-20260505
git diff --stat upstream/dev/cbsms..zak/belief-odom-conditional-20260505
```

Ahead commits:

```text
28d0ccc Wire CBS odometry belief messages
8f176da Wire CBS temporary-linear controls
f880e12 Add LiORF CBS covariance controls
```

Changed files:

```text
M CMakeLists.txt
M config/s3e_alpha.yaml
M launch/run_lio_sam_s3e_alpha.launch
A msg/pose_odom_belief.msg
A msg/pose_odom_belief_array.msg
M src/mapOptmization.cpp
```

What changed:

- Adds ROS messages:
  `liorf/pose_odom_belief.msg` and
  `liorf/pose_odom_belief_array.msg`.
- Publishes outgoing LiORF sender-computed odometry beliefs when
  `cbs_belief_factor_mode:=odom_between`.
- Subscribes to incoming Kimera odometry beliefs and timestamp-matches both
  endpoints to LiORF local keyframes.
- Wires LiORF launch parameters for:
  `cbs_belief_factor_mode`, `cbs_use_temporary_cbs_linear_priors`,
  `cbs_use_sender_conditional_odom_factors`, soft reset, raw previous gate,
  covariance scaling, and odometry belief topics.
- Calls `bpsam->setMarginalizationGraph(LOCAL)` before exporting beliefs so
  outgoing covariance comes from the cleaned local marginal graph.

## Repo: `Kimera-VIO`

Comparison:

```bash
git log --oneline upstream/dev/cbsms..zak/belief-odom-conditional-20260505
git diff --stat upstream/dev/cbsms..zak/belief-odom-conditional-20260505
```

Ahead commits:

```text
77ccc725 Forward CBS odometry beliefs through backend
5097a1b7 Add CBS receiver injection controls
dde904a1 Add Kimera CBS belief diagnostics
```

Changed files:

```text
M include/kimera-vio/backend/VioBackend-definitions.h
M include/kimera-vio/backend/VioBackend.h
M include/kimera-vio/backend/VioBackendModule.h
M include/kimera-vio/pipeline/Pipeline.h
M src/backend/VioBackend.cpp
M src/backend/VioBackendModule.cpp
```

What changed:

- Adds backend input/output plumbing for CBS pose beliefs and odometry beliefs.
- Adds pending incoming odometry belief queue and timestamp matching to Kimera
  keyframes.
- Exports outgoing Kimera sender-computed odometry beliefs through
  `BPSAM::getOdometryBeliefs(...)`.
- Wires BPSAM controls:
  `cbs_belief_factor_mode`,
  `cbs_use_temporary_cbs_linear_priors`,
  `cbs_use_sender_conditional_odom_factors`,
  already-applied gate thresholds, raw previous gate, and soft reset.
- Adds diagnostic/provenance rows for Kimera CBS belief generation and
  covariance sanity.

## Repo: `Kimera-VIO-ROS`

Comparison:

```bash
git log --oneline upstream/dev/cbsms..zak/belief-odom-conditional-20260505
git diff --stat upstream/dev/cbsms..zak/belief-odom-conditional-20260505
```

Ahead commits:

```text
d829f1f Bridge CBS odometry beliefs headlessly
dd3dbd8 Expose CBS receiver launch controls
5c6edfe Add CBS Rerun alignment controls
```

Changed files:

```text
M include/kimera_vio_ros/KimeraVioRos.h
M include/kimera_vio_ros/RosVisualizer.h
M launch/kimera_vio_ros.launch
M launch/kimera_vio_ros_s3e_alpha_mono.launch
M launch/s3e_alpha_liorf_kimera_experiment.launch
M src/KimeraVioRos.cpp
M src/RosVisualizer.cpp
```

What changed:

- Moves CBS bridge functionality into `KimeraVioRos` so it works even when
  visualization is disabled.
- Adds odometry-belief ROS pub/sub for Kimera:
  `cbs_odom_belief_in_topic` and `cbs_odom_belief_out_topic`.
- Converts odometry beliefs through the known camera/lidar/body extrinsic
  convention before forwarding to/from Kimera backend.
- Adds headless `/kimera_vio_ros/odometry` output so experiment reports can
  compute Kimera trajectory metrics without VTK/OpenGL visualization.
- Wires all experiment launch args for the new CBS modes and topic directions.
- Keeps Rerun world-alignment controls from the earlier visualization work.

## Repo: `cbsms`

Comparison:

```bash
git log --oneline origin/main..zak/belief-odom-conditional-20260505
git diff --stat origin/main..zak/belief-odom-conditional-20260505
```

Ahead commits before this comparison document:

```text
d2ae58e Document CBSMS belief odometry handoff
e338f98 Add repeatable CBSMS experiment reporting
```

Changed files before this comparison document:

```text
M README.md
A docs/2026-05-03_sender_side_filter_debug.md
A docs/2026-05-04_belief_odometry_factors.md
A docs/2026-05-05_mike_handoff_belief_odometry.md
A docs/experiment_reports.md
A docs/project_understanding.md
A launch/s3e_alpha_liorf_kimera_topics.launch
A tools/cbsms_experiment.py
A tools/run_s3e_report.sh
A zak_belief_odom_20260505.repos
A zak_cbsms.rbl
```

What changed:

- Adds reproducible `cbsms_experiment.py run/report` tooling for S3E alpha CBS
  experiments.
- Adds markdown reports and project-understanding notes for the CBSMS setup.
- Adds parsing/reporting for odometry belief rows and pre-injection residuals.
- Adds `zak_belief_odom_20260505.repos`, pointing to Zak's branch set across
  the modified repositories.
- Adds the handoff document Mike should read first:
  `docs/2026-05-05_mike_handoff_belief_odometry.md`.

## Current Experimental Commands

Strict sender-conditional mode:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL \
  --name s3e-alpha-60s-sender-conditional-odom-explicit \
  --duration 60 \
  --timeout-padding 90 \
  --no-use-kimera-rviz \
  --no-use-liorf-rviz \
  --no-kimera-visualize \
  --no-rerun-visualizer-enable \
  --extra-arg cbs_belief_factor_mode:=odom_between \
  --extra-arg cbs_use_sender_conditional_odom_factors:=true
```

Plain sender-computed `BetweenFactor` comparison:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL \
  --name s3e-alpha-60s-sender-odom-between-no-conditional \
  --duration 60 \
  --timeout-padding 90 \
  --no-use-kimera-rviz \
  --no-use-liorf-rviz \
  --no-kimera-visualize \
  --no-rerun-visualizer-enable \
  --extra-arg cbs_belief_factor_mode:=odom_between \
  --extra-arg cbs_use_sender_conditional_odom_factors:=false
```

## Observed 60 s Behavior

The branch set is not presented as a tuned final result. It is a working
implementation and diagnostic scaffold for the belief-odometry idea.

| mode | Kimera APE RMSE | LiORF APE RMSE | meaning |
| --- | --- | --- | --- |
| strict `A,Q` conditional | `0.6259 m` | `20.8067 m` | helps Kimera but destabilizes LiORF |
| plain sender-computed `BetweenFactor` | `3.2997 m` | `0.7545 m` | keeps LiORF healthy but hurts Kimera |

The next technical review should focus on:

- tangent-space/frame semantics of the sender conditional matrix `A`;
- how `Q` should be scaled per direction;
- whether the strict conditional should be reanchored differently on the
  receiver side;
- whether LiORF and Kimera should use different odometry-factor modes or
  covariance scales.

