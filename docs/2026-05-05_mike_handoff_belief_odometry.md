# CBSMS Belief Odometry Handoff

Date: 2026-05-05

This handoff explains the Zak branch set for the CBSMS belief-odometry work.
The goal is to let another agent import Zak's branches and inspect/update the
code without relying on chat history.

## Branch Set

Clone or update the cbsms meta package from:

```text
git@github.com:Zakyera/cbsms.git
branch: zak/belief-odom-conditional-20260505
```

Then import the changed CBSMS component repositories with:

```bash
vcs import /home/yeranis/repos/V4RL < src/cbsms/zak_belief_odom_20260505.repos
```

The manifest points to:

| path | repo | branch |
| --- | --- | --- |
| `src/cbs` | `git@github.com:Zakyera/cbs.git` | `zak/belief-odom-conditional-20260505` |
| `src/liorf` | `git@github.com:Zakyera/liorf.git` | `zak/belief-odom-conditional-20260505` |
| `src/Kimera-VIO` | `git@github.com:Zakyera/Kimera-VIO.git` | `zak/belief-odom-conditional-20260505` |
| `src/Kimera-VIO-ROS` | `git@github.com:Zakyera/Kimera-VIO-ROS.git` | `zak/belief-odom-conditional-20260505` |

## What Changed

The old receiver-side CBS path converted accepted external pose beliefs into
absolute `PriorFactor<Pose3>` constraints. This branch set adds a belief
odometry mode:

```bash
cbs_belief_factor_mode:=odom_between
```

In this mode the sender computes consecutive belief odometry edges from its own
local graph:

```text
Z_ij = inverse(T_i) * T_j
Q = Lambda_jj^-1
A = -Lambda_jj^-1 * Lambda_ji
```

where `Lambda` is the sender-side joint marginal information for the pair
`(i,j)`. The receiver timestamp-matches both endpoints to its own local keys and
injects an external odometry constraint between the matched receiver poses.

## Receiver Factor Modes

The new switch:

```bash
cbs_use_sender_conditional_odom_factors:=true
```

selects the strict sender-conditional factor in temporary-linear mode. The
factor residual is:

```text
r = delta_to - A * delta_from
noise = Q
```

When the switch is false, the same sender-computed odometry message falls back
to a plain relative factor:

```text
BetweenFactor<Pose3>(receiver_from, receiver_to, Z_ij, Q)
```

Both paths keep the temporary-linear CBS behavior: the external factor affects
the current iSAM2 solve and is removed before fixed-lag marginalization.

## Major Files

| repo | important files |
| --- | --- |
| `cbs` | `include/cbs/bpsam/bpsam.h`, `src/bpsam/bpsam.cpp`, `examples/bpsam_belief_odom_smoke.cpp` |
| `liorf` | `msg/pose_odom_belief*.msg`, `src/mapOptmization.cpp`, `launch/run_lio_sam_s3e_alpha.launch` |
| `Kimera-VIO` | `src/backend/VioBackend.cpp`, backend/module output plumbing |
| `Kimera-VIO-ROS` | `src/KimeraVioRos.cpp`, CBS bridge and launch topic wiring |
| `cbsms` | `tools/cbsms_experiment.py`, `launch/s3e_alpha_liorf_kimera_topics.launch`, this document |

## Validation Commands

Build:

```bash
docker exec cbsms_ws bash -lc \
  "cd /workspace && source /opt/ros/noetic/setup.bash && \
   catkin build cbs liorf kimera_vio kimera_vio_ros --no-status --summarize -j8 -p1"
```

Strict sender-conditional run:

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

## Current Technical Status

The transport and injection are working in both directions. The branch set
therefore moves the problem from "are beliefs arriving?" to "which factor
semantics and covariance scaling are mathematically correct for each direction?"

Observed 60 s comparison:

| mode | Kimera APE RMSE | LiORF APE RMSE | interpretation |
| --- | --- | --- | --- |
| strict `A,Q` conditional | `0.6259 m` | `20.8067 m` | helps Kimera but destabilizes LiORF |
| plain sender-computed `BetweenFactor` | `3.2997 m` | `0.7545 m` | keeps LiORF healthy but hurts Kimera |

The next review target should be the tangent-space/frame semantics of `A`, the
receiver-side anchoring of the strict conditional residual, and the covariance
scale of `Q` in each direction.
