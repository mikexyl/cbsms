# CBSMS Project Understanding

Last updated: 2026-05-04.

This note captures the working understanding of the current CBSMS workspace,
the project goal, and the implementation details verified in the local code and
recent Git history. Treat it as an engineering memory for Zak and collaborators;
if Mike changes the design, update this file with the commit and date.

## Project Goal

CBSMS is being used for a same-drone multi-sensor experiment:

- Kimera-VIO estimates the drone pose from camera and IMU.
- LiORF estimates the drone pose from lidar and IMU.
- Both estimators run on the same physical platform.
- The camera, lidar, and IMU extrinsic transformations are known.
- The estimators exchange pose beliefs through CBS/BPSAM.

This is different from the original multi-robot CBS paper setup. In the paper,
agents can need an anchor/between-agent stage to establish relative frames. In
this project, LiORF and Kimera are not separate robots with unknown relative
frames. They are two estimator agents on the same drone, so the CBS exchange is
pose-only. We do not need the paper's second-stage anchor-belief exchange for
unknown inter-agent transforms.

## Repositories And Branches

The workspace anchor is the `cbsms` metapackage in:

```text
/home/yeranis/repos/V4RL/src/cbsms
```

The core branches currently used by the workspace are:

| repo | branch | purpose |
| --- | --- | --- |
| `cbsms` | `main` | workspace metapackage, Docker, reports, docs |
| `cbs` | `prior-belief-factors` | BPSAM/CBS belief exchange backend |
| `liorf` | `dev/cbsms` | LiORF lidar estimator with CBS bridge |
| `Kimera-VIO` | `dev/cbsms` | Kimera backend with CBS/BPSAM integration |
| `Kimera-VIO-ROS` | `dev/cbsms` | ROS/Rerun bridge and S3E launch wiring |

Mike's remotes were verified with `git ls-remote` on 2026-05-02, and the local
heads matched his GitHub branch heads at that time.

## Estimator Backends

### Kimera

Kimera uses fixed-lag smoothing. In the S3E Alpha Mono XFeat configuration,
`nr_states` is 50, so the active backend horizon is approximately 50 pose
states. Poses outside that horizon are marginalized by the fixed-lag smoother.

For CBS exchange, Kimera requests outgoing beliefs for the active fixed-lag
pose keys, not only the newest pose.

Relevant files:

- `Kimera-VIO/params/S3EAlphaMonoXfeat/BackendParams.yaml`
- `Kimera-VIO/src/backend/VioBackend.cpp`
- `Kimera-VIO-ROS/src/RosVisualizer.cpp`

### LiORF

LiORF has two different window concepts:

1. A local scan-to-map submap window used for lidar registration.
2. A CBS belief exchange window used for outgoing and incoming CBS beliefs.

The scan-to-map window is not the same thing as the optimizer state. LiORF's
main pose graph is an incremental iSAM2/BPSAM keyframe graph and is not fixed
lag in the same sense as Kimera.

For S3E Alpha, LiORF's CBS exchange window is currently 30 poses:

```text
cbsBeliefExchangeWindowSize = 30
```

So LiORF also sends a window of recent pose beliefs, not only the newest pose.

Relevant files:

- `liorf/config/s3e_alpha.yaml`
- `liorf/launch/run_lio_sam_s3e_alpha.launch`
- `liorf/src/mapOptmization.cpp`

## What Is Exchanged

The exchanged ROS message is a `pose_belief_array`. Each belief carries:

- source agent id: `k` for Kimera or `l` for LiORF;
- pose index;
- timestamp;
- 6D pose mean in Pose3 tangent coordinates;
- 6x6 covariance;
- relaxation/contraction metadata.

Directions in reports:

| direction | meaning |
| --- | --- |
| `L2K` | LiORF belief received by Kimera |
| `K2L` | Kimera belief received by LiORF |

Both sides use known extrinsics to publish/receive beliefs in the expected
exchange frame. The intent is same-drone pose sharing, not unknown inter-robot
frame estimation.

## Factor Semantics

In GTSAM terms, the cross-estimator CBS belief injection currently becomes a
unary pose prior:

```cpp
gtsam::PriorFactor<gtsam::Pose3>(pose_key, prior_value, noise)
```

It does not become a binary pose-pose factor:

```cpp
gtsam::BetweenFactor<gtsam::Pose3>(key_i, key_j, relative_pose, noise)
```

This was explicitly changed by Mike in the `cbs` repository:

| commit | date | author | message |
| --- | --- | --- | --- |
| `b1f0850` | 2026-04-30 19:01:02 +0300 | Mike Xiangyu Liu | `use pose priors for CBS beliefs` |

Before that commit, BPSAM converted CBS beliefs into `BetweenFactor<Pose3>`
factors involving robot/anchor-style keys. After that commit, accepted CBS
beliefs are tracked as removable CBS prior slots and converted to
`PriorFactor<Pose3>` on the receiver's local pose key.

Internal odometry is still represented differently. For example, LiORF odometry
between consecutive lidar keyframes uses `BetweenFactor<Pose3>`. The important
distinction is:

- estimator-internal relative motion: often `BetweenFactor`;
- cross-estimator CBS belief injection: currently `PriorFactor`.

## Outgoing Belief Windows

Both estimators send multiple beliefs per update:

- Kimera sends beliefs for active fixed-lag pose keys, roughly 50 in the S3E
  Alpha configuration.
- LiORF sends beliefs for the recent CBS exchange window, currently 30 poses.

A pose can therefore be sent many times while it remains in the active exchange
window. It is not just "send the newest estimate once." The resent belief may
change over time because the pose estimate and marginal covariance can change
after optimizer updates.

The receiver matches incoming beliefs to local pose keys by pose index or
timestamp, then lets BPSAM decide whether the belief initializes GBP state,
gets accepted, or is rejected.

Common rejection/status categories:

- first message initializes GBP state only;
- inactive receiver window;
- timestamp mismatch;
- missing receiver state;
- invalid covariance/shape;
- BPSAM update rejection;
- root-size guard.

## Covariance Semantics

The intended outgoing covariance is local-only: it should represent the sender's
own estimator information for that pose, not information that already came from
the peer through CBS. This is important to reduce double counting.

The report currently distinguishes:

- sender-side covariance trace before transport;
- receiver-side covariance trace after frame conversion;
- receiver GBP local belief before the BPSAM update;
- receiver GBP local belief after the BPSAM update;
- rotational and translational diagonal-trace splits.

Important caveat: the "local before/after" values in the injected-belief
covariance table are BPSAM/GBP belief summaries around the CBS belief update.
They are useful for seeing the CBS contraction behavior, but they should not be
blindly described as the full final iSAM2 posterior marginal unless that is
explicitly computed and reported.

## Temporary Linear CBS Prior Mode

The first temporary-prior experiment added accepted CBS beliefs as normal
`PriorFactor<Pose3>` factors, let iSAM2 update once, then removed those factor
slots before fixed-lag marginalization. That protected marginalization from
persistent CBS factors, but it was not the same as Mike's intended linear
Bayes-tree surgery.

The newer mode is now the default in the S3E launch files. It is controlled by:

```text
cbs_use_temporary_cbs_linear_priors:=true
```

Use `cbs_use_temporary_cbs_linear_priors:=false` only when deliberately
re-running the legacy persistent-prior path for comparison. The default also
keeps `cbs_enable_soft_reset:=true`, so incoming beliefs are contracted through
GBP soft reset instead of being mostly hard-rejected by the old reset gate.

In this mode, accepted CBS beliefs are still represented as `PriorFactor<Pose3>`
objects, but they are not inserted into `nonlinearFactors_` or `linearFactors_`.
Instead:

1. BPSAM passes the accepted CBS priors through
   `ISAM2UpdateParams::temporaryFactorsForDelta`.
2. iSAM2 performs the normal local update and rebuilds the clean local Bayes
   tree.
3. The temporary priors are linearized at the current iSAM2 linearization point.
4. iSAM2 builds a temporary Gaussian system from the current clean Bayes-tree
   clique conditionals plus those linearized priors.
5. That temporary augmented linear system is solved for the current delta using
   the Bayes-tree elimination ordering.
6. The temporary delta is retracted into the iSAM2 linearization point
   (`theta_`).
7. The temporary Gaussian system is discarded.
8. iSAM2 immediately rebuilds the Bayes tree from the persistent clean graph
   only, linearized at the fused state.
9. The cached delta is reset to zero, so the fused state is the current estimate
   while future marginalization/covariance work still sees only the clean graph.

This means the current state estimate can be pulled by incoming CBS beliefs,
but the stored nonlinear graph, stored linear factors, and rebuilt Bayes tree do
not contain those CBS priors. Fixed-lag marginalization therefore cannot absorb
the received CBS priors into future marginal factors. This now matches Mike's
intended sequence: linearize, manipulate the linear clique system, update,
de-manipulate, then relinearize the clean graph.

The older mode remains available as:

```text
cbs_use_temporary_cbs_prior_factors:=true
```

Use the newer linear-prior mode for Mike's "linearize, manipulate cliques,
update, de-manipulate" idea. Use the older factor-slot mode only as a fallback
or comparison point.

### Temporary Linear Already-Applied Accounting

Temporary-linear priors are non-persistent, but repeated full beliefs for the
same sender/key can still be harmful if each repeat is applied as a fresh
temporary impulse. To avoid that, the receiver now keeps a last-applied
temporary-linear belief per source/local-key pair.

The conservative receiver gate is controlled by:

```text
cbs_temporary_linear_already_applied_gate_enable:=true
cbs_temporary_linear_already_applied_metric_threshold:=0.01
cbs_temporary_linear_already_applied_dmu_threshold:=0.001
cbs_temporary_linear_already_applied_cov_rel_threshold:=0.001
```

When an accepted incoming belief is nearly identical to the last temporary
belief that was actually applied for the same source/key, BPSAM reports:

```text
accepted_but_skipped_already_applied
```

and does not pass that belief to iSAM2 again. The report also writes
`parsed/cbs_temporary_linear_accounting.csv` and summarizes counts in the
`Temporary Linear Accounting` section of `summary.md`.

This is option B in the current design discussion: skip near-identical
already-applied beliefs. It does not yet implement option A, where only the
incremental information difference between the new and last-applied belief is
sent into the temporary linear solve.

The 2026-05-04 60 s comparison runs showed:

| run | Kimera APE | LiORF APE | temporary-linear accounting signal |
| --- | --- | --- | --- |
| CBS off | 1.3575 | 0.5753 | no CBS transport |
| K2L only | 1.3575 | 1.9724 | 2208 K2L temporary applications |
| L2K only, normal sender gate | 1.3575 | 0.5805 | no L2K applications beyond first-message initialization |
| L2K only, LiORF send-all | 45.7857 | 0.6050 | 152 L2K applications, 3764 repeated impulses prevented |
| two-way | 0.7693 | 5.9401 | 2475 K2L and 189 L2K temporary applications |

The duplicate gate works mechanically: in the LiORF send-all stress case it
skipped thousands of repeated L2K beliefs. However, Kimera still diverged from
only 152 first-time L2K temporary applications, and LiORF still degraded in the
two-way case. Therefore the next debugging target is not only repeated
identical impulses. We still need to inspect mean/frame/timestamp consistency,
the strength and semantics of first-time priors, and whether the temporary
linear factor should be an incremental information update rather than a full
posterior belief.

### Pre-Injection Residual Diagnostics

Accepted CBS priors now emit `CBS_PREINJECTION_RESIDUAL_ROW` immediately before
they are passed into the temporary linear solve or prior-factor path. The row
compares the receiver's current local pose estimate against the incoming prior
pose:

```text
Logmap(T_receiver_current^-1 * T_incoming_prior)
```

The report writes:

```text
parsed/cbs_preinjection_residuals.csv
parsed/preinjection_residual_summary.csv
```

and adds a `Pre-Injection Residuals` table to `summary.md`. The table reports
SE(3) residual norm, rotational norm, translational norm in meters, absolute
yaw error, and incoming covariance trace. This is the direct check for whether
a belief is smooth in the sender sequence but still incompatible with the
receiver's matched pose before injection.

The 2026-05-04 pre-injection diagnostic runs showed:

| run | direction | applied | trans p50 | trans p95 | trans max | yaw p95 |
| --- | --- | --- | --- | --- | --- | --- |
| K2L only | K2L | 2206 | 2.2802 | 8.0925 | 10.6771 | 11.3853 |
| L2K send-all | L2K | 153 | 22.5286 | 99.1683 | 193.5304 | 76.6892 |
| two-way | K2L | 2441 | 1.4452 | 8.7898 | 16.5034 | 16.7821 |
| two-way | L2K | 134 | 1.4024 | 4.9620 | 5.0723 | 5.8808 |

The L2K send-all case confirms that the forced LiORF-to-Kimera priors become
very incompatible with Kimera's current matched poses before injection. The
two-way case is less extreme for L2K because the receiver gates reject most of
the later LiORF messages, but K2L still has large tail residuals and LiORF
degrades.

## Rerun Visualization

Rerun is used for live and recorded visualization. The workspace has a blueprint
file:

```text
cbsms/cbsms.rbl
```

Typical entities shown include:

- LiORF trajectory, current pose, local map, factor graph, timing;
- Kimera trajectory and CBS belief visualizations;
- CBS counters such as published/received beliefs per update;
- optimization timing;
- covariance/uncertainty plots;
- local and global point cloud/map views.

The visual alignment used for Rerun is for interpretation and comparison. It is
not the same thing as changing estimator outputs or changing the optimization.

## Standard Run And Reports

The standard report runner is:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --name s3e-alpha-report \
  --duration 60
```

Each run writes a directory under:

```text
/home/yeranis/repos/V4RL/runs/
```

Important report artifacts:

| artifact | purpose |
| --- | --- |
| `summary.md` | human-readable report |
| `summary.json` | machine-readable report |
| `roslaunch.log` | raw ROS log |
| `trajectories/` | raw and TUM trajectories |
| `parsed/cbs_transport.csv` | belief transport and covariance frame conversion rows |
| `parsed/cbs_roundtrip.csv` | transform/covariance roundtrip checks |
| `parsed/cbs_merge_k2l.csv` | LiORF receiving Kimera belief decisions |
| `parsed/cbs_bpsam_add.csv` | accepted/rejected BPSAM belief rows |
| `parsed/cbs_temporary_linear_accounting.csv` | temporary-linear applied/skipped accounting |
| `parsed/cbs_preinjection_residuals.csv` | receiver pose vs incoming prior residual before injection |
| `parsed/injected_belief_covariance_samples.csv` | full covariance sample details |
| `parsed/evo_metrics.csv` | evo APE/RPE results |

The Markdown report includes compact 40-sample covariance tables for accepted
`L2K` and `K2L` injected beliefs. The CSV keeps the full diagonal strings for
deeper inspection.

## Evaluation Meaning

The report currently computes:

- trajectory RMSE against S3E ground truth;
- evo APE and RPE using TUM files;
- Kimera-vs-LiORF disagreement after alignment;
- CBS transport counts;
- BPSAM acceptance/rejection counts;
- covariance transport and roundtrip checks;
- injected-belief covariance samples.

Ground-truth metrics evaluate estimator accuracy. Kimera-vs-LiORF metrics
evaluate disagreement between the two estimators; they are not ground-truth
accuracy.

## Current Mental Model

At a high level, each run behaves like this:

1. Kimera and LiORF estimate the same drone trajectory from different sensors.
2. Each backend produces local pose beliefs for recent active poses.
3. The ROS bridge converts those pose beliefs into the agreed exchange frame.
4. The receiving side matches the belief to a local pose key.
5. BPSAM performs GBP/contraction logic.
6. In the legacy path, accepted beliefs become tracked `PriorFactor<Pose3>`
   factors on the receiver pose key.
7. In temporary linear-prior mode, accepted beliefs affect the current linear
   solve, the resulting fused delta is committed to the state, and the clean
   graph is immediately relinearized without inserting persistent CBS factors.
8. Stale legacy CBS prior factors are removed when they fall outside the active
   CBS window.
9. Reports summarize trajectory accuracy, belief traffic, acceptance/rejection,
   covariance transport, and injected belief covariance behavior.

## Open Questions To Keep Tracking

- Whether pose priors are the final preferred modeling choice for same-drone
  LiORF/Kimera sharing, or whether a different factor form should be revisited.
- Whether outgoing local-only covariance extraction is conservative enough to
  avoid double counting in all modes.
- Whether the LiORF covariance scale is dominated by anchoring/gauge choices in
  some settings.
- Which CBS belief window sizes are best for stability and computation.
- Whether rejection thresholds and first-message behavior should be reported as
  run parameters in every `summary.md`.
- Whether final receiver posterior marginal covariance should be added as a
  separate report column distinct from GBP before/after belief summaries.
