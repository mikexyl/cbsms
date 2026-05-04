# CBSMS Experiment Reports

Use `tools/cbsms_experiment.py` from the host to run the standard S3E Alpha
LiORF/Kimera CBS experiment and produce a reproducible report.

## Standard Run

Start Rerun separately if you want live visualization:

```bash
rerun --bind 0.0.0.0 --port 9876 /home/yeranis/repos/V4RL/src/cbsms/cbsms.rbl
```

Then launch the experiment/report runner:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --name s3e-square1-cbs \
  --duration 60
```

For day-to-day use, the shorter wrapper is:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/run_s3e_report.sh cbs-on 60
```

It runs headless by default, creates a timestamped run folder under
`/home/yeranis/repos/V4RL/runs/`, and prints the generated `summary.md` path.
Use these common commands:

```bash
# CBS on, 60 s, report only
/home/yeranis/repos/V4RL/src/cbsms/tools/run_s3e_report.sh cbs-on 60

# CBS off baseline, 60 s, report only
/home/yeranis/repos/V4RL/src/cbsms/tools/run_s3e_report.sh cbs-off 60

# CBS on with live Rerun streaming to an already-open viewer
/home/yeranis/repos/V4RL/src/cbsms/tools/run_s3e_report.sh cbs-on 60 rerun
```

The runner:

- creates a run directory under `/home/yeranis/repos/V4RL/runs/`;
- records the exact launch arguments and git state of the core repos;
- runs `s3e_alpha_liorf_kimera_experiment.launch` inside `cbsms_ws`;
- records `/kimera_vio_ros/odometry` and `/liorf/mapping/odometry` to CSV;
- parses CBS transport, roundtrip, merge, and BPSAM add rows from the ROS log;
- aligns the odometry to S3E `alpha_gt.txt` using planar yaw+translation;
- exports TUM trajectories and runs `evo_ape`/`evo_rpe` when `evo` is
  installed;
- writes `summary.md`, `summary.json`, and parsed CSV files.

## Important Artifacts

Each run directory contains:

- `summary.md`: human-readable report.
- `summary.json`: machine-readable summary.
- `manifest.json`: launch args, git commits, dirty state, recorded topics.
- `roslaunch.log`: raw ROS launch output.
- `trajectories/kimera_odometry.csv`: raw Kimera odometry.
- `trajectories/liorf_odometry.csv`: raw LiORF odometry.
- `trajectories/tum/`: TUM-format trajectories for evo.
- `parsed/cbs_transport.csv`: per-belief transport/covariance transform rows.
- `parsed/cbs_roundtrip.csv`: covariance and pose roundtrip checks.
- `parsed/cbs_merge_k2l.csv`: LiORF receiving Kimera belief merge decisions.
- `parsed/cbs_bpsam_add.csv`: BPSAM accept/reject details for both directions.
- `parsed/cbs_receiver_diagnostics.csv`: receiver-side raw previous incoming
  vs GBP peer belief diagnostics for each valid incoming CBS belief.
- `parsed/injected_belief_covariance_samples.csv`: first 40 accepted injected
  belief covariance samples per direction.
- `parsed/trajectory_metrics.csv`: RMSE and inter-estimator error metrics.
- `parsed/evo_metrics.csv`: parsed `evo_ape` and `evo_rpe` statistics.
- `parsed/evo/*.zip`: raw evo result archives.

## Common Variants

CBS-disabled baseline:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --name s3e-square1-no-cbs \
  --duration 60 \
  --no-enable-cbs-bridge
```

Run without Rerun streaming:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --name s3e-square1-headless \
  --duration 60 \
  --no-rerun-visualizer-enable
```

Pass an extra launch argument:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --name tolerance-test \
  --duration 60 \
  --extra-arg liorf_belief_timestamp_tolerance_sec:=0.2
```

Regenerate the report after parser changes:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py report \
  /home/yeranis/repos/V4RL/runs/<run-id>
```

## Metric Notes

`kimera_vs_ground_truth` and `liorf_vs_ground_truth` are position RMSE values
after planar SE2 alignment to S3E ground truth. This is appropriate for quick
S3E report comparison, but it is not a full SE3 trajectory evaluation.

The `Evo Metrics` section uses `evo_ape` and `evo_rpe` on TUM files. Ground
truth is interpolated to the estimator timestamps before evo is called, so evo
does exact timestamp association on the generated `*_eval.tum` files. The
default evo settings are:

- APE: translation part, Umeyama SE3 alignment, no scale correction.
- RPE: translation part, Umeyama SE3 alignment, no scale correction,
  `delta=1 frame`.

Do not use scale correction by default for this stack; LiORF and Kimera VIO are
expected to be metric-scale estimators.

`kimera_vs_liorf` is an inter-estimator disagreement metric after planar
alignment. It is useful for seeing where the two estimators disagree; it is not
ground-truth accuracy.

`L2K` means a LiORF belief received by Kimera. `K2L` means a Kimera belief
received by LiORF.

The `Injected Belief Covariance Samples` section reports the first 40 accepted
BPSAM updates per direction. For each accepted injected belief, it shows:

- sender-side covariance trace before transport;
- receiver-side covariance trace after frame conversion;
- receiver GBP belief covariance trace before the BPSAM update;
- receiver GBP belief covariance trace after the BPSAM update;
- rotational and translational diagonal-trace splits for the 6D Pose3
  tangent-space covariance used by BPSAM.

The Markdown table intentionally reports total trace plus rotational and
translational diagonal-trace splits instead of full 6x6 matrices so
`summary.md` remains shareable. The full `diag6` strings remain in
`parsed/injected_belief_covariance_samples.csv`; use the raw log when full
transport rows or BPSAM messages are needed.

The `Receiver Belief Diagnostics` section compares two distances for incoming
beliefs:

- raw: current incoming belief vs previous raw incoming belief from the same
  sender/key;
- GBP: current incoming belief vs the receiver's existing GBP peer belief for
  that sender/key before the update.

If raw Hellinger stays modest while GBP Hellinger is high, the sender sequence
is not the main problem; the receiver's GBP peer belief is stale or diverged.
