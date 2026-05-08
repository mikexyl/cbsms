# Rerun Commands

Use this flow to inspect the S3E LiORF/Kimera CBS run live in the Rerun
viewer. Start the viewer first, then start the experiment.

## Terminal 1: Rerun Viewer

```bash
cd /home/yeranis/repos/V4RL
rerun --bind 0.0.0.0 --port 9876 src/cbsms/cbsms.rbl
```

## Terminal 2: 60s CBS Run

```bash
cd /home/yeranis/repos/V4RL

/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL \
  --name adjacent-60s-cov-diagnostic-rerun \
  --duration 60 \
  --timeout-padding 90 \
  --no-use-kimera-rviz \
  --no-use-liorf-rviz \
  --no-kimera-visualize \
  --rerun-visualizer-enable \
  --rerun-host rerun+http://172.17.0.1:9876/proxy
```

The standard launch path is adjacent-only odometry belief exchange. The old
multi-horizon interval mode is intentionally not part of this command.

## Optional Overrides

Use these only when deliberately testing a specific behavior:

```bash
--extra-arg cbs_odom_unmatched_retry_max_beliefs:=0
--extra-arg liorf_belief_exchange_window_size:=30
--extra-arg cbs_enable_soft_reset:=true
```

## Useful Output Files

After the run finishes, inspect the generated run directory under:

```text
/home/yeranis/repos/V4RL/runs/
```

The CBS odometry covariance diagnostics are written to:

```text
parsed/cbs_odom_factor_covariance.csv
parsed/odom_factor_covariance_summary.csv
parsed/odom_factor_covariance_samples.csv
```

