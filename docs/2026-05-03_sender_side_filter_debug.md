# 2026-05-03 Sender-Side Belief Filter Debug Note

This note records the CBS/BPSAM debugging changes made on 2026-05-03 while
investigating whether receiver-side Hellinger rejection was comparing the wrong
beliefs.

## Question

Zak's hypothesis was:

- In the CBS paper, before sending a belief, the sender compares the current
  outgoing belief against the previous belief that same sender sent.
- The receiver should not reject a LiORF belief just because LiORF and Kimera
  currently disagree in pose.
- If the receiver compares the current incoming sender belief against the
  receiver's own current local estimate, that would be wrong for this project.

## Code Changes

### CBS / BPSAM

File:

```text
/home/yeranis/repos/V4RL/src/cbs/src/bpsam/bpsam.cpp
```

Added sender-side outgoing belief filtering and logging in the robot-aware
overload:

```cpp
BPSAM::getBeliefs(KeySet keys, AgentId requesting_robot, bool use_other_robots)
```

The new code compares:

```text
current outgoing belief
vs
last belief returned by this BPSAM instance for the same receiver/key/producer
```

It logs one `CBS_OUTGOING_FILTER_ROW` for each candidate outgoing belief with:

- direction, for example `K2L` or `L2K`;
- sender robot;
- receiver robot;
- producing agent;
- belief key;
- metric type;
- similarity threshold;
- whether a previous sent belief exists;
- metric distance;
- previous and current covariance trace;
- mean delta norm;
- status.

Statuses:

| status | meaning |
| --- | --- |
| `sent_first_message` | no previous sent belief exists for this receiver/key |
| `sent_changed` | belief passed the sender-side filter and was sent |
| `filtered_similar` | belief was omitted because distance was below threshold |
| `sent_metric_error` | metric computation failed, so the belief was sent |
| `sent_filter_disabled` | threshold is disabled |

The filter updates `last_sent_beliefs_` only with beliefs actually returned for
publishing.

### Kimera-VIO

File:

```text
/home/yeranis/repos/V4RL/src/Kimera-VIO/src/backend/VioBackend.cpp
```

Changed Kimera outgoing belief collection to call the robot-aware BPSAM API:

```cpp
smoother_->getBeliefs(request_keys, static_cast<cbs::AgentId>('l'), false);
```

This activates sender-side filtering for Kimera-to-LiORF (`K2L`) messages and
prevents relaying other agents' GBP beliefs.

### LiORF

Files:

```text
/home/yeranis/repos/V4RL/src/liorf/src/mapOptmization.cpp
/home/yeranis/repos/V4RL/src/liorf/CMakeLists.txt
```

Changed LiORF outgoing belief collection to call the robot-aware BPSAM API:

```cpp
bpsam->getBeliefs(requestKeys, static_cast<cbs::AgentId>('k'), false);
```

This activates sender-side filtering for LiORF-to-Kimera (`L2K`) messages and
prevents relaying other agents' GBP beliefs.

Also fixed a LiORF build issue caused by OpenCV header/library mismatch. LiORF
was compiling against workspace OpenCV 3 headers from `opencv3_catkin`, but
linking mostly against system/ROS OpenCV 4. The local fix was to link LiORF
against `opencv3_catkin` and remove `cv_bridge` from `catkin_LIBRARIES`, since
LiORF only includes the header and does not call `cv_bridge` symbols.

The build still prints OpenCV 3/4 conflict warnings because `aria_viz` pulls
OpenCV 4, but the LiORF executable builds successfully.

### Report Parser

File:

```text
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py
```

Added parsing and reporting for:

```text
CBS_OUTGOING_FILTER_ROW
```

New artifacts:

```text
parsed/cbs_outgoing_filter.csv
parsed/outgoing_filter_summary.csv
```

New `summary.md` section:

```text
Sender-Side Belief Filter
```

## Validation Run

Run:

```text
/home/yeranis/repos/V4RL/runs/20260503-000957_sender-filter-60s-soft-reset-off
```

Report:

```text
/home/yeranis/repos/V4RL/runs/20260503-000957_sender-filter-60s-soft-reset-off/summary.md
```

Command:

```bash
/home/yeranis/repos/V4RL/src/cbsms/tools/cbsms_experiment.py run \
  --name sender-filter-60s-soft-reset-off \
  --duration 60 \
  --no-rerun-visualizer-enable \
  --extra-arg cbs_enable_soft_reset:=false
```

Important results:

| item | K2L | L2K |
| --- | ---: | ---: |
| sender-side `filtered_similar` | 603 | 1495 |
| sender-side `sent_changed` | 12284 | 2652 |
| receiver accepted | 103 | 26 |
| receiver rejected update status | 5965 | 2576 |
| rejected Hellinger p95 | 0.9998 | 0.9993 |

Trajectory metrics from the same run:

| estimator | APE RMSE | RPE RMSE |
| --- | ---: | ---: |
| Kimera | 0.8655 m | 0.4804 m |
| LiORF | 1.7790 m | 0.8120 m |

This run is useful as a diagnostic run, not as the best current performance
configuration. The previous 60 s CBS-on diagnostic run without this sender-side
path had better APE values:

```text
/home/yeranis/repos/V4RL/runs/20260502-232210_l2k-report-60s-soft-reset-off
```

Previous run APE:

| estimator | APE RMSE |
| --- | ---: |
| Kimera | 0.3453 m |
| LiORF | 0.6353 m |

## Current Interpretation

The receiver is not directly comparing the incoming sender belief against the
receiver's own local iSAM2 pose estimate.

However, the receiver also is not comparing against a raw immutable
`last_received_from_sender` cache. The current BPSAM receiver-side update path
compares the incoming belief against the receiver's existing GBP belief for
that sender/key. That GBP belief is the receiver's maintained peer-belief copy.
It may have been initialized or contracted by previous incoming beliefs, and if
soft reset is off, a large Hellinger distance can reject the update and leave
that GBP copy stale.

This means high receiver-side Hellinger does not necessarily prove the receiver
is comparing LiORF against Kimera's own local estimate. It may mean:

- the incoming sender belief really changed too much relative to the previous
  sender belief;
- or the receiver's GBP peer-belief copy became stale because earlier large
  changes were rejected;
- or covariance/mean evolution makes the Hellinger distance near 1 even when
  the raw sender sequence is reasonable.

## Recommended Next Diagnostic

Add a receiver-side raw cache:

```text
last_received_beliefs_[source_agent][receiver_key]
```

Then report both distances for every incoming belief:

1. `incoming_current` vs `incoming_previous_raw_from_same_sender`
2. `incoming_current` vs `receiver_existing_gbp_peer_belief`

That will separate the CBS-paper sender-sequence question from the current GBP
contraction/staleness question.

## Follow-Up Receiver Diagnostic

Implemented after the recommendation above.

Additional BPSAM change:

- added `last_received_beliefs_[source_agent][belief_key]`;
- logged `CBS_RECEIVER_DIAGNOSTIC_ROW` for each incoming belief;
- compared current incoming belief against both:
  - previous raw incoming belief from the same sender/key;
  - receiver's existing GBP peer belief before the BPSAM update.

Additional report artifacts:

```text
parsed/cbs_receiver_diagnostics.csv
parsed/receiver_diagnostic_summary.csv
```

Additional `summary.md` section:

```text
Receiver Belief Diagnostics
```

Validation run:

```text
/home/yeranis/repos/V4RL/runs/20260503-003944_receiver-diagnostics-60s-soft-reset-off
```

Report:

```text
/home/yeranis/repos/V4RL/runs/20260503-003944_receiver-diagnostics-60s-soft-reset-off/summary.md
```

Key result:

| direction/status | raw H p50 | raw H p95 | GBP H p50 | GBP H p95 | GBP-raw p95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `K2L` rejected update status | 0.0811 | 0.3349 | 0.6720 | 0.9999 | 0.9110 |
| `L2K` rejected update status | 0.0564 | 0.5935 | 0.5395 | 0.9490 | 0.8472 |

This supports the stale/diverged receiver-GBP-copy hypothesis. Many rejected
beliefs are not huge jumps relative to the sender's previous raw incoming
belief, but they are huge relative to the receiver's stored GBP peer belief.

Concrete counts from rejected updates:

| direction | condition | count | share |
| --- | --- | ---: | ---: |
| `K2L` | raw H `< 0.1` and GBP H `> 0.5` | 2274 / 5717 | 39.8% |
| `K2L` | raw H `< 0.2` and GBP H `> 0.5` | 3332 / 5717 | 58.3% |
| `L2K` | raw H `< 0.1` and GBP H `> 0.5` | 876 / 2666 | 32.9% |
| `L2K` | raw H `< 0.2` and GBP H `> 0.5` | 1054 / 2666 | 39.5% |

Interpretation: receiver rejection is not explained only by the sender sending
wildly different consecutive beliefs. Receiver GBP state can become stale after
rejections, so comparing new incoming beliefs against that stale GBP copy can
keep rejecting later beliefs.

## Soft Reset On Check

Validation run:

```text
/home/yeranis/repos/V4RL/runs/20260503-010135_receiver-diagnostics-60s-soft-reset-on
```

Report:

```text
/home/yeranis/repos/V4RL/runs/20260503-010135_receiver-diagnostics-60s-soft-reset-on/summary.md
```

With `cbs_enable_soft_reset:=true`, receiver-side `rejected_update_status`
dropped to zero in both directions.

| direction | accepted | first-message rejected | update-status rejected |
| --- | ---: | ---: | ---: |
| `K2L` | 6147 | 158 | 0 |
| `L2K` | 3922 | 166 | 0 |

Trajectory metrics:

| estimator | APE RMSE | RPE RMSE |
| --- | ---: | ---: |
| Kimera | 0.6845 m | 0.4866 m |
| LiORF | 0.5416 m | 0.7034 m |

Soft reset therefore confirms that the hard Hellinger reset/reject gate was the
source of the thousands of `rejected_update_status` rows. It does not by itself
prove that soft reset is the final best behavior: it accepts many high-distance
updates and the trajectory result is mixed compared with the soft-reset-off
diagnostic run.
