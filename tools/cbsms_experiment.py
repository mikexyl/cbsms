#!/usr/bin/env python3
"""Run CBSMS experiments and generate reproducible reports.

The runner is intentionally host-side: it starts the existing Docker container,
captures roslaunch output, records the estimator odometry topics as CSV, then
parses the resulting artifacts into a compact report.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import shutil
import shlex
import signal
import subprocess
import sys
import time
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_CONTAINER = "cbsms_ws"
DEFAULT_BAG_PATH = "/data/s3e/S3E_Square_1/S3E_Square_1_alpha_ros1.bag"
DEFAULT_GT_RELATIVE = "cbs_fresh_ws/datasets/S3E/S3E_Square_1/alpha_gt.txt"
DEFAULT_RERUN_HOST = "rerun+http://172.17.0.1:9876/proxy"
CORE_REPOS = ["cbsms", "cbs", "liorf", "Kimera-VIO", "Kimera-VIO-ROS"]
ANSI_RE = re.compile(
    r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~]|\][^\a]*(?:\a|\x1b\\))"
)
ROW_MARKERS = (
    "CBS_TRANSPORT_ROW_L2K",
    "CBS_TRANSPORT_ROW_K2L",
    "CBS_ROUNDTRIP_ROW_L2K",
    "CBS_ROUNDTRIP_ROW_K2L",
    "CBS_MERGE_ROW_L2K",
    "CBS_MERGE_ROW_K2L",
    "CBS_BPSAM_ADD_ROW_L2K",
    "CBS_BPSAM_ADD_ROW_K2L",
    "CBS_OUTGOING_FILTER_ROW",
    "CBS_RECEIVER_DIAGNOSTIC_ROW",
    "CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW",
    "CBS_PREINJECTION_RESIDUAL_ROW",
    "CBS_TEMPORARY_LINEARIZATION_RESIDUAL_ROW",
    "CBS_KIMERA_OUTGOING_PROVENANCE_ROW",
    "CBS_MARGINALIZATION_GRAPH_ROW",
)
VALID_BPSAM_STATUSES = {
    "accepted",
    "accepted_but_skipped_already_applied",
    "rejected_shape",
    "rejected_first_message",
    "rejected_update_status",
    "rejected_inactive_window",
    "rejected_exception",
    "unknown",
}
BPSAM_UPDATE_NUMBER_RE = re.compile(
    r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?"
)
MERGE_STATUSES = {
    "bpsam_added",
    "bpsam_accepted_but_skipped_already_applied",
    "bpsam_rejected_first_message",
    "bpsam_rejected_update_status",
    "bpsam_rejected_shape",
    "bpsam_rejected_inactive_window",
    "bpsam_rejected_exception",
    "bpsam_rejected_root_size",
    "bpsam_rejected",
    "dropped_no_local_timestamp",
    "dropped_invalid_stamp",
    "dropped_timestamp_mismatch",
    "dropped_window",
    "dropped_missing_state",
    "dropped_bad_covariance",
}
OUTGOING_FILTER_STATUSES = {
    "sent_filter_disabled",
    "sent_first_message",
    "sent_metric_error",
    "sent_changed",
    "filtered_similar",
}
TEMPORARY_LINEAR_ACCOUNTING_ACTIONS = {
    "temporary_linear_applied",
    "accepted_but_skipped_already_applied",
    "applied_incremental",
}
PREINJECTION_RESIDUAL_ACTIONS = {
    "temporary_linear_applied",
    "temporary_prior_factor_applied",
    "persistent_prior_applied",
}
RECEIVER_METRIC_STATUSES = {
    "ok",
    "no_previous",
    "no_peer",
}


Point = Tuple[float, float, float]
Trajectory = List[Tuple[float, Point]]
PoseRow = Dict[str, float]
PoseTrajectory = List[PoseRow]


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[3]


def iso_now() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def slugify(text: str) -> str:
    text = text.strip().lower()
    text = re.sub(r"[^a-z0-9_.-]+", "-", text)
    text = text.strip("-")
    return text or "run"


def run_command(
    args: Sequence[str],
    cwd: Optional[Path] = None,
    env: Optional[Dict[str, str]] = None,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(args),
        cwd=str(cwd) if cwd else None,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def git_info(repo: Path) -> Dict[str, Any]:
    def git(*args: str) -> str:
        result = run_command(["git", "-C", str(repo), *args])
        return result.stdout.strip()

    if not (repo / ".git").exists():
        return {"present": False}

    status = git("status", "--short")
    return {
        "present": True,
        "path": str(repo),
        "branch": git("branch", "--show-current"),
        "commit": git("rev-parse", "HEAD"),
        "commit_short": git("rev-parse", "--short", "HEAD"),
        "remote": git("remote", "get-url", "origin"),
        "dirty": bool(status),
        "status_short": status.splitlines(),
    }


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def container_path(workspace: Path, host_path: Path) -> str:
    host_path = host_path.resolve()
    workspace = workspace.resolve()
    try:
        rel = host_path.relative_to(workspace)
    except ValueError:
        raise ValueError(f"{host_path} is not under workspace {workspace}")
    return "/workspace/" + str(rel).replace(os.sep, "/")


def docker_bash(container: str, command: str) -> List[str]:
    return ["docker", "exec", container, "bash", "-lc", command]


def ros_env_command(command: str) -> str:
    return (
        "source /opt/ros/noetic/setup.bash && "
        "cd /workspace && "
        "source devel/setup.bash && "
        f"{command}"
    )


def list_container_processes(container: str) -> List[Tuple[int, str]]:
    result = run_command(
        ["docker", "exec", container, "ps", "-eo", "pid=", "-o", "cmd="]
    )
    processes: List[Tuple[int, str]] = []
    if result.returncode != 0:
        return processes
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, command = stripped.partition(" ")
        pid = to_int(pid_text)
        if pid > 0:
            processes.append((pid, command.strip()))
    return processes


def interrupt_container_processes(container: str, needles: Sequence[str]) -> None:
    pids = [
        str(pid)
        for pid, command in list_container_processes(container)
        if any(needle in command for needle in needles)
    ]
    if pids:
        run_command(["docker", "exec", container, "kill", "-INT", *pids])


def stop_existing_experiment(container: str) -> None:
    interrupt_container_processes(
        container,
        [
            "s3e_alpha_liorf_kimera_experiment.launch",
            "s3e_alpha_liorf_kimera_topics.launch",
            "rostopic echo -p /kimera_vio_ros/odometry",
            "rostopic echo -p /liorf/mapping/odometry",
        ],
    )


def wait_for_ros_master(container: str, timeout_sec: float) -> bool:
    deadline = time.time() + timeout_sec
    probe = ros_env_command("rostopic list >/dev/null 2>&1")
    while time.time() < deadline:
        result = run_command(docker_bash(container, probe))
        if result.returncode == 0:
            return True
        time.sleep(0.5)
    return False


def start_rostopic_csv(
    container: str,
    topic: str,
    output_container_path: str,
    stderr_path: Path,
) -> subprocess.Popen:
    command = ros_env_command(
        f"rostopic echo -p {shlex.quote(topic)} > {shlex.quote(output_container_path)}"
    )
    err = stderr_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        docker_bash(container, command),
        stdout=subprocess.DEVNULL,
        stderr=err,
        text=True,
    )


def terminate_process(proc: subprocess.Popen, timeout_sec: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout_sec)


def run_experiment(args: argparse.Namespace) -> Path:
    workspace = args.workspace.resolve()
    runs_root = args.output_root.resolve() if args.output_root else workspace / "runs"
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_id = f"{stamp}_{slugify(args.name)}" if args.name else stamp
    run_dir = runs_root / run_id
    traj_dir = run_dir / "trajectories"
    run_dir.mkdir(parents=True, exist_ok=False)
    traj_dir.mkdir(parents=True, exist_ok=True)

    git_repos = {
        name: git_info(workspace / "src" / name)
        for name in CORE_REPOS
    }

    gt_path = args.gt_path.resolve() if args.gt_path else workspace / DEFAULT_GT_RELATIVE
    launch_args = {
        "bag_path": args.bag_path,
        "bag_duration": str(args.duration),
        "enable_cbs_bridge": str(args.enable_cbs_bridge).lower(),
        "use_kimera_rviz": str(args.use_kimera_rviz).lower(),
        "use_liorf_rviz": str(args.use_liorf_rviz).lower(),
        "kimera_visualize": str(args.kimera_visualize).lower(),
        "rerun_visualizer_enable": str(args.rerun_visualizer_enable).lower(),
        "rerun_world_alignment_enable": str(args.rerun_world_alignment_enable).lower(),
        "rerun_host": args.rerun_host,
    }
    for item in args.extra_arg:
        if ":=" not in item:
            raise ValueError(f"extra launch arg must look like key:=value: {item}")
        key, value = item.split(":=", 1)
        launch_args[key] = value

    launch_target = (
        f"{args.launch_package} {args.launch_file}".strip()
        if args.launch_package
        else args.launch_file
    )

    manifest: Dict[str, Any] = {
        "run_id": run_id,
        "created_at": iso_now(),
        "workspace": str(workspace),
        "container": args.container,
        "launch_file": launch_target,
        "launch_args": launch_args,
        "duration_sec": args.duration,
        "timeout_padding_sec": args.timeout_padding,
        "ground_truth": str(gt_path),
        "git": git_repos,
        "recorded_topics": {},
    }
    write_json(run_dir / "manifest.json", manifest)

    if args.clean_start:
        stop_existing_experiment(args.container)
        time.sleep(1.0)

    launch_arg_text = " ".join(
        f"{key}:={shlex.quote(value)}" for key, value in launch_args.items()
    )
    if args.launch_package:
        roslaunch_target = (
            f"{shlex.quote(args.launch_package)} {shlex.quote(args.launch_file)}"
        )
    else:
        roslaunch_target = shlex.quote(args.launch_file)
    launch_command = ros_env_command(
        f"roslaunch {roslaunch_target} " + launch_arg_text
    )
    roslaunch_log = run_dir / "roslaunch.log"
    with roslaunch_log.open("w", encoding="utf-8", errors="replace") as log_file:
        proc = subprocess.Popen(
            docker_bash(args.container, launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )

        topic_procs: List[subprocess.Popen] = []
        if args.record_trajectories and wait_for_ros_master(args.container, 30.0):
            topics = {
                "kimera_odometry": "/kimera_vio_ros/odometry",
                "liorf_odometry": "/liorf/mapping/odometry",
            }
            for label, topic in topics.items():
                csv_path = traj_dir / f"{label}.csv"
                err_path = traj_dir / f"{label}.stderr.log"
                topic_procs.append(
                    start_rostopic_csv(
                        args.container,
                        topic,
                        container_path(workspace, csv_path),
                        err_path,
                    )
                )
                manifest["recorded_topics"][label] = {
                    "topic": topic,
                    "csv": str(csv_path),
                }
            write_json(run_dir / "manifest.json", manifest)

        deadline = time.time() + args.duration + args.timeout_padding
        while proc.poll() is None and time.time() < deadline:
            time.sleep(1.0)

        if proc.poll() is None:
            stop_existing_experiment(args.container)
            try:
                proc.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5.0)

        for topic_proc in topic_procs:
            terminate_process(topic_proc)

    manifest["finished_at"] = iso_now()
    manifest["roslaunch_returncode"] = proc.returncode
    write_json(run_dir / "manifest.json", manifest)

    generate_report(run_dir, gt_path)
    return run_dir


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def parse_csv_rows_from_line(line: str, marker: str) -> List[List[str]]:
    rows: List[List[str]] = []
    start = 0
    while True:
        idx = line.find(marker, start)
        if idx < 0:
            break
        next_idx = min(
            (
                pos
                for row_marker in ROW_MARKERS
                if (pos := line.find(row_marker, idx + len(marker))) >= 0
            ),
            default=len(line),
        )
        payload = line[idx:next_idx].strip()
        try:
            rows.append(next(csv.reader([payload])))
        except csv.Error:
            pass
        start = idx + len(marker)
    return rows


def to_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def timestamp_to_seconds(value: float) -> float:
    if math.isfinite(value) and value > 1.0e12:
        return value * 1.0e-9
    return value


def to_int(value: str) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def parse_vector_token(token: str) -> List[float]:
    if not token.startswith("v[") or not token.endswith("]"):
        return []
    return [to_float(part) for part in token[2:-1].split(";")]


def parse_matrix_token(token: str) -> List[List[float]]:
    if not token.startswith("m[") or not token.endswith("]"):
        return []
    rows: List[List[float]] = []
    for row in token[2:-1].split("|"):
        rows.append([to_float(part) for part in row.split(";")])
    return rows


def matrix_trace(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    n = min(len(matrix), min((len(row) for row in matrix), default=0))
    return sum(matrix[i][i] for i in range(n))


def matrix_frobenius(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    total = 0.0
    for row in matrix:
        for value in row:
            if math.isfinite(value):
                total += value * value
    return math.sqrt(total)


def matrix_diag_min(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    n = min(len(matrix), min((len(row) for row in matrix), default=0))
    vals = [matrix[i][i] for i in range(n) if math.isfinite(matrix[i][i])]
    return min(vals) if vals else math.nan


def matrix_diag_max(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    n = min(len(matrix), min((len(row) for row in matrix), default=0))
    vals = [matrix[i][i] for i in range(n) if math.isfinite(matrix[i][i])]
    return max(vals) if vals else math.nan


def numeric_stats(values: Iterable[float]) -> Dict[str, Any]:
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {"count": 0, "mean": math.nan, "min": math.nan, "max": math.nan, "sum": 0.0}
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "min": min(clean),
        "max": max(clean),
        "sum": sum(clean),
    }


def percentile(values: Iterable[float], q: float) -> float:
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * q
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return clean[low]
    return clean[low] * (high - rank) + clean[high] * (rank - low)


def parse_log_artifacts(log_paths: Sequence[Path]) -> Dict[str, Any]:
    transport_rows: List[Dict[str, Any]] = []
    roundtrip_rows: List[Dict[str, Any]] = []
    merge_rows: List[Dict[str, Any]] = []
    bpsam_rows: List[Dict[str, Any]] = []
    outgoing_filter_rows: List[Dict[str, Any]] = []
    receiver_diagnostic_rows: List[Dict[str, Any]] = []
    temporary_linear_accounting_rows: List[Dict[str, Any]] = []
    preinjection_residual_rows: List[Dict[str, Any]] = []
    temporary_linearization_residual_rows: List[Dict[str, Any]] = []
    provenance_rows: List[Dict[str, Any]] = []
    marginalization_graph_rows: List[Dict[str, Any]] = []
    kimera_flow_rows: List[Dict[str, int]] = []
    alignment_rows: List[Dict[str, float]] = []
    skipped_rows: Counter[str] = Counter()

    kimera_flow_re = re.compile(
        r"Kimera CBS incoming flow: pending=(?P<pending>\d+) "
        r"resolved=(?P<resolved>\d+) selected=(?P<selected>\d+) "
        r"bpsam_added=(?P<bpsam_added>\d+) rejected=(?P<rejected>\d+) "
        r"rejected_by\(window=(?P<window>\d+),timestamp=(?P<timestamp>\d+),"
        r"state=(?P<state>\d+),covariance=(?P<covariance>\d+),"
        r"bpsam=(?P<bpsam>\d+),bpsam_first_message=(?P<bpsam_first_message>\d+),"
        r"bpsam_update_status=(?P<bpsam_update_status>\d+),"
        r"bpsam_inactive_window=(?P<bpsam_inactive_window>\d+),"
        r"bpsam_shape=(?P<bpsam_shape>\d+),"
        r"bpsam_exception=(?P<bpsam_exception>\d+)"
    )
    alignment_re = re.compile(
        r"Kimera Rerun world alignment initialized .* dt=(?P<dt_ms>[-+0-9.eE]+) ms"
    )

    for path in log_paths:
        if not path.exists():
            continue
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for raw_line in stream:
                line = strip_ansi(raw_line)

                flow_match = kimera_flow_re.search(line)
                if flow_match:
                    kimera_flow_rows.append(
                        {key: int(value) for key, value in flow_match.groupdict().items()}
                    )

                alignment_match = alignment_re.search(line)
                if alignment_match:
                    alignment_rows.append({"dt_ms": to_float(alignment_match.group("dt_ms"))})

                for marker in ROW_MARKERS:
                    if marker not in line:
                        continue
                    for row in parse_csv_rows_from_line(line, marker):
                        if not row or row[0] != marker:
                            skipped_rows[f"{marker}:malformed_marker"] += 1
                            continue
                        if marker.startswith("CBS_TRANSPORT_ROW") and (
                            len(row) < 17 or row[16] != "ok"
                        ):
                            skipped_rows[f"{marker}:malformed_transport"] += 1
                            continue
                        if marker.startswith("CBS_ROUNDTRIP_ROW") and (
                            len(row) < 6 or row[5] != "ok"
                        ):
                            skipped_rows[f"{marker}:malformed_roundtrip"] += 1
                            continue
                        if marker.startswith("CBS_BPSAM_ADD_ROW") and (
                            len(row) < 14 or row[3] not in VALID_BPSAM_STATUSES
                        ):
                            skipped_rows[f"{marker}:malformed_bpsam"] += 1
                            continue
                        if marker == "CBS_OUTGOING_FILTER_ROW" and (
                            len(row) < 14 or row[13] not in OUTGOING_FILTER_STATUSES
                        ):
                            skipped_rows[f"{marker}:malformed_outgoing_filter"] += 1
                            continue
                        if marker == "CBS_RECEIVER_DIAGNOSTIC_ROW" and (
                            len(row) < 25 or row[5] not in VALID_BPSAM_STATUSES
                        ):
                            skipped_rows[f"{marker}:malformed_receiver_diagnostic"] += 1
                            continue
                        if marker == "CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW" and (
                            len(row) < 18 or row[5] not in TEMPORARY_LINEAR_ACCOUNTING_ACTIONS
                        ):
                            skipped_rows[f"{marker}:malformed_temporary_linear_accounting"] += 1
                            continue
                        if marker == "CBS_PREINJECTION_RESIDUAL_ROW" and (
                            len(row) < 22 or row[5] not in PREINJECTION_RESIDUAL_ACTIONS
                        ):
                            skipped_rows[f"{marker}:malformed_preinjection_residual"] += 1
                            continue
                        if marker == "CBS_TEMPORARY_LINEARIZATION_RESIDUAL_ROW" and (
                            len(row) < 23 or row[5] not in PREINJECTION_RESIDUAL_ACTIONS
                        ):
                            skipped_rows[
                                f"{marker}:malformed_temporary_linearization_residual"
                            ] += 1
                            continue
                        if marker == "CBS_MARGINALIZATION_GRAPH_ROW" and len(row) < 13:
                            skipped_rows[f"{marker}:malformed_marginalization_graph"] += 1
                            continue

                        if marker.startswith("CBS_TRANSPORT_ROW"):
                            direction = row[0].replace("CBS_TRANSPORT_ROW_", "")
                            sender_cov = parse_matrix_token(row[10])
                            received_cov = parse_matrix_token(row[11])
                            reconstructed_cov = parse_matrix_token(row[12])
                            transport_rows.append(
                                {
                                    "direction": direction,
                                    "key": row[1],
                                    "sender_timestamp_ns": row[2],
                                    "sender_frame": row[3],
                                    "receiver_expected_frame": row[4],
                                    "transform_label": row[5],
                                    "sender_mu": row[6],
                                    "received_mu": row[7],
                                    "reconstructed_mu": row[8],
                                    "mean_error_norm": to_float(row[9]),
                                    "sender_cov_trace": matrix_trace(sender_cov),
                                    "sender_cov_frobenius": matrix_frobenius(sender_cov),
                                    "sender_cov_diag_min": matrix_diag_min(sender_cov),
                                    "sender_cov_diag_max": matrix_diag_max(sender_cov),
                                    "received_cov_trace": matrix_trace(received_cov),
                                    "received_cov_frobenius": matrix_frobenius(received_cov),
                                    "received_cov_diag_min": matrix_diag_min(received_cov),
                                    "received_cov_diag_max": matrix_diag_max(received_cov),
                                    "reconstructed_cov_trace": matrix_trace(reconstructed_cov),
                                    "cov_error_fro": to_float(row[13]),
                                    "cov_symmetry_error": to_float(row[14]),
                                    "received_cov_min_eigenvalue": to_float(row[15]),
                                    "status": row[16],
                                }
                            )
                        elif marker.startswith("CBS_ROUNDTRIP_ROW"):
                            roundtrip_rows.append(
                                {
                                    "direction": row[0].replace("CBS_ROUNDTRIP_ROW_", ""),
                                    "key": row[1],
                                    "sender_timestamp_ns": row[2],
                                    "mean_roundtrip_error": to_float(row[3]),
                                    "cov_roundtrip_error": to_float(row[4]),
                                    "status": row[5],
                                }
                            )
                        elif marker.startswith("CBS_MERGE_ROW") and len(row) >= 24:
                            if row[15] not in MERGE_STATUSES:
                                skipped_rows[f"{marker}:malformed_merge"] += 1
                                continue
                            merge_rows.append(
                                {
                                    "direction": row[0].replace("CBS_MERGE_ROW_", ""),
                                    "sample_idx": row[1],
                                    "sender_frame": row[2],
                                    "sender_timestamp_ns": row[3],
                                    "sender_key": row[4],
                                    "receiver_key": row[5],
                                    "sent_trace": to_float(row[6]),
                                    "received_trace": to_float(row[7]),
                                    "receiver_local_before_trace": to_float(row[8]),
                                    "receiver_merged_trace": to_float(row[9]),
                                    "receiver_posterior_pre_trace": to_float(row[10]),
                                    "receiver_posterior_post_trace": to_float(row[11]),
                                    "hellinger_local_incoming": to_float(row[12]),
                                    "hellinger_local_merged": to_float(row[13]),
                                    "mahal_local_incoming": to_float(row[14]),
                                    "status": row[15],
                                    "receiver_local_source": row[16],
                                    "receiver_local_raw_trace": to_float(row[17]),
                                    "receiver_local_anchored_trace": to_float(row[18]),
                                    "dmu_local_incoming": to_float(row[19]),
                                    "dmu_local_merged": to_float(row[20]),
                                    "step": to_float(row[21]),
                                    "dxycurr": to_float(row[22]),
                                    "dxy_target": to_float(row[23]),
                                }
                            )
                        elif marker.startswith("CBS_MERGE_ROW"):
                            skipped_rows[f"{marker}:malformed_merge"] += 1
                        elif marker.startswith("CBS_BPSAM_ADD_ROW"):
                            bpsam_rows.append(
                                {
                                    "direction": row[0].replace("CBS_BPSAM_ADD_ROW_", ""),
                                    "sender_key": row[1],
                                    "receiver_key": row[2],
                                    "status": row[3],
                                    "enable_soft_reset": row[4],
                                    "metric_type": row[5],
                                    "d_reset": to_float(row[6]),
                                    "contract_alpha": to_float(row[7]),
                                    "existing_gbp_var": row[8],
                                    "is_first_message": row[9],
                                    "dxycurr": to_float(row[10]),
                                    "dxy": to_float(row[11]),
                                    "contraction_step_size": to_float(row[12]),
                                    "message": row[13],
                                }
                            )
                        elif marker == "CBS_OUTGOING_FILTER_ROW":
                            outgoing_filter_rows.append(
                                {
                                    "direction": row[1],
                                    "sender_robot": row[2],
                                    "receiver_robot": row[3],
                                    "producing_agent": row[4],
                                    "belief_key": row[5],
                                    "metric_type": row[6],
                                    "threshold": to_float(row[7]),
                                    "has_last": row[8],
                                    "metric_distance": to_float(row[9]),
                                    "last_trace": to_float(row[10]),
                                    "current_trace": to_float(row[11]),
                                    "dmu": to_float(row[12]),
                                    "status": row[13],
                                }
                            )
                        elif marker == "CBS_RECEIVER_DIAGNOSTIC_ROW":
                            if len(row) >= 40:
                                raw_gate_enabled = row[12]
                                raw_gate_checked = row[13]
                                raw_gate_rejected = row[14]
                                temporary_linear_gate_enabled = row[15]
                                temporary_linear_gate_checked = row[16]
                                accepted_but_skipped_already_applied = row[17]
                                repeated_impulse_prevented = row[18]
                                applied_incremental = row[19]
                                has_last_applied = row[20]
                                last_applied_metric_status = row[21]
                                last_applied_metric_distance = to_float(row[22])
                                last_applied_dmu = to_float(row[23])
                                last_applied_trace = to_float(row[24])
                                last_applied_cov_rel_frobenius = to_float(row[25])
                                has_gbp_peer = row[26]
                                gbp_peer_metric_status = row[27]
                                gbp_peer_metric_distance = to_float(row[28])
                                gbp_peer_dmu = to_float(row[29])
                                gbp_peer_trace = to_float(row[30])
                                incoming_trace = to_float(row[31])
                                d_reset = to_float(row[32])
                                enable_soft_reset = row[33]
                                gbp_update_soft_reset_effective = row[34]
                                contract_alpha = to_float(row[35])
                                is_first_message = row[36]
                                existing_gbp_var = row[37]
                                dxycurr = to_float(row[38])
                                contraction_step_size = to_float(row[39])
                            elif len(row) >= 29:
                                raw_gate_enabled = row[12]
                                raw_gate_checked = row[13]
                                raw_gate_rejected = row[14]
                                temporary_linear_gate_enabled = "false"
                                temporary_linear_gate_checked = "false"
                                accepted_but_skipped_already_applied = "false"
                                repeated_impulse_prevented = "false"
                                applied_incremental = "false"
                                has_last_applied = "false"
                                last_applied_metric_status = "no_previous"
                                last_applied_metric_distance = math.nan
                                last_applied_dmu = math.nan
                                last_applied_trace = math.nan
                                last_applied_cov_rel_frobenius = math.nan
                                has_gbp_peer = row[15]
                                gbp_peer_metric_status = row[16]
                                gbp_peer_metric_distance = to_float(row[17])
                                gbp_peer_dmu = to_float(row[18])
                                gbp_peer_trace = to_float(row[19])
                                incoming_trace = to_float(row[20])
                                d_reset = to_float(row[21])
                                enable_soft_reset = row[22]
                                gbp_update_soft_reset_effective = row[23]
                                contract_alpha = to_float(row[24])
                                is_first_message = row[25]
                                existing_gbp_var = row[26]
                                dxycurr = to_float(row[27])
                                contraction_step_size = to_float(row[28])
                            else:
                                raw_gate_enabled = "false"
                                raw_gate_checked = "false"
                                raw_gate_rejected = "false"
                                temporary_linear_gate_enabled = "false"
                                temporary_linear_gate_checked = "false"
                                accepted_but_skipped_already_applied = "false"
                                repeated_impulse_prevented = "false"
                                applied_incremental = "false"
                                has_last_applied = "false"
                                last_applied_metric_status = "no_previous"
                                last_applied_metric_distance = math.nan
                                last_applied_dmu = math.nan
                                last_applied_trace = math.nan
                                last_applied_cov_rel_frobenius = math.nan
                                has_gbp_peer = row[12]
                                gbp_peer_metric_status = row[13]
                                gbp_peer_metric_distance = to_float(row[14])
                                gbp_peer_dmu = to_float(row[15])
                                gbp_peer_trace = to_float(row[16])
                                incoming_trace = to_float(row[17])
                                d_reset = to_float(row[18])
                                enable_soft_reset = row[19]
                                gbp_update_soft_reset_effective = row[19]
                                contract_alpha = to_float(row[20])
                                is_first_message = row[21]
                                existing_gbp_var = row[22]
                                dxycurr = to_float(row[23])
                                contraction_step_size = to_float(row[24])
                            receiver_diagnostic_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "status": row[5],
                                    "metric_type": row[6],
                                    "has_raw_previous": row[7],
                                    "raw_previous_metric_status": row[8],
                                    "raw_previous_metric_distance": to_float(row[9]),
                                    "raw_previous_dmu": to_float(row[10]),
                                    "raw_previous_trace": to_float(row[11]),
                                    "raw_previous_gate_enabled": raw_gate_enabled,
                                    "raw_previous_gate_checked": raw_gate_checked,
                                    "raw_previous_gate_rejected": raw_gate_rejected,
                                    "temporary_linear_gate_enabled": temporary_linear_gate_enabled,
                                    "temporary_linear_gate_checked": temporary_linear_gate_checked,
                                    "accepted_but_skipped_already_applied": accepted_but_skipped_already_applied,
                                    "repeated_impulse_prevented": repeated_impulse_prevented,
                                    "applied_incremental": applied_incremental,
                                    "has_last_applied": has_last_applied,
                                    "last_applied_metric_status": last_applied_metric_status,
                                    "last_applied_metric_distance": last_applied_metric_distance,
                                    "last_applied_dmu": last_applied_dmu,
                                    "last_applied_trace": last_applied_trace,
                                    "last_applied_cov_rel_frobenius": last_applied_cov_rel_frobenius,
                                    "has_gbp_peer": has_gbp_peer,
                                    "gbp_peer_metric_status": gbp_peer_metric_status,
                                    "gbp_peer_metric_distance": gbp_peer_metric_distance,
                                    "gbp_peer_dmu": gbp_peer_dmu,
                                    "gbp_peer_trace": gbp_peer_trace,
                                    "incoming_trace": incoming_trace,
                                    "d_reset": d_reset,
                                    "enable_soft_reset": enable_soft_reset,
                                    "gbp_update_soft_reset_effective": gbp_update_soft_reset_effective,
                                    "contract_alpha": contract_alpha,
                                    "is_first_message": is_first_message,
                                    "existing_gbp_var": existing_gbp_var,
                                    "dxycurr": dxycurr,
                                    "contraction_step_size": contraction_step_size,
                                }
                            )
                        elif marker == "CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW":
                            temporary_linear_accounting_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "action": row[5],
                                    "repeated_impulse_prevented": row[6],
                                    "applied_incremental": row[7],
                                    "has_last_applied": row[8],
                                    "last_applied_metric_status": row[9],
                                    "last_applied_metric_distance": to_float(row[10]),
                                    "last_applied_dmu": to_float(row[11]),
                                    "last_applied_trace": to_float(row[12]),
                                    "incoming_trace": to_float(row[13]),
                                    "last_applied_cov_rel_frobenius": to_float(row[14]),
                                    "metric_threshold": to_float(row[15]),
                                    "dmu_threshold": to_float(row[16]),
                                    "cov_rel_threshold": to_float(row[17]),
                                }
                            )
                        elif marker == "CBS_PREINJECTION_RESIDUAL_ROW":
                            preinjection_residual_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "action": row[5],
                                    "receiver_pose_source": row[6],
                                    "residual_norm": to_float(row[7]),
                                    "rot_norm": to_float(row[8]),
                                    "trans_norm": to_float(row[9]),
                                    "dx": to_float(row[10]),
                                    "dy": to_float(row[11]),
                                    "dz": to_float(row[12]),
                                    "yaw_error_rad": to_float(row[13]),
                                    "yaw_error_deg": to_float(row[14]),
                                    "incoming_trace": to_float(row[15]),
                                    "receiver_x": to_float(row[16]),
                                    "receiver_y": to_float(row[17]),
                                    "receiver_z": to_float(row[18]),
                                    "incoming_x": to_float(row[19]),
                                    "incoming_y": to_float(row[20]),
                                    "incoming_z": to_float(row[21]),
                                }
                            )
                        elif marker == "CBS_TEMPORARY_LINEARIZATION_RESIDUAL_ROW":
                            temporary_linearization_residual_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "action": row[5],
                                    "linearization_source": row[6],
                                    "linearized_key": row[7],
                                    "residual_norm": to_float(row[8]),
                                    "rot_norm": to_float(row[9]),
                                    "trans_norm": to_float(row[10]),
                                    "dx": to_float(row[11]),
                                    "dy": to_float(row[12]),
                                    "dz": to_float(row[13]),
                                    "yaw_error_rad": to_float(row[14]),
                                    "yaw_error_deg": to_float(row[15]),
                                    "incoming_trace": to_float(row[16]),
                                    "receiver_x": to_float(row[17]),
                                    "receiver_y": to_float(row[18]),
                                    "receiver_z": to_float(row[19]),
                                    "incoming_x": to_float(row[20]),
                                    "incoming_y": to_float(row[21]),
                                    "incoming_z": to_float(row[22]),
                                }
                            )
                        elif marker == "CBS_KIMERA_OUTGOING_PROVENANCE_ROW" and len(row) >= 8:
                            provenance_rows.append(
                                {
                                    "key": row[1],
                                    "timestamp_ns": row[2],
                                    "source": row[3],
                                    "has_local_marginal": row[4],
                                    "has_bpsam": row[5],
                                    "is_external": row[6],
                                    "frame_semantic": row[7],
                                    "cov_semantic": row[8] if len(row) > 8 else "",
                                }
                            )
                        elif marker == "CBS_KIMERA_OUTGOING_PROVENANCE_ROW":
                            skipped_rows[f"{marker}:malformed_provenance"] += 1
                        elif marker == "CBS_MARGINALIZATION_GRAPH_ROW":
                            robot = row[1]
                            direction = (
                                "K2L"
                                if robot == "k"
                                else "L2K"
                                if robot == "l"
                                else f"{robot.upper()}2?"
                            )
                            marginalization_graph_rows.append(
                                {
                                    "direction": direction,
                                    "robot": robot,
                                    "type": row[2],
                                    "active_filter": row[3] == "true",
                                    "active_key_count": to_float(row[4]),
                                    "input_factor_slots": to_float(row[5]),
                                    "kept_factor_count": to_float(row[6]),
                                    "removed_total": to_float(row[7]),
                                    "removed_outside_active": to_float(row[8]),
                                    "removed_tracked_cbs_prior": to_float(row[9]),
                                    "removed_pose_belief": to_float(row[10]),
                                    "removed_anchor_belief": to_float(row[11]),
                                    "tmp_marginals": row[12] == "true",
                                }
                            )

    return {
        "transport": transport_rows,
        "roundtrip": roundtrip_rows,
        "merge": merge_rows,
        "bpsam_add": bpsam_rows,
        "outgoing_filter": outgoing_filter_rows,
        "receiver_diagnostic": receiver_diagnostic_rows,
        "temporary_linear_accounting": temporary_linear_accounting_rows,
        "preinjection_residual": preinjection_residual_rows,
        "temporary_linearization_residual": temporary_linearization_residual_rows,
        "provenance": provenance_rows,
        "marginalization_graph": marginalization_graph_rows,
        "kimera_flow": kimera_flow_rows,
        "alignment": alignment_rows,
        "skipped_rows": dict(skipped_rows),
    }


def write_dicts_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_ground_truth(path: Path) -> Trajectory:
    return [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in load_ground_truth_poses(path)]


def load_ground_truth_poses(path: Path) -> PoseTrajectory:
    points: Trajectory = []
    if not path.exists():
        return points
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            pose = {
                "t": timestamp_to_seconds(to_float(parts[0])),
                "x": to_float(parts[1]),
                "y": to_float(parts[2]),
                "z": to_float(parts[3]),
                "qx": to_float(parts[4]),
                "qy": to_float(parts[5]),
                "qz": to_float(parts[6]),
                "qw": to_float(parts[7]),
            }
            if all(math.isfinite(value) for value in pose.values()):
                points.append(normalize_pose_quaternion(pose))
    return sorted(points, key=lambda item: item["t"])


def first_existing_key(row: Dict[str, str], candidates: Sequence[str]) -> Optional[str]:
    for key in candidates:
        if key in row:
            return key
    for key in row:
        for candidate in candidates:
            if key.endswith(candidate):
                return key
    return None


def load_odometry_csv(path: Path) -> Trajectory:
    return [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in load_odometry_poses(path)]


def normalize_pose_quaternion(pose: PoseRow) -> PoseRow:
    norm = math.sqrt(
        pose["qx"] ** 2 + pose["qy"] ** 2 + pose["qz"] ** 2 + pose["qw"] ** 2
    )
    if not math.isfinite(norm) or norm <= 0.0:
        pose["qx"] = 0.0
        pose["qy"] = 0.0
        pose["qz"] = 0.0
        pose["qw"] = 1.0
        return pose
    pose["qx"] /= norm
    pose["qy"] /= norm
    pose["qz"] /= norm
    pose["qw"] /= norm
    return pose


def load_odometry_poses(path: Path) -> PoseTrajectory:
    if not path.exists() or path.stat().st_size == 0:
        return []
    poses: PoseTrajectory = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            if not row:
                continue
            x_key = first_existing_key(row, ("field.pose.pose.position.x", "pose.pose.position.x"))
            y_key = first_existing_key(row, ("field.pose.pose.position.y", "pose.pose.position.y"))
            z_key = first_existing_key(row, ("field.pose.pose.position.z", "pose.pose.position.z"))
            if not x_key or not y_key or not z_key:
                continue
            qx_key = first_existing_key(
                row, ("field.pose.pose.orientation.x", "pose.pose.orientation.x")
            )
            qy_key = first_existing_key(
                row, ("field.pose.pose.orientation.y", "pose.pose.orientation.y")
            )
            qz_key = first_existing_key(
                row, ("field.pose.pose.orientation.z", "pose.pose.orientation.z")
            )
            qw_key = first_existing_key(
                row, ("field.pose.pose.orientation.w", "pose.pose.orientation.w")
            )

            sec_key = first_existing_key(row, ("field.header.stamp.secs", "header.stamp.secs"))
            nsec_key = first_existing_key(row, ("field.header.stamp.nsecs", "header.stamp.nsecs"))
            stamp_key = first_existing_key(row, ("field.header.stamp", "header.stamp", "%time"))
            if sec_key and nsec_key:
                t = to_float(row[sec_key]) + to_float(row[nsec_key]) * 1e-9
            elif stamp_key:
                t = to_float(row[stamp_key])
            else:
                t = math.nan
            t = timestamp_to_seconds(t)

            pose = {
                "t": t,
                "x": to_float(row[x_key]),
                "y": to_float(row[y_key]),
                "z": to_float(row[z_key]),
                "qx": to_float(row[qx_key]) if qx_key else 0.0,
                "qy": to_float(row[qy_key]) if qy_key else 0.0,
                "qz": to_float(row[qz_key]) if qz_key else 0.0,
                "qw": to_float(row[qw_key]) if qw_key else 1.0,
            }
            if all(math.isfinite(value) for value in pose.values()):
                poses.append(normalize_pose_quaternion(pose))
    return sorted(poses, key=lambda item: item["t"])


def interpolate_trajectory(traj: Trajectory, query_t: float) -> Optional[Point]:
    if not traj:
        return None
    times = [item[0] for item in traj]
    if query_t < times[0] or query_t > times[-1]:
        return None
    idx = bisect.bisect_left(times, query_t)
    if idx < len(traj) and abs(times[idx] - query_t) < 1e-9:
        return traj[idx][1]
    if idx == 0 or idx >= len(traj):
        return None
    t0, p0 = traj[idx - 1]
    t1, p1 = traj[idx]
    if t1 <= t0:
        return p0
    alpha = (query_t - t0) / (t1 - t0)
    return (
        p0[0] + alpha * (p1[0] - p0[0]),
        p0[1] + alpha * (p1[1] - p0[1]),
        p0[2] + alpha * (p1[2] - p0[2]),
    )


def pair_trajectories(source: Trajectory, target: Trajectory) -> Tuple[List[Point], List[Point]]:
    src_points: List[Point] = []
    dst_points: List[Point] = []
    for t, point in source:
        target_point = interpolate_trajectory(target, t)
        if target_point is None:
            continue
        src_points.append(point)
        dst_points.append(target_point)
    return src_points, dst_points


def path_length(traj: Trajectory) -> float:
    if len(traj) < 2:
        return 0.0
    total = 0.0
    last = traj[0][1]
    for _, point in traj[1:]:
        total += distance(point, last)
        last = point
    return total


def distance(lhs: Point, rhs: Point) -> float:
    return math.sqrt(
        (lhs[0] - rhs[0]) ** 2 + (lhs[1] - rhs[1]) ** 2 + (lhs[2] - rhs[2]) ** 2
    )


def mean_point(points: Sequence[Point]) -> Point:
    n = float(len(points))
    return (
        sum(point[0] for point in points) / n,
        sum(point[1] for point in points) / n,
        sum(point[2] for point in points) / n,
    )


def align_se2(source: Sequence[Point], target: Sequence[Point]) -> Tuple[float, Point, List[Point]]:
    if len(source) != len(target) or len(source) < 2:
        return 0.0, (0.0, 0.0, 0.0), list(source)

    src_mean = mean_point(source)
    dst_mean = mean_point(target)
    a = 0.0
    b = 0.0
    for src, dst in zip(source, target):
        sx = src[0] - src_mean[0]
        sy = src[1] - src_mean[1]
        dx = dst[0] - dst_mean[0]
        dy = dst[1] - dst_mean[1]
        a += sx * dx + sy * dy
        b += sx * dy - sy * dx
    yaw = math.atan2(b, a)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    tx = dst_mean[0] - (cos_yaw * src_mean[0] - sin_yaw * src_mean[1])
    ty = dst_mean[1] - (sin_yaw * src_mean[0] + cos_yaw * src_mean[1])
    tz = dst_mean[2] - src_mean[2]

    aligned = [
        (
            cos_yaw * point[0] - sin_yaw * point[1] + tx,
            sin_yaw * point[0] + cos_yaw * point[1] + ty,
            point[2] + tz,
        )
        for point in source
    ]
    return yaw, (tx, ty, tz), aligned


def rmse(errors: Sequence[float]) -> float:
    if not errors:
        return math.nan
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def trajectory_metric_row(name: str, traj: Trajectory, reference: Trajectory) -> Dict[str, Any]:
    src, dst = pair_trajectories(traj, reference)
    yaw, translation, aligned = align_se2(src, dst)
    errors = [distance(lhs, rhs) for lhs, rhs in zip(aligned, dst)]
    return {
        "name": name,
        "samples": len(traj),
        "duration_sec": (traj[-1][0] - traj[0][0]) if len(traj) >= 2 else math.nan,
        "path_length_m": path_length(traj),
        "reference_matches": len(errors),
        "alignment": "SE2 yaw+translation",
        "alignment_yaw_deg": math.degrees(yaw),
        "alignment_tx_m": translation[0],
        "alignment_ty_m": translation[1],
        "alignment_tz_m": translation[2],
        "position_rmse_m": rmse(errors),
        "position_mean_error_m": numeric_stats(errors)["mean"],
        "position_max_error_m": numeric_stats(errors)["max"],
    }


def compute_trajectory_metrics(run_dir: Path, gt_path: Path) -> List[Dict[str, Any]]:
    gt = load_ground_truth(gt_path)
    kimera = load_odometry_csv(run_dir / "trajectories" / "kimera_odometry.csv")
    liorf = load_odometry_csv(run_dir / "trajectories" / "liorf_odometry.csv")

    rows: List[Dict[str, Any]] = []
    if gt:
        rows.append(trajectory_metric_row("kimera_vs_ground_truth", kimera, gt))
        rows.append(trajectory_metric_row("liorf_vs_ground_truth", liorf, gt))
    if kimera and liorf:
        rows.append(trajectory_metric_row("kimera_vs_liorf", kimera, liorf))
    return rows


def write_tum_poses(path: Path, poses: PoseTrajectory) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for pose in sorted(poses, key=lambda item: item["t"]):
            stream.write(
                f"{pose['t']:.9f} {pose['x']:.9f} {pose['y']:.9f} "
                f"{pose['z']:.9f} {pose['qx']:.9f} {pose['qy']:.9f} "
                f"{pose['qz']:.9f} {pose['qw']:.9f}\n"
            )


def nearest_pose_by_time(poses: PoseTrajectory, query_t: float) -> Optional[PoseRow]:
    if not poses:
        return None
    times = [pose["t"] for pose in poses]
    idx = bisect.bisect_left(times, query_t)
    if idx <= 0:
        return poses[0]
    if idx >= len(poses):
        return poses[-1]
    before = poses[idx - 1]
    after = poses[idx]
    return before if abs(before["t"] - query_t) <= abs(after["t"] - query_t) else after


def interpolate_ground_truth_for_estimator(
    gt_poses: PoseTrajectory,
    estimator_poses: PoseTrajectory,
) -> Tuple[PoseTrajectory, PoseTrajectory]:
    gt_points = [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in gt_poses]
    gt_eval: PoseTrajectory = []
    est_eval: PoseTrajectory = []
    for est_pose in estimator_poses:
        point = interpolate_trajectory(gt_points, est_pose["t"])
        nearest = nearest_pose_by_time(gt_poses, est_pose["t"])
        if point is None or nearest is None:
            continue
        gt_eval.append(
            {
                "t": est_pose["t"],
                "x": point[0],
                "y": point[1],
                "z": point[2],
                "qx": nearest["qx"],
                "qy": nearest["qy"],
                "qz": nearest["qz"],
                "qw": nearest["qw"],
            }
        )
        est_eval.append(est_pose)
    return gt_eval, est_eval


def export_tum_artifacts(run_dir: Path, gt_path: Path) -> Dict[str, Path]:
    tum_dir = run_dir / "trajectories" / "tum"
    gt_poses = load_ground_truth_poses(gt_path)
    kimera_poses = load_odometry_poses(run_dir / "trajectories" / "kimera_odometry.csv")
    liorf_poses = load_odometry_poses(run_dir / "trajectories" / "liorf_odometry.csv")

    paths = {
        "ground_truth": tum_dir / "ground_truth.tum",
        "kimera": tum_dir / "kimera.tum",
        "liorf": tum_dir / "liorf.tum",
        "ground_truth_kimera": tum_dir / "ground_truth_for_kimera.tum",
        "kimera_eval": tum_dir / "kimera_eval.tum",
        "ground_truth_liorf": tum_dir / "ground_truth_for_liorf.tum",
        "liorf_eval": tum_dir / "liorf_eval.tum",
    }

    write_tum_poses(paths["ground_truth"], gt_poses)
    write_tum_poses(paths["kimera"], kimera_poses)
    write_tum_poses(paths["liorf"], liorf_poses)

    gt_kimera, kimera_eval = interpolate_ground_truth_for_estimator(
        gt_poses, kimera_poses
    )
    gt_liorf, liorf_eval = interpolate_ground_truth_for_estimator(gt_poses, liorf_poses)
    write_tum_poses(paths["ground_truth_kimera"], gt_kimera)
    write_tum_poses(paths["kimera_eval"], kimera_eval)
    write_tum_poses(paths["ground_truth_liorf"], gt_liorf)
    write_tum_poses(paths["liorf_eval"], liorf_eval)
    return paths


def find_executable(name: str) -> Optional[str]:
    found = shutil.which(name)
    if found:
        return found
    pipx_candidate = Path.home() / ".local" / "bin" / name
    if pipx_candidate.exists():
        return str(pipx_candidate)
    return None


def parse_evo_stats(output: str) -> Dict[str, float]:
    stats: Dict[str, float] = {}
    stat_re = re.compile(r"^\s*(max|mean|median|min|rmse|sse|std)\s+([-+0-9.eE]+)\s*$")
    for line in strip_ansi(output).splitlines():
        match = stat_re.match(line)
        if match:
            stats[match.group(1)] = to_float(match.group(2))
    return stats


def run_evo_command(
    command: Sequence[str],
    output_path: Path,
    evo_home: Path,
) -> subprocess.CompletedProcess:
    evo_home.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["HOME"] = str(evo_home)
    env["MPLBACKEND"] = "Agg"
    result = run_command(command, env=env)
    output_path.write_text(
        "$ " + " ".join(shlex.quote(part) for part in command) + "\n\n" + result.stdout,
        encoding="utf-8",
    )
    return result


def run_evo_metrics(run_dir: Path, gt_path: Path) -> List[Dict[str, Any]]:
    evo_ape = find_executable("evo_ape")
    evo_rpe = find_executable("evo_rpe")
    if not evo_ape or not evo_rpe:
        return [
            {
                "estimator": "all",
                "metric": "evo",
                "status": "unavailable",
                "message": "evo_ape/evo_rpe not found",
            }
        ]

    tum_paths = export_tum_artifacts(run_dir, gt_path)
    evo_dir = run_dir / "parsed" / "evo"
    evo_dir.mkdir(parents=True, exist_ok=True)
    evo_home = run_dir / ".evo_home"
    rows: List[Dict[str, Any]] = []

    specs = [
        ("kimera", tum_paths["ground_truth_kimera"], tum_paths["kimera_eval"]),
        ("liorf", tum_paths["ground_truth_liorf"], tum_paths["liorf_eval"]),
    ]
    for estimator, reference, estimate in specs:
        if not reference.exists() or not estimate.exists() or estimate.stat().st_size == 0:
            rows.append(
                {
                    "estimator": estimator,
                    "metric": "evo",
                    "status": "skipped",
                    "message": "missing TUM trajectory data",
                }
            )
            continue

        commands = [
            (
                "ape",
                [
                    evo_ape,
                    "tum",
                    str(reference),
                    str(estimate),
                    "--align",
                    "--pose_relation",
                    "trans_part",
                    "--no_warnings",
                    "--save_results",
                    str(evo_dir / f"ape_{estimator}.zip"),
                ],
            ),
            (
                "rpe",
                [
                    evo_rpe,
                    "tum",
                    str(reference),
                    str(estimate),
                    "--align",
                    "--pose_relation",
                    "trans_part",
                    "--delta",
                    "1",
                    "--delta_unit",
                    "f",
                    "--no_warnings",
                    "--save_results",
                    str(evo_dir / f"rpe_{estimator}.zip"),
                ],
            ),
        ]
        for metric, command in commands:
            output_path = evo_dir / f"{metric}_{estimator}.txt"
            result = run_evo_command(command, output_path, evo_home)
            stats = parse_evo_stats(result.stdout)
            row: Dict[str, Any] = {
                "estimator": estimator,
                "metric": metric,
                "status": "ok" if result.returncode == 0 else "failed",
                "returncode": result.returncode,
                "pose_relation": "trans_part",
                "alignment": "Umeyama SE3 no scale",
                "delta": "1 frame" if metric == "rpe" else "",
                "log": str(output_path),
                "result_zip": str(evo_dir / f"{metric}_{estimator}.zip"),
            }
            for key in ("rmse", "mean", "median", "std", "min", "max", "sse"):
                row[key] = stats.get(key, math.nan)
            rows.append(row)
    return rows


def counter_dict(rows: Iterable[Dict[str, Any]], key: str) -> Dict[str, int]:
    return dict(Counter(str(row.get(key, "")) for row in rows if row.get(key, "") != ""))


def direction_counts(rows: Iterable[Dict[str, Any]]) -> Dict[str, int]:
    return dict(Counter(str(row.get("direction", "")) for row in rows if row.get("direction", "")))


def sum_kimera_flow(rows: List[Dict[str, int]]) -> Dict[str, int]:
    totals: Dict[str, int] = defaultdict(int)
    for row in rows:
        for key, value in row.items():
            totals[key] += value
    return dict(totals)


def covariance_summary(transport_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in transport_rows:
        by_direction[str(row["direction"])].append(row)
    for direction, values in sorted(by_direction.items()):
        rows.append(
            {
                "direction": direction,
                "count": len(values),
                "sent_trace_mean": numeric_stats(row["sender_cov_trace"] for row in values)["mean"],
                "sent_trace_min": numeric_stats(row["sender_cov_trace"] for row in values)["min"],
                "sent_trace_max": numeric_stats(row["sender_cov_trace"] for row in values)["max"],
                "received_trace_mean": numeric_stats(row["received_cov_trace"] for row in values)["mean"],
                "received_trace_min": numeric_stats(row["received_cov_trace"] for row in values)["min"],
                "received_trace_max": numeric_stats(row["received_cov_trace"] for row in values)["max"],
                "received_min_eigen_min": numeric_stats(
                    row["received_cov_min_eigenvalue"] for row in values
                )["min"],
                "cov_error_fro_max": numeric_stats(row["cov_error_fro"] for row in values)["max"],
                "cov_symmetry_error_max": numeric_stats(
                    row["cov_symmetry_error"] for row in values
                )["max"],
            }
        )
    return rows


def marginalization_graph_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_group: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_group[
            (
                str(row.get("direction", "")),
                str(row.get("robot", "")),
                str(row.get("type", "")),
            )
        ].append(row)
    for (direction, robot, graph_type), values in sorted(by_group.items()):
        rows.append(
            {
                "direction": direction,
                "robot": robot,
                "type": graph_type,
                "calls": len(values),
                "active_filter_calls": sum(1 for row in values if row.get("active_filter")),
                "tmp_marginals_calls": sum(1 for row in values if row.get("tmp_marginals")),
                "active_keys_mean": numeric_stats(
                    row.get("active_key_count") for row in values
                )["mean"],
                "input_factors_mean": numeric_stats(
                    row.get("input_factor_slots") for row in values
                )["mean"],
                "kept_factors_mean": numeric_stats(
                    row.get("kept_factor_count") for row in values
                )["mean"],
                "removed_total_mean": numeric_stats(
                    row.get("removed_total") for row in values
                )["mean"],
                "removed_outside_active_mean": numeric_stats(
                    row.get("removed_outside_active") for row in values
                )["mean"],
                "removed_tracked_cbs_prior_sum": numeric_stats(
                    row.get("removed_tracked_cbs_prior") for row in values
                )["sum"],
                "removed_pose_belief_sum": numeric_stats(
                    row.get("removed_pose_belief") for row in values
                )["sum"],
                "removed_anchor_belief_sum": numeric_stats(
                    row.get("removed_anchor_belief") for row in values
                )["sum"],
            }
        )
    return rows


def roundtrip_summary(roundtrip_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in roundtrip_rows:
        by_direction[str(row["direction"])].append(row)
    for direction, values in sorted(by_direction.items()):
        rows.append(
            {
                "direction": direction,
                "count": len(values),
                "mean_pose_roundtrip_error": numeric_stats(
                    row["mean_roundtrip_error"] for row in values
                )["mean"],
                "max_pose_roundtrip_error": numeric_stats(
                    row["mean_roundtrip_error"] for row in values
                )["max"],
                "mean_cov_roundtrip_error": numeric_stats(
                    row["cov_roundtrip_error"] for row in values
                )["mean"],
                "max_cov_roundtrip_error": numeric_stats(
                    row["cov_roundtrip_error"] for row in values
                )["max"],
            }
        )
    return rows


def merge_quality_summary(merge_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in merge_rows:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        hellinger_values = [row.get("hellinger_local_incoming", math.nan) for row in values]
        dmu_values = [row.get("dmu_local_incoming", math.nan) for row in values]
        mahal_values = [row.get("mahal_local_incoming", math.nan) for row in values]
        step_values = [row.get("step", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "hellinger_count": numeric_stats(hellinger_values)["count"],
                "hellinger_p50": percentile(hellinger_values, 0.50),
                "hellinger_p95": percentile(hellinger_values, 0.95),
                "hellinger_max": numeric_stats(hellinger_values)["max"],
                "dmu_p95": percentile(dmu_values, 0.95),
                "dmu_max": numeric_stats(dmu_values)["max"],
                "mahalanobis_p95": percentile(mahal_values, 0.95),
                "mahalanobis_max": numeric_stats(mahal_values)["max"],
                "step_mean": numeric_stats(step_values)["mean"],
            }
        )
    return rows


def outgoing_filter_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        metric_values = [row.get("metric_distance", math.nan) for row in values]
        dmu_values = [row.get("dmu", math.nan) for row in values]
        current_trace_values = [row.get("current_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "metric_count": numeric_stats(metric_values)["count"],
                "metric_p50": percentile(metric_values, 0.50),
                "metric_p95": percentile(metric_values, 0.95),
                "metric_max": numeric_stats(metric_values)["max"],
                "dmu_p95": percentile(dmu_values, 0.95),
                "dmu_max": numeric_stats(dmu_values)["max"],
                "current_trace_mean": numeric_stats(current_trace_values)["mean"],
                "threshold_mean": numeric_stats(row.get("threshold", math.nan) for row in values)[
                    "mean"
                ],
            }
        )
    return rows


def receiver_diagnostic_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        raw_values = [row.get("raw_previous_metric_distance", math.nan) for row in values]
        raw_dmu_values = [row.get("raw_previous_dmu", math.nan) for row in values]
        gbp_values = [row.get("gbp_peer_metric_distance", math.nan) for row in values]
        gbp_dmu_values = [row.get("gbp_peer_dmu", math.nan) for row in values]
        gaps = []
        for row in values:
            raw = row.get("raw_previous_metric_distance", math.nan)
            gbp = row.get("gbp_peer_metric_distance", math.nan)
            if math.isfinite(raw) and math.isfinite(gbp):
                gaps.append(gbp - raw)
        incoming_traces = [row.get("incoming_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "raw_gate_enabled_count": sum(
                    1 for row in values if str(row.get("raw_previous_gate_enabled", "")) == "true"
                ),
                "raw_gate_checked_count": sum(
                    1 for row in values if str(row.get("raw_previous_gate_checked", "")) == "true"
                ),
                "raw_gate_rejected_count": sum(
                    1 for row in values if str(row.get("raw_previous_gate_rejected", "")) == "true"
                ),
                "raw_metric_count": numeric_stats(raw_values)["count"],
                "raw_metric_p50": percentile(raw_values, 0.50),
                "raw_metric_p95": percentile(raw_values, 0.95),
                "raw_metric_max": numeric_stats(raw_values)["max"],
                "raw_dmu_p95": percentile(raw_dmu_values, 0.95),
                "gbp_metric_count": numeric_stats(gbp_values)["count"],
                "gbp_metric_p50": percentile(gbp_values, 0.50),
                "gbp_metric_p95": percentile(gbp_values, 0.95),
                "gbp_metric_max": numeric_stats(gbp_values)["max"],
                "gbp_dmu_p95": percentile(gbp_dmu_values, 0.95),
                "gbp_minus_raw_p50": percentile(gaps, 0.50),
                "gbp_minus_raw_p95": percentile(gaps, 0.95),
                "incoming_trace_mean": numeric_stats(incoming_traces)["mean"],
            }
        )
    return rows


def temporary_linear_accounting_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_action: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_action[
            (str(row.get("direction", "")), str(row.get("action", "")))
        ].append(row)

    for (direction, action), values in sorted(by_direction_action.items()):
        last_metric_values = [
            row.get("last_applied_metric_distance", math.nan) for row in values
        ]
        last_dmu_values = [row.get("last_applied_dmu", math.nan) for row in values]
        cov_rel_values = [
            row.get("last_applied_cov_rel_frobenius", math.nan) for row in values
        ]
        incoming_traces = [row.get("incoming_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "action": action,
                "count": len(values),
                "repeated_impulse_prevented_count": sum(
                    1 for row in values if str(row.get("repeated_impulse_prevented", "")) == "true"
                ),
                "applied_incremental_count": sum(
                    1 for row in values if str(row.get("applied_incremental", "")) == "true"
                ),
                "has_last_applied_count": sum(
                    1 for row in values if str(row.get("has_last_applied", "")) == "true"
                ),
                "last_H_p50": percentile(last_metric_values, 0.50),
                "last_H_p95": percentile(last_metric_values, 0.95),
                "last_dmu_p95": percentile(last_dmu_values, 0.95),
                "last_cov_rel_p95": percentile(cov_rel_values, 0.95),
                "incoming_trace_mean": numeric_stats(incoming_traces)["mean"],
            }
        )
    return rows


def preinjection_residual_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_action: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_action[
            (str(row.get("direction", "")), str(row.get("action", "")))
        ].append(row)

    for (direction, action), values in sorted(by_direction_action.items()):
        residuals = [row.get("residual_norm", math.nan) for row in values]
        rot_values = [row.get("rot_norm", math.nan) for row in values]
        trans_values = [row.get("trans_norm", math.nan) for row in values]
        yaw_values = [abs(row.get("yaw_error_deg", math.nan)) for row in values]
        trace_values = [row.get("incoming_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "action": action,
                "count": len(values),
                "residual_p50": percentile(residuals, 0.50),
                "residual_p95": percentile(residuals, 0.95),
                "residual_max": numeric_stats(residuals)["max"],
                "rot_p95": percentile(rot_values, 0.95),
                "trans_p50": percentile(trans_values, 0.50),
                "trans_p95": percentile(trans_values, 0.95),
                "trans_max": numeric_stats(trans_values)["max"],
                "abs_yaw_deg_p95": percentile(yaw_values, 0.95),
                "incoming_trace_mean": numeric_stats(trace_values)["mean"],
            }
        )
    return rows


def parse_bpsam_update_diags(message: str) -> Dict[str, Any]:
    values = [to_float(match.group(0)) for match in BPSAM_UPDATE_NUMBER_RE.finditer(message)]
    if len(values) < 36:
        return {}
    groups = [values[0:12], values[12:24], values[24:36]]
    before_diag = groups[0][6:12]
    incoming_diag = groups[1][6:12]
    after_diag = groups[2][6:12]
    return {
        "local_before_diag6": before_diag,
        "incoming_diag6": incoming_diag,
        "local_after_diag6": after_diag,
        "local_before_trace_from_diag": sum(before_diag),
        "incoming_trace_from_diag": sum(incoming_diag),
        "local_after_trace_from_diag": sum(after_diag),
    }


def format_diag6(values: Any) -> str:
    if not isinstance(values, list) or len(values) != 6:
        return "n/a"
    return "[" + ", ".join(format_float(value, precision=3) for value in values) + "]"


def diag_block_traces(values: Any) -> Tuple[float, float]:
    if not isinstance(values, list) or len(values) != 6:
        return math.nan, math.nan
    return sum(values[:3]), sum(values[3:])


def pop_matching_row(
    queues: Dict[Tuple[str, ...], List[Dict[str, Any]]],
    key: Tuple[str, ...],
) -> Optional[Dict[str, Any]]:
    rows = queues.get(key)
    if not rows:
        return None
    return rows.pop(0)


def finite_delta(after: Any, before: Any) -> float:
    try:
        after_value = float(after)
        before_value = float(before)
    except (TypeError, ValueError):
        return math.nan
    if not math.isfinite(after_value) or not math.isfinite(before_value):
        return math.nan
    return after_value - before_value


def injected_belief_covariance_samples(
    transport_rows: List[Dict[str, Any]],
    merge_rows: List[Dict[str, Any]],
    bpsam_rows: List[Dict[str, Any]],
    sample_limit: int = 40,
) -> List[Dict[str, Any]]:
    transport_by_direction_key: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in transport_rows:
        transport_by_direction_key[(str(row.get("direction", "")), str(row.get("key", "")))].append(row)

    accepted_merge_by_pair: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in merge_rows:
        if row.get("status") == "bpsam_added":
            accepted_merge_by_pair[
                (
                    str(row.get("direction", "")),
                    str(row.get("sender_key", "")),
                    str(row.get("receiver_key", "")),
                )
            ].append(row)

    counts: Counter[str] = Counter()
    samples: List[Dict[str, Any]] = []
    for row in bpsam_rows:
        direction = str(row.get("direction", ""))
        if row.get("status") != "accepted" or counts[direction] >= sample_limit:
            continue

        sender_key = str(row.get("sender_key", ""))
        receiver_key = str(row.get("receiver_key", ""))
        diag_info = parse_bpsam_update_diags(str(row.get("message", "")))
        if not diag_info:
            continue
        incoming_rot_trace, incoming_trans_trace = diag_block_traces(
            diag_info.get("incoming_diag6")
        )
        before_rot_trace, before_trans_trace = diag_block_traces(
            diag_info.get("local_before_diag6")
        )
        after_rot_trace, after_trans_trace = diag_block_traces(
            diag_info.get("local_after_diag6")
        )

        transport = pop_matching_row(transport_by_direction_key, (direction, sender_key))
        merge = pop_matching_row(accepted_merge_by_pair, (direction, sender_key, receiver_key))

        sent_trace = (
            merge.get("sent_trace")
            if merge
            else transport.get("sender_cov_trace")
            if transport
            else math.nan
        )
        received_trace = (
            merge.get("received_trace")
            if merge
            else transport.get("received_cov_trace")
            if transport
            else diag_info.get("incoming_trace_from_diag", math.nan)
        )
        local_before_trace = (
            merge.get("receiver_local_before_trace")
            if merge
            else diag_info.get("local_before_trace_from_diag", math.nan)
        )
        local_after_trace = (
            merge.get("receiver_merged_trace")
            if merge
            else diag_info.get("local_after_trace_from_diag", math.nan)
        )

        counts[direction] += 1
        samples.append(
            {
                "direction": direction,
                "sample": counts[direction],
                "sender_key": sender_key,
                "receiver_key": receiver_key,
                "sent_trace": sent_trace,
                "received_trace": received_trace,
                "local_before_trace": local_before_trace,
                "local_after_trace": local_after_trace,
                "local_trace_delta": finite_delta(local_after_trace, local_before_trace),
                "contraction_step_size": row.get("contraction_step_size", math.nan),
                "dxycurr": row.get("dxycurr", math.nan),
                "dxy": row.get("dxy", math.nan),
                "received_rot_trace": incoming_rot_trace,
                "received_trans_trace": incoming_trans_trace,
                "local_before_rot_trace": before_rot_trace,
                "local_before_trans_trace": before_trans_trace,
                "local_after_rot_trace": after_rot_trace,
                "local_after_trans_trace": after_trans_trace,
                "received_diag6": format_diag6(diag_info.get("incoming_diag6")),
                "local_before_diag6": format_diag6(diag_info.get("local_before_diag6")),
                "local_after_diag6": format_diag6(diag_info.get("local_after_diag6")),
                "covariance_source": "merge_row+factor_update"
                if merge
                else "transport_row+factor_update",
            }
        )

    return samples


def format_float(value: Any, precision: int = 4) -> str:
    if isinstance(value, (int, str)):
        return str(value)
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "n/a"
    if not math.isfinite(numeric):
        return "n/a"
    if abs(numeric) >= 1000 or (0 < abs(numeric) < 0.001):
        return f"{numeric:.3e}"
    return f"{numeric:.{precision}f}"


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    if not rows:
        return "_No rows._\n"
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_float(value) for value in row) + " |")
    return "\n".join(lines) + "\n"


def load_manifest(run_dir: Path) -> Dict[str, Any]:
    path = run_dir / "manifest.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def generate_report(run_dir: Path, gt_path: Optional[Path] = None) -> Dict[str, Any]:
    manifest = load_manifest(run_dir)
    if gt_path is None:
        gt_text = manifest.get("ground_truth")
        gt_path = Path(gt_text) if gt_text else Path()

    parsed = parse_log_artifacts([run_dir / "roslaunch.log"])
    trajectory_rows = compute_trajectory_metrics(run_dir, gt_path) if gt_path else []
    evo_rows = run_evo_metrics(run_dir, gt_path) if gt_path else []
    cov_rows = covariance_summary(parsed["transport"])
    roundtrip_rows = roundtrip_summary(parsed["roundtrip"])
    merge_quality_rows = merge_quality_summary(parsed["merge"])
    outgoing_filter_rows = outgoing_filter_summary(parsed["outgoing_filter"])
    receiver_diagnostic_rows = receiver_diagnostic_summary(parsed["receiver_diagnostic"])
    temporary_linear_accounting_rows = temporary_linear_accounting_summary(
        parsed["temporary_linear_accounting"]
    )
    preinjection_residual_rows = preinjection_residual_summary(
        parsed["preinjection_residual"]
    )
    temporary_linearization_residual_rows = preinjection_residual_summary(
        parsed["temporary_linearization_residual"]
    )
    marginalization_graph_rows = marginalization_graph_summary(
        parsed["marginalization_graph"]
    )
    injected_cov_rows = injected_belief_covariance_samples(
        parsed["transport"],
        parsed["merge"],
        parsed["bpsam_add"],
        sample_limit=40,
    )

    artifacts_dir = run_dir / "parsed"
    artifacts_dir.mkdir(exist_ok=True)
    write_dicts_csv(artifacts_dir / "cbs_transport.csv", parsed["transport"])
    write_dicts_csv(artifacts_dir / "cbs_roundtrip.csv", parsed["roundtrip"])
    write_dicts_csv(artifacts_dir / "cbs_merge.csv", parsed["merge"])
    write_dicts_csv(
        artifacts_dir / "cbs_merge_k2l.csv",
        [row for row in parsed["merge"] if row.get("direction") == "K2L"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_merge_l2k.csv",
        [row for row in parsed["merge"] if row.get("direction") == "L2K"],
    )
    write_dicts_csv(artifacts_dir / "cbs_bpsam_add.csv", parsed["bpsam_add"])
    write_dicts_csv(artifacts_dir / "cbs_outgoing_filter.csv", parsed["outgoing_filter"])
    write_dicts_csv(
        artifacts_dir / "cbs_receiver_diagnostics.csv",
        parsed["receiver_diagnostic"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_temporary_linear_accounting.csv",
        parsed["temporary_linear_accounting"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_preinjection_residuals.csv",
        parsed["preinjection_residual"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_temporary_linearization_residuals.csv",
        parsed["temporary_linearization_residual"],
    )
    write_dicts_csv(artifacts_dir / "cbs_kimera_provenance.csv", parsed["provenance"])
    write_dicts_csv(
        artifacts_dir / "cbs_marginalization_graph.csv",
        parsed["marginalization_graph"],
    )
    write_dicts_csv(artifacts_dir / "trajectory_metrics.csv", trajectory_rows)
    write_dicts_csv(artifacts_dir / "evo_metrics.csv", evo_rows)
    write_dicts_csv(artifacts_dir / "covariance_summary.csv", cov_rows)
    write_dicts_csv(artifacts_dir / "roundtrip_summary.csv", roundtrip_rows)
    write_dicts_csv(artifacts_dir / "merge_quality_summary.csv", merge_quality_rows)
    write_dicts_csv(artifacts_dir / "outgoing_filter_summary.csv", outgoing_filter_rows)
    write_dicts_csv(
        artifacts_dir / "receiver_diagnostic_summary.csv",
        receiver_diagnostic_rows,
    )
    write_dicts_csv(
        artifacts_dir / "temporary_linear_accounting_summary.csv",
        temporary_linear_accounting_rows,
    )
    write_dicts_csv(
        artifacts_dir / "preinjection_residual_summary.csv",
        preinjection_residual_rows,
    )
    write_dicts_csv(
        artifacts_dir / "temporary_linearization_residual_summary.csv",
        temporary_linearization_residual_rows,
    )
    write_dicts_csv(
        artifacts_dir / "marginalization_graph_summary.csv",
        marginalization_graph_rows,
    )
    write_dicts_csv(
        artifacts_dir / "injected_belief_covariance_samples.csv",
        injected_cov_rows,
    )

    summary: Dict[str, Any] = {
        "run_id": manifest.get("run_id", run_dir.name),
        "created_at": manifest.get("created_at"),
        "finished_at": manifest.get("finished_at"),
        "roslaunch_returncode": manifest.get("roslaunch_returncode"),
        "transport_counts": direction_counts(parsed["transport"]),
        "roundtrip_counts": direction_counts(parsed["roundtrip"]),
        "outgoing_filter_counts": direction_counts(parsed["outgoing_filter"]),
        "receiver_diagnostic_counts": direction_counts(parsed["receiver_diagnostic"]),
        "temporary_linear_accounting_counts": direction_counts(
            parsed["temporary_linear_accounting"]
        ),
        "preinjection_residual_counts": direction_counts(parsed["preinjection_residual"]),
        "temporary_linearization_residual_counts": direction_counts(
            parsed["temporary_linearization_residual"]
        ),
        "merge_status_by_direction": {
            direction: counter_dict(
                [row for row in parsed["merge"] if row.get("direction") == direction],
                "status",
            )
            for direction in sorted(direction_counts(parsed["merge"]))
        },
        "k2l_merge_status": counter_dict(
            [row for row in parsed["merge"] if row.get("direction") == "K2L"],
            "status",
        ),
        "l2k_merge_status": counter_dict(
            [row for row in parsed["merge"] if row.get("direction") == "L2K"],
            "status",
        ),
        "bpsam_add_status_by_direction": {
            direction: counter_dict(
                [row for row in parsed["bpsam_add"] if row.get("direction") == direction],
                "status",
            )
            for direction in sorted(direction_counts(parsed["bpsam_add"]))
        },
        "kimera_flow_totals": sum_kimera_flow(parsed["kimera_flow"]),
        "alignment": parsed["alignment"],
        "trajectory_metrics": trajectory_rows,
        "evo_metrics": evo_rows,
        "covariance_summary": cov_rows,
        "roundtrip_summary": roundtrip_rows,
        "merge_quality_summary": merge_quality_rows,
        "outgoing_filter_summary": outgoing_filter_rows,
        "receiver_diagnostic_summary": receiver_diagnostic_rows,
        "temporary_linear_accounting_summary": temporary_linear_accounting_rows,
        "preinjection_residual_summary": preinjection_residual_rows,
        "temporary_linearization_residual_summary": temporary_linearization_residual_rows,
        "marginalization_graph_summary": marginalization_graph_rows,
        "injected_belief_covariance_samples": injected_cov_rows,
        "skipped_log_rows": parsed["skipped_rows"],
    }
    write_json(run_dir / "summary.json", summary)
    (run_dir / "summary.md").write_text(render_markdown_report(run_dir, manifest, summary), encoding="utf-8")
    return summary


def render_markdown_report(run_dir: Path, manifest: Dict[str, Any], summary: Dict[str, Any]) -> str:
    launch_args = manifest.get("launch_args", {})
    git = manifest.get("git", {})
    lines: List[str] = []
    lines.append(f"# CBSMS Run Report: {summary.get('run_id', run_dir.name)}\n")
    lines.append("## Run\n")
    lines.append(f"- Run directory: `{run_dir}`")
    lines.append(f"- Created: `{summary.get('created_at')}`")
    lines.append(f"- Finished: `{summary.get('finished_at')}`")
    lines.append(f"- Roslaunch return code: `{summary.get('roslaunch_returncode')}`")
    lines.append(f"- Bag: `{launch_args.get('bag_path', 'n/a')}`")
    lines.append(f"- Duration: `{launch_args.get('bag_duration', 'n/a')} s`")
    lines.append(f"- Ground truth: `{manifest.get('ground_truth', 'n/a')}`")
    lines.append(f"- Rerun host: `{launch_args.get('rerun_host', 'n/a')}`\n")

    lines.append("## Git State\n")
    git_rows = []
    for name in CORE_REPOS:
        info = git.get(name, {})
        git_rows.append(
            [
                name,
                info.get("branch", "n/a"),
                info.get("commit_short", "n/a"),
                "dirty" if info.get("dirty") else "clean",
            ]
        )
    lines.append(markdown_table(["repo", "branch", "commit", "state"], git_rows))

    lines.append("## Trajectories\n")
    traj_rows = [
        [
            row["name"],
            row["samples"],
            row["reference_matches"],
            row["path_length_m"],
            row["position_rmse_m"],
            row["position_mean_error_m"],
            row["position_max_error_m"],
            row["alignment_yaw_deg"],
        ]
        for row in summary.get("trajectory_metrics", [])
    ]
    lines.append(
        markdown_table(
            ["metric", "samples", "matches", "path m", "rmse m", "mean m", "max m", "yaw deg"],
            traj_rows,
        )
    )

    lines.append("## Evo Metrics\n")
    evo_rows = [
        [
            row.get("estimator", ""),
            row.get("metric", ""),
            row.get("status", ""),
            row.get("rmse", math.nan),
            row.get("mean", math.nan),
            row.get("max", math.nan),
            row.get("alignment", ""),
            row.get("delta", ""),
        ]
        for row in summary.get("evo_metrics", [])
    ]
    lines.append(
        markdown_table(
            ["estimator", "metric", "status", "rmse m", "mean m", "max m", "alignment", "delta"],
            evo_rows,
        )
    )
    lines.append(
        "Evo uses TUM files in `trajectories/tum/`; ground truth is interpolated "
        "to each estimator timestamp before running translation APE/RPE.\n"
    )

    lines.append("## CBS Transport\n")
    transport = summary.get("transport_counts", {})
    roundtrip = summary.get("roundtrip_counts", {})
    lines.append(
        markdown_table(
            ["direction", "transport rows", "roundtrip rows"],
            [
                [direction, transport.get(direction, 0), roundtrip.get(direction, 0)]
                for direction in sorted(set(transport) | set(roundtrip))
            ],
        )
    )
    lines.append("- `L2K` means LiORF belief received by Kimera.")
    lines.append("- `K2L` means Kimera belief received by LiORF.\n")
    skipped = summary.get("skipped_log_rows", {})
    if skipped:
        lines.append("Malformed CBS log rows skipped during parsing:\n")
        lines.append(markdown_table(["reason", "count"], sorted(skipped.items())))

    outgoing_filter = summary.get("outgoing_filter_summary", [])
    if outgoing_filter:
        lines.append("## Sender-Side Belief Filter\n")
        lines.append(
            "This compares the current outgoing belief against the last belief "
            "that same sender returned for the same receiver/key. Rows marked "
            "`filtered_similar` were omitted before publishing because their "
            "metric distance was below the sender-side similarity threshold.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "metric n",
                    "metric p50",
                    "metric p95",
                    "metric max",
                    "dmu p95",
                    "dmu max",
                    "trace mean",
                    "threshold",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("metric_count", 0),
                        row.get("metric_p50", math.nan),
                        row.get("metric_p95", math.nan),
                        row.get("metric_max", math.nan),
                        row.get("dmu_p95", math.nan),
                        row.get("dmu_max", math.nan),
                        row.get("current_trace_mean", math.nan),
                        row.get("threshold_mean", math.nan),
                    ]
                    for row in outgoing_filter
                ],
            )
        )

    receiver_diag = summary.get("receiver_diagnostic_summary", [])
    if receiver_diag:
        lines.append("## Receiver Belief Diagnostics\n")
        lines.append(
            "`raw` compares the current incoming belief against the previous raw "
            "incoming belief from the same sender/key. `GBP` compares the current "
            "incoming belief against the receiver's existing GBP peer belief for "
            "that sender/key before the update. If raw H is low but GBP H is high, "
            "the sender sequence is smooth while the receiver's GBP copy is stale "
            "or diverged.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "raw gate checked",
                    "raw gate rejected",
                    "raw n",
                    "raw H p50",
                    "raw H p95",
                    "raw H max",
                    "raw dmu p95",
                    "GBP n",
                    "GBP H p50",
                    "GBP H p95",
                    "GBP H max",
                    "GBP dmu p95",
                    "GBP-raw p50",
                    "GBP-raw p95",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("raw_gate_checked_count", 0),
                        row.get("raw_gate_rejected_count", 0),
                        row.get("raw_metric_count", 0),
                        row.get("raw_metric_p50", math.nan),
                        row.get("raw_metric_p95", math.nan),
                        row.get("raw_metric_max", math.nan),
                        row.get("raw_dmu_p95", math.nan),
                        row.get("gbp_metric_count", 0),
                        row.get("gbp_metric_p50", math.nan),
                        row.get("gbp_metric_p95", math.nan),
                        row.get("gbp_metric_max", math.nan),
                        row.get("gbp_dmu_p95", math.nan),
                        row.get("gbp_minus_raw_p50", math.nan),
                        row.get("gbp_minus_raw_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in receiver_diag
                ],
            )
        )

    temporary_linear = summary.get("temporary_linear_accounting_summary", [])
    if temporary_linear:
        lines.append("## Temporary Linear Accounting\n")
        lines.append(
            "Temporary-linear CBS priors are non-persistent. This table reports "
            "which accepted beliefs were actually applied as temporary linear "
            "updates and which near-identical already-applied beliefs were "
            "skipped to prevent repeated impulses.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "repeated prevented",
                    "incremental",
                    "has last",
                    "last H p50",
                    "last H p95",
                    "last dmu p95",
                    "last cov rel p95",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("repeated_impulse_prevented_count", 0),
                        row.get("applied_incremental_count", 0),
                        row.get("has_last_applied_count", 0),
                        row.get("last_H_p50", math.nan),
                        row.get("last_H_p95", math.nan),
                        row.get("last_dmu_p95", math.nan),
                        row.get("last_cov_rel_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in temporary_linear
                ],
            )
        )

    preinj = summary.get("preinjection_residual_summary", [])
    if preinj:
        lines.append("## Pre-Injection Residuals\n")
        lines.append(
            "These rows compare the receiver's current local pose estimate against "
            "the incoming CBS prior pose immediately before the accepted prior is "
            "inserted into the temporary linear solve or prior-factor path. The "
            "translation values are in meters; yaw is reported as absolute degrees.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "res p50",
                    "res p95",
                    "res max",
                    "rot p95",
                    "trans p50",
                    "trans p95",
                    "trans max",
                    "|yaw| p95 deg",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("residual_p50", math.nan),
                        row.get("residual_p95", math.nan),
                        row.get("residual_max", math.nan),
                        row.get("rot_p95", math.nan),
                        row.get("trans_p50", math.nan),
                        row.get("trans_p95", math.nan),
                        row.get("trans_max", math.nan),
                        row.get("abs_yaw_deg_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in preinj
                ],
            )
        )

    temp_lin_res = summary.get("temporary_linearization_residual_summary", [])
    if temp_lin_res:
        lines.append("## Temporary Linearization Residuals\n")
        lines.append(
            "These rows are logged inside GTSAM at the exact point where "
            "`temporaryFactorsForDelta` are linearized against `theta_`. They "
            "are the residuals seen by the temporary CBS prior in the linear "
            "delta solve, after the regular local iSAM2 update/relinearization "
            "for that cycle.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "res p50",
                    "res p95",
                    "res max",
                    "rot p95",
                    "trans p50",
                    "trans p95",
                    "trans max",
                    "|yaw| p95 deg",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("residual_p50", math.nan),
                        row.get("residual_p95", math.nan),
                        row.get("residual_max", math.nan),
                        row.get("rot_p95", math.nan),
                        row.get("trans_p50", math.nan),
                        row.get("trans_p95", math.nan),
                        row.get("trans_max", math.nan),
                        row.get("abs_yaw_deg_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in temp_lin_res
                ],
            )
        )

    lines.append("## Acceptance Status\n")
    merge_status = summary.get("merge_status_by_direction", {})
    direction_titles = {
        "L2K": "Kimera receiving LiORF beliefs (`L2K`)",
        "K2L": "LiORF receiving Kimera beliefs (`K2L`)",
    }
    for direction in ("L2K", "K2L"):
        counts = merge_status.get(direction, {})
        if not counts:
            continue
        lines.append(f"### {direction_titles[direction]}\n")
        lines.append(markdown_table(["status", "count"], sorted(counts.items())))

    lines.append("### BPSAM add rows\n")
    bpsam_rows = []
    for direction, counts in summary.get("bpsam_add_status_by_direction", {}).items():
        for status, count in sorted(counts.items()):
            bpsam_rows.append([direction, status, count])
    lines.append(markdown_table(["direction", "status", "count"], bpsam_rows))

    kimera_flow = summary.get("kimera_flow_totals", {})
    if kimera_flow:
        lines.append("### Kimera incoming flow totals\n")
        lines.append(markdown_table(["counter", "sum"], sorted(kimera_flow.items())))

    merge_quality = summary.get("merge_quality_summary", [])
    if merge_quality:
        lines.append("## Merge Quality\n")
        lines.append(
            "Hellinger, Mahalanobis, and `dmu` compare the receiver's existing "
            "GBP belief for that peer/key against the incoming belief before "
            "the BPSAM update. Empty values mean there was no previous peer "
            "belief for that key or the belief was dropped before the gate.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "H n",
                    "H p50",
                    "H p95",
                    "H max",
                    "dmu p95",
                    "dmu max",
                    "mahal p95",
                    "mahal max",
                    "step mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("hellinger_count", 0),
                        row.get("hellinger_p50", math.nan),
                        row.get("hellinger_p95", math.nan),
                        row.get("hellinger_max", math.nan),
                        row.get("dmu_p95", math.nan),
                        row.get("dmu_max", math.nan),
                        row.get("mahalanobis_p95", math.nan),
                        row.get("mahalanobis_max", math.nan),
                        row.get("step_mean", math.nan),
                    ]
                    for row in merge_quality
                ],
            )
        )

    marg_graph = summary.get("marginalization_graph_summary", [])
    if marg_graph:
        lines.append("## Outgoing Covariance Graph\n")
        lines.append(
            "These rows are emitted when a sender builds the marginal graph used "
            "for outgoing belief covariance. `LOCAL` with `tmp calls == calls` "
            "means `getBeliefs()` used temporary marginals instead of the full "
            "persistent iSAM2 graph. Removed CBS/pose/anchor columns show "
            "external belief factors excluded from that covariance graph.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "robot",
                    "type",
                    "calls",
                    "active calls",
                    "tmp calls",
                    "active keys mean",
                    "input factors mean",
                    "kept factors mean",
                    "removed outside mean",
                    "removed CBS prior sum",
                    "removed pose belief sum",
                    "removed anchor sum",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("robot", ""),
                        row.get("type", ""),
                        row.get("calls", 0),
                        row.get("active_filter_calls", 0),
                        row.get("tmp_marginals_calls", 0),
                        row.get("active_keys_mean", math.nan),
                        row.get("input_factors_mean", math.nan),
                        row.get("kept_factors_mean", math.nan),
                        row.get("removed_outside_active_mean", math.nan),
                        row.get("removed_tracked_cbs_prior_sum", math.nan),
                        row.get("removed_pose_belief_sum", math.nan),
                        row.get("removed_anchor_belief_sum", math.nan),
                    ]
                    for row in marg_graph
                ],
            )
        )

    lines.append("## Covariance\n")
    cov_rows = [
        [
            row["direction"],
            row["count"],
            row["sent_trace_mean"],
            row["received_trace_mean"],
            row["received_min_eigen_min"],
            row["cov_error_fro_max"],
            row["cov_symmetry_error_max"],
        ]
        for row in summary.get("covariance_summary", [])
    ]
    lines.append(
        markdown_table(
            [
                "direction",
                "count",
                "sent trace mean",
                "received trace mean",
                "min eig min",
                "cov err max",
                "sym err max",
            ],
            cov_rows,
        )
    )
    injected_cov = summary.get("injected_belief_covariance_samples", [])
    if injected_cov:
        lines.append("## Injected Belief Covariance Samples\n")
        lines.append(
            "First 40 accepted BPSAM updates per direction. Covariances are "
            "reported as 6D Pose3 tangent-space summaries. `trace` is the sum "
            "of the 6x6 covariance diagonal; `rot tr` sums diagonal entries "
            "0-2 and `trans tr` sums entries 3-5. "
            "`sent` is the sender-side belief covariance, `received` is after "
            "receiver-frame conversion, and `local before/after` is the "
            "receiver GBP belief for the peer/key before and after the BPSAM "
            "update. Full diagonal vectors are saved in "
            "`parsed/injected_belief_covariance_samples.csv`.\n"
        )
        for direction in ("L2K", "K2L"):
            rows = [row for row in injected_cov if row.get("direction") == direction]
            if not rows:
                continue
            lines.append(f"### {direction}\n")
            lines.append(
                markdown_table(
                    [
                        "#",
                        "belief",
                        "sent tr",
                        "recv tr",
                        "before tr",
                        "after tr",
                        "delta tr",
                        "recv rot/trans tr",
                        "before rot/trans tr",
                        "after rot/trans tr",
                        "GBP step",
                    ],
                    [
                        [
                            row.get("sample", ""),
                            f"{row.get('sender_key', '')} -> {row.get('receiver_key', '')}",
                            row.get("sent_trace", math.nan),
                            row.get("received_trace", math.nan),
                            row.get("local_before_trace", math.nan),
                            row.get("local_after_trace", math.nan),
                            row.get("local_trace_delta", math.nan),
                            (
                                f"{format_float(row.get('received_rot_trace', math.nan))} / "
                                f"{format_float(row.get('received_trans_trace', math.nan))}"
                            ),
                            (
                                f"{format_float(row.get('local_before_rot_trace', math.nan))} / "
                                f"{format_float(row.get('local_before_trans_trace', math.nan))}"
                            ),
                            (
                                f"{format_float(row.get('local_after_rot_trace', math.nan))} / "
                                f"{format_float(row.get('local_after_trans_trace', math.nan))}"
                            ),
                            row.get("contraction_step_size", math.nan),
                        ]
                        for row in rows
                    ],
                )
            )

    lines.append("## Roundtrip Checks\n")
    rt_rows = [
        [
            row["direction"],
            row["count"],
            row["mean_pose_roundtrip_error"],
            row["max_pose_roundtrip_error"],
            row["mean_cov_roundtrip_error"],
            row["max_cov_roundtrip_error"],
        ]
        for row in summary.get("roundtrip_summary", [])
    ]
    lines.append(
        markdown_table(
            ["direction", "count", "pose mean", "pose max", "cov mean", "cov max"],
            rt_rows,
        )
    )

    alignment = summary.get("alignment", [])
    if alignment:
        lines.append("## Visualization Alignment\n")
        lines.append(
            markdown_table(
                ["event", "nearest pose dt ms"],
                [[idx, row.get("dt_ms", math.nan)] for idx, row in enumerate(alignment, start=1)],
            )
        )

    lines.append("## Artifacts\n")
    lines.append("- Raw log: `roslaunch.log`")
    lines.append("- Raw odometry CSVs: `trajectories/`")
    lines.append("- TUM trajectories for evo: `trajectories/tum/`")
    lines.append("- Parsed CBS CSVs: `parsed/`")
    lines.append("- Evo outputs: `parsed/evo/`")
    lines.append("- Machine summary: `summary.json`\n")
    return "\n".join(lines)


def cmd_report(args: argparse.Namespace) -> None:
    run_dir = args.run_dir.resolve()
    gt_path = args.gt_path.resolve() if args.gt_path else None
    generate_report(run_dir, gt_path)
    print(run_dir / "summary.md")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="launch the standard experiment and report it")
    run_parser.add_argument("--workspace", type=Path, default=workspace_root_from_script())
    run_parser.add_argument("--container", default=DEFAULT_CONTAINER)
    run_parser.add_argument("--output-root", type=Path, default=None)
    run_parser.add_argument("--name", default="")
    run_parser.add_argument("--bag-path", default=DEFAULT_BAG_PATH)
    run_parser.add_argument("--gt-path", type=Path, default=None)
    run_parser.add_argument("--duration", type=float, default=60.0)
    run_parser.add_argument("--timeout-padding", type=float, default=40.0)
    run_parser.add_argument("--launch-package", default="kimera_vio_ros")
    run_parser.add_argument("--launch-file", default="s3e_alpha_liorf_kimera_experiment.launch")
    run_parser.add_argument("--rerun-host", default=DEFAULT_RERUN_HOST)
    run_parser.add_argument("--extra-arg", action="append", default=[])
    run_parser.add_argument("--clean-start", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--record-trajectories", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--enable-cbs-bridge", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--use-kimera-rviz", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--use-liorf-rviz", action=argparse.BooleanOptionalAction, default=False)
    run_parser.add_argument("--kimera-visualize", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--rerun-visualizer-enable", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--rerun-world-alignment-enable", action=argparse.BooleanOptionalAction, default=True)

    report_parser = subparsers.add_parser("report", help="regenerate a report from an existing run dir")
    report_parser.add_argument("run_dir", type=Path)
    report_parser.add_argument("--gt-path", type=Path, default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "run":
            run_dir = run_experiment(args)
            print(run_dir / "summary.md")
        elif args.command == "report":
            cmd_report(args)
        else:
            parser.error(f"unknown command: {args.command}")
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
