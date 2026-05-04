#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  run_s3e_report.sh [cbs-on|cbs-off] [duration_sec] [headless|rerun] [run_name]

Examples:
  # Standard reproducible CBS-on report, no live Rerun streaming.
  run_s3e_report.sh cbs-on 60

  # CBS-off baseline report.
  run_s3e_report.sh cbs-off 60

  # CBS-on report while streaming to an already-open Rerun viewer.
  run_s3e_report.sh cbs-on 60 rerun

Output:
  Prints the generated summary.md path. The run directory is created under:
  /home/yeranis/repos/V4RL/runs/
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runner="${script_dir}/cbsms_experiment.py"

mode="${1:-cbs-on}"
duration="${2:-60}"
visual_mode="${3:-headless}"
run_name="${4:-}"

case "${mode}" in
  cbs-on|on)
    cbs_arg="--enable-cbs-bridge"
    mode_slug="cbs-on"
    ;;
  cbs-off|off)
    cbs_arg="--no-enable-cbs-bridge"
    mode_slug="cbs-off"
    ;;
  *)
    echo "Unknown mode: ${mode}" >&2
    usage >&2
    exit 2
    ;;
esac

case "${visual_mode}" in
  headless|no-rerun)
    rerun_arg="--no-rerun-visualizer-enable"
    ;;
  rerun|live)
    rerun_arg="--rerun-visualizer-enable"
    ;;
  *)
    echo "Unknown visual mode: ${visual_mode}" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ -z "${run_name}" ]]; then
  run_name="s3e-alpha-${duration}s-${mode_slug}"
fi

exec "${runner}" run \
  --name "${run_name}" \
  --duration "${duration}" \
  "${cbs_arg}" \
  "${rerun_arg}"
