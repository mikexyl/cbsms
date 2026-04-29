#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker compose -f "${script_dir}/compose.yml" up -d cbsms
docker compose -f "${script_dir}/compose.yml" exec cbsms src/cbsms/docker/build_workspace.sh "$@"
