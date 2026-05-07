#!/usr/bin/env bash
#
# Created using OpenAI Codex.
# Reviewed and verified by Magnus Mortensen.
# Helper script for saving offboard SLAM session artifacts.
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

find_repo_root() {
  local dir="$1"
  while [[ "$dir" != "/" ]]; do
    if [[ -d "$dir/.git" && -d "$dir/server_ws/offboard_slam" ]]; then
      printf '%s\n' "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

REPO_ROOT=""
if REPO_ROOT="$(find_repo_root "$(pwd)")"; then
  :
elif REPO_ROOT="$(find_repo_root "${SCRIPT_DIR}")"; then
  :
else
  REPO_ROOT=""
fi

if [[ -n "${REPO_ROOT}" ]]; then
  MAP_DIR="${REPO_ROOT}/server_ws/offboard_slam/maps"
else
  MAP_DIR="${PACKAGE_DIR}/maps"
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <map_name> [slam_namespace]"
  echo "Example: $0 lab_run"
  echo "Example: $0 lab_run /slam_toolbox"
  exit 1
fi

MAP_NAME="$1"
SLAM_NAMESPACE="${2:-/slam_toolbox}"
SESSION_DIR="${MAP_DIR}/${MAP_NAME}"
MAP_BASENAME="${SESSION_DIR}/${MAP_NAME}"
POSEGRAPH_BASENAME="${SESSION_DIR}/${MAP_NAME}"

mkdir -p "${SESSION_DIR}"

if [[ -n "${REPO_ROOT}" ]]; then
  echo "Saving SLAM artifacts into source repo maps directory:"
  echo "  ${SESSION_DIR}"
else
  echo "Source repo root not found, saving into installed package directory:"
  echo "  ${SESSION_DIR}"
fi

if [[ "${SLAM_NAMESPACE}" == "/" ]]; then
  SERIALIZE_SERVICE="/serialize_map"
else
  SERIALIZE_SERVICE="${SLAM_NAMESPACE%/}/serialize_map"
fi

echo "Saving occupancy map to ${MAP_BASENAME}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${MAP_BASENAME}"

echo "Serializing slam pose graph to ${POSEGRAPH_BASENAME}.posegraph/.data"
ros2 service call "${SERIALIZE_SERVICE}" slam_toolbox/srv/SerializePoseGraph "{filename: '${POSEGRAPH_BASENAME}'}"

echo "Saved SLAM session artifacts:"
echo "  ${MAP_BASENAME}.yaml"
echo "  ${MAP_BASENAME}.pgm"
echo "  ${POSEGRAPH_BASENAME}.posegraph"
echo "  ${POSEGRAPH_BASENAME}.data"

ln -sfn "${MAP_NAME}.yaml" "${SESSION_DIR}/current.yaml"
ln -sfn "${MAP_NAME}.pgm" "${SESSION_DIR}/current.pgm"
ln -sfn "${MAP_NAME}.posegraph" "${SESSION_DIR}/current.posegraph"
ln -sfn "${MAP_NAME}.data" "${SESSION_DIR}/current.data"
ln -sfn "${MAP_NAME}" "${MAP_DIR}/current"

echo "Updated repo-local current map aliases:"
echo "  ${SESSION_DIR}/current.yaml -> ${MAP_NAME}.yaml"
echo "  ${SESSION_DIR}/current.pgm -> ${MAP_NAME}.pgm"
echo "  ${SESSION_DIR}/current.posegraph -> ${MAP_NAME}.posegraph"
echo "  ${SESSION_DIR}/current.data -> ${MAP_NAME}.data"
echo "  ${MAP_DIR}/current -> ${MAP_NAME}/"
