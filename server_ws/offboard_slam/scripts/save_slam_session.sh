#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_DIR="${PACKAGE_DIR}/maps"

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <map_name> [slam_namespace]"
  echo "Example: $0 lab_run"
  echo "Example: $0 lab_run /slam_toolbox"
  exit 1
fi

MAP_NAME="$1"
SLAM_NAMESPACE="${2:-/slam_toolbox}"
MAP_BASENAME="${MAP_DIR}/${MAP_NAME}"
POSEGRAPH_PATH="${MAP_DIR}/${MAP_NAME}.posegraph"

mkdir -p "${MAP_DIR}"

if [[ "${SLAM_NAMESPACE}" == "/" ]]; then
  SERIALIZE_SERVICE="/serialize_map"
else
  SERIALIZE_SERVICE="${SLAM_NAMESPACE%/}/serialize_map"
fi

echo "Saving occupancy map to ${MAP_BASENAME}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${MAP_BASENAME}"

echo "Serializing slam pose graph to ${POSEGRAPH_PATH}"
ros2 service call "${SERIALIZE_SERVICE}" slam_toolbox/srv/SerializePoseGraph "{filename: '${POSEGRAPH_PATH}'}"

echo "Saved SLAM session artifacts:"
echo "  ${MAP_BASENAME}.yaml"
echo "  ${MAP_BASENAME}.pgm"
echo "  ${POSEGRAPH_PATH}"
