#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-65}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

set +u
source /opt/ros/humble/setup.bash
for setup_file in \
  "$SCRIPT_DIR/../../install/setup.bash" \
  "$SCRIPT_DIR/../install/setup.bash" \
  "$SCRIPT_DIR/install/setup.bash"; do
  if [[ -f "$setup_file" ]]; then
    source "$setup_file"
    break
  fi
done
set -u

FILTER_PARAMS_FILE="${LIVOX_SELF_FILTER_CONFIG:-$SCRIPT_DIR/livox_self_filter_ros2/config/real_livox_self_filter.yaml}"
if [[ ! -f "$FILTER_PARAMS_FILE" ]]; then
  echo "Livox self-filter config file not found: $FILTER_PARAMS_FILE" >&2
  exit 1
fi

USE_RVIZ=false
LAUNCH_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --vis|--rviz)
      USE_RVIZ=true
      ;;
    *)
      LAUNCH_ARGS+=("$arg")
      ;;
  esac
done

echo "Starting lightweight Livox self-filter"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "  filter_params_file=$FILTER_PARAMS_FILE"
echo "  use_rviz=$USE_RVIZ"
echo "  publish_debug_clouds=$USE_RVIZ"

exec ros2 launch livox_self_filter_ros2 real_livox_self_filter.launch.py \
  filter_params_file:="$FILTER_PARAMS_FILE" \
  use_rviz:="$USE_RVIZ" \
  publish_debug_clouds:="$USE_RVIZ" \
  "${LAUNCH_ARGS[@]}"
