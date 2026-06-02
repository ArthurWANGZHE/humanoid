#!/usr/bin/env bash

# Source this file to clear ROS/colcon/Python overlay variables that commonly
# break Isaac Sim's embedded Python environment.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed."
  echo
  echo "Usage:"
  echo "  source /home/arthur/humanoid/scripts/isaac_rl/clear_isaac_python_env.sh"
  echo "  /home/arthur/isaacsim/python.sh <script> <args...>"
  exit 1
fi

_isaac_env_vars=(
  PYTHONPATH
  AMENT_PREFIX_PATH
  COLCON_PREFIX_PATH
  ROS_PACKAGE_PATH
  CMAKE_PREFIX_PATH
  ROS_DISTRO
)

echo "[isaac-env] cwd: $(pwd)"
echo "[isaac-env] clearing variables:"
for _var_name in "${_isaac_env_vars[@]}"; do
  echo "  - ${_var_name}"
  unset "${_var_name}"
done

export ISAACSIM_PYTHON="/home/arthur/isaacsim/python.sh"
echo "[isaac-env] ISAACSIM_PYTHON=${ISAACSIM_PYTHON}"
echo "[isaac-env] environment cleaned for manual Isaac runs"

unset _var_name
unset _isaac_env_vars
