#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/ugv_ws/install/setup.bash
set -u
python3 /opt/robot/ros2/prepare_config.py
if [[ -f /run/ugv-nav/metric-profile.json ]]; then
  exec ros2 launch /opt/robot/ros2/launch/mapping.launch.py
fi
if [[ -f /run/ugv-nav/observation-profile.json ]]; then
  exec ros2 launch /opt/robot/ros2/launch/observation_mapping.launch.py
fi
exec ros2 launch /opt/robot/ros2/launch/sensors.launch.py
