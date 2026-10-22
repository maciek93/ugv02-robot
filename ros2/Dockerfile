FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential python3-pip python3-flask python3-requests python3-yaml \
    python3-colcon-common-extensions \
    libhidapi-hidraw0 ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup ros-jazzy-tf2-ros && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    adafruit-blinka==9.2.0 adafruit-circuitpython-busdevice==5.2.17 hidapi==0.15.0

RUN mkdir -p /opt/ugv_ws/src && \
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git /opt/ugv_ws/src/ldlidar_stl_ros2 && \
    git -C /opt/ugv_ws/src/ldlidar_stl_ros2 checkout bf668a89baf722a787dadc442860dcbf33a82f5a && \
    sed -i '1i#include <pthread.h>' /opt/ugv_ws/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp && \
    cd /opt/ugv_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install

RUN git clone https://github.com/mp-extras/vl53l5cx.git /opt/robot/vl53l5cx-driver && \
    git -C /opt/robot/vl53l5cx-driver checkout c7476877e96fabe81516176fbe8c575923d368b1

COPY ros2 /opt/robot/ros2
COPY supervised/metric_map.py /opt/robot/metric_map.py
COPY supervised/metric_mission.py /opt/robot/metric_mission.py

ENV PYTHONPATH=/opt/robot:/opt/robot/ros2:/opt/robot/vl53l5cx-driver
ENTRYPOINT ["/bin/bash", "/opt/robot/ros2/start.sh"]
