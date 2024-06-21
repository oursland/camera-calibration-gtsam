FROM ros:humble-ros-base

SHELL [ "/bin/bash", "-c" ]

RUN apt update && \
    apt install -y --no-install-recommends \
        libboost-all-dev \
        libusb-1.0-0-dev \
        ros-humble-camera-calibration-parsers \
        ros-humble-camera-info-manager \
        ros-humble-gtsam \
        ros-humble-image-proc \
        ros-humble-image-transport \
    && \
    rm -rf /var/lib/apt/lists/*

COPY . /workspace
WORKDIR /workspace

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPython_EXECUTABLE=$(which python) \
            -DPython3_EXECUTABLE=$(which python)

WORKDIR /

CMD [ "/workspace/ros_entrypoint.sh", "ros2", "launch", "mocap_calibration", "intrinsic_calibration.launch.py" ]
