# ROS 2 Jazzy bridge for the excavator simulator (phase2/ros_server.py runs on the host, outside Docker).
#   docker build -t construction-sim-ros2 ros2_bridge
#   docker run --rm -it --network host --ipc host construction-sim-ros2
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends python3-numpy && rm -rf /var/lib/apt/lists/*
COPY excavator_bridge.py /opt/excavator_bridge/excavator_bridge.py
CMD ["python3", "/opt/excavator_bridge/excavator_bridge.py"]
