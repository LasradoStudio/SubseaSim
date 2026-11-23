# ROS 2 Jazzy bridge + RViz2 for SubseaSim.
# Build context is the Bridge/ directory (see docker-compose.yml):
#   docker build -f docker/Dockerfile -t subsea-bridge .
# CI builds this image on every push; running the container has not been tried.
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy \
        ros-jazzy-rviz2 \
        ros-jazzy-tf2-ros \
        ros-jazzy-sensor-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# The package is pure Python and only needs numpy + ROS, so it is put on PYTHONPATH
# rather than pip-installed into Ubuntu's externally managed interpreter.
WORKDIR /opt/subsea
COPY subsea_bridge/ /opt/subsea/subsea_bridge/
COPY docker/rviz/ /opt/subsea/rviz/
ENV PYTHONPATH=/opt/subsea \
    PYTHONUNBUFFERED=1 \
    SUBSEA_HOST=host.docker.internal \
    SUBSEA_PORT=9870

# ros:jazzy's entrypoint (/ros_entrypoint.sh) sources /opt/ros/jazzy/setup.bash.
CMD ["python3", "-m", "subsea_bridge.ros2_bridge", "--ros-args", "-r", "__ns:=/subsea"]
