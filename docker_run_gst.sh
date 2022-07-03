#!/bin/bash



BASE_NAME="ros_isaac:latest"
CONTAINER_NAME="ros2_gst"

DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
DOCKER_ARGS+=("--pid=host")

echo "Running $CONTAINER_NAME"
docker run -it --rm \
    --privileged --network host \
    ${DOCKER_ARGS[@]} \
    -v /home/jetson/docker_map:/docker_map \
    -v /dev/shm:/dev/shm \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --entrypoint /ros_entrypoint.sh \
    --workdir /opt/isaac_ros_ws \
    $BASE_NAME \
    'ros2' 'launch' '/docker_map/gst.launch.py'
