# jetsondockerexperiments
Experiments with ROS2 / Docker on a Jetson Xavier

This repository contains scripts for building NVIDIA ISAAC environments
in Docker, with other common ROS2 packages.

Currently using ROS2 Foxy and Jetpack 4.7.0.

## The Scripts
- ``fix_tint.sh``: Fixes the pick tinge the occurs in some CSI Cameras (Leopard Imaging IMX219 and similar)
- ``./isaacDocker/build.sh``: Builds a ROS2 Foxy Docker container with NVIDIA ISAAC components
- ``docker_run_argus_mono.sh``: Runs the Docker container with a single CSI Camera, resize and rectification ROS2 nodes
- ``docker_run_gst.sh``: Runs the Docker container with dual CSI cameras, with slowed framerate and rectification. This uses Gstreamer, not ISAAC.

## Running

Place this repository in ``~/`` on a Jetson Xavier. Build the Docker container, then run either of the two examples.

This assumes that there is a IMX219 camera on each of the two CSI ports. If not, the ``.launch.py`` files will need to be modified.

Docker will allow the ROS2 nodes inside the containers to be exposed to the host network. So just run ROS2 (Foxy) ``rviz2`` or ``rqt`` to view the images on your laptop.

## References

https://forums.developer.nvidia.com/t/li-imx219-mipi-ff-nano-h136-pink-tint-problem/163533/11

https://github.com/rbonghi/isaac_ros_tutorial

https://github.com/NVIDIA-ISAAC-ROS/
