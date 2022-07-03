#!/bin/bash

# Fix pink tinge in camera. Taken from
# https://forums.developer.nvidia.com/t/li-imx219-mipi-ff-nano-h136-pink-tint-problem/163533/10

sudo cp camera_overrides.isp to /var/nvidia/nvcam/settings
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
