#!/bin/bash

# Get the entry in the dpkg status file corresponding to the provied package name
# Prepend two newlines so it can be safely added to the end of any existing
# dpkg/status file.
get_dpkg_status() {
    echo -e "\n"
    awk '/Package: '"$1"'/,/^$/' /var/lib/dpkg/status
}

DPKG_STATUS=$(get_dpkg_status cuda-cudart-10-2)$(get_dpkg_status libcufft-10-2)
docker build -t ros_isaac --build-arg "DPKG_STATUS=$DPKG_STATUS" .
