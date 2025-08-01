#!/bin/bash

# Setup script for quadrotor simulation
# This script sets up the environment for proper material loading

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

# Add our package to Gazebo resource path
export GAZEBO_RESOURCE_PATH="$PACKAGE_DIR:$GAZEBO_RESOURCE_PATH"

echo "Gazebo resource path updated to include custom materials"
echo "GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"

# Source ROS workspace if not already sourced
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo "Warning: ROS environment not detected. Please source your ROS setup first."
    echo "Example: source /opt/ros/noetic/setup.bash"
fi

echo "Environment setup complete. You can now launch the simulation."