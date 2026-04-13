#!/usr/bin/env bash
# Build the rclrs benchmark inside a ROS 2 Humble colcon workspace.
#
# Expects to be run inside `osrf/ros:humble-desktop` (or equivalent),
# with this repo mounted. Creates /tmp/ros2_ws, clones all the
# dependency repos that rclrs needs for Humble, symlinks our package
# into the workspace, and runs `colcon build`.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")"/../.. && pwd)"
PKG_DIR="$REPO_ROOT/ros2/rs_pubsub/ros2_benchmark_rs"
WS="${WS:-/ros2_ws}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# System prerequisites for rclrs.
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
    git libclang-dev python3-pip python3-vcstool curl build-essential \
    "ros-$ROS_DISTRO-example-interfaces" "ros-$ROS_DISTRO-test-msgs"

# Rust toolchain (if missing).
if ! command -v cargo >/dev/null 2>&1; then
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable --profile minimal
fi
. "$HOME/.cargo/env"

# colcon plugins for Rust.
pip install --break-system-packages colcon-cargo colcon-ros-cargo 2>/dev/null || \
    pip install colcon-cargo colcon-ros-cargo

# Workspace with rclrs deps (Jazzy).
mkdir -p "$WS/src"
cd "$WS/src"
[ -d common_interfaces ] || git clone -b "$ROS_DISTRO" https://github.com/ros2/common_interfaces.git
[ -d rcl_interfaces ]    || git clone -b "$ROS_DISTRO" https://github.com/ros2/rcl_interfaces.git
[ -d rosidl_core ]       || git clone -b "$ROS_DISTRO" https://github.com/ros2/rosidl_core.git
[ -d rosidl_defaults ]   || git clone -b "$ROS_DISTRO" https://github.com/ros2/rosidl_defaults.git
[ -d unique_identifier_msgs ] || git clone -b "$ROS_DISTRO" https://github.com/ros2/unique_identifier_msgs.git
[ -d test_interface_files ]   || git clone -b "$ROS_DISTRO" https://github.com/ros2/test_interface_files.git
[ -d rosidl_rust ]       || git clone https://github.com/ros2-rust/rosidl_rust.git
[ -d ros2_rust ]         || git clone https://github.com/ros2-rust/ros2_rust.git

# Copy (not symlink) our package into the workspace. A symlink would
# cause cargo's config.toml discovery to walk up from the canonical path
# (outside the workspace), so it would never see /ros2_ws/.cargo/config.toml
# and the crates.io -> local path patches for std_msgs etc. would be
# missed.
rm -rf "$WS/src/ros2_benchmark_rs"
cp -r "$PKG_DIR" "$WS/src/ros2_benchmark_rs"

cd "$WS"
set +u
. "/opt/ros/$ROS_DISTRO/setup.sh"
set -u
colcon build --packages-up-to ros2_benchmark_rs --packages-skip test_msgs
