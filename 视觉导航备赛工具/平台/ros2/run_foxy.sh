#!/bin/bash
# Start a command in a clean ROS 2 environment, separate from ROS 1.
set -e
exec env -i HOME="$HOME" USER="$USER" LANG=C.UTF-8 \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}" ROS_LOCALHOST_ONLY=1 \
  bash --noprofile --norc -c 'source /opt/ros/foxy/setup.bash; exec "$@"' ros2-foxy "$@"
