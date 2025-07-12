#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="$(dirname "$SCRIPT_DIR")"

cd "$TARGET_DIR" || exit 1

git clone https://github.com/pal-robotics/pal_gazebo_worlds.git
git clone -b humble-devel https://github.com/pal-robotics/pal_navigation_cfg_public.git
git clone https://github.com/pal-robotics/tiago_robot.git
git clone https://github.com/pal-robotics/tiago_simulation.git

# Paths to destination files
TIAGO_LAUNCH="$TARGET_DIR/tiago_simulation/tiago_gazebo/launch/tiago_gazebo.launch.py"
PAL_WORLD="$TARGET_DIR/pal_gazebo_worlds/worlds/pal_office.world"

# Replace files
cp "$SCRIPT_DIR/replace_tiago_gazebo.launch.py" "$TIAGO_LAUNCH"
cp "$SCRIPT_DIR/replace_pal_office.world" "$PAL_WORLD"

echo "Repositories setup finished."