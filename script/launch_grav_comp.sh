#!/bin/bash
#
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ======== Configuration ========
ARM_SIDE=${1:-right_arm}
CAN_IF=${2:-can0}
ARM_TYPE=${3:-v10}
TMPDIR="/tmp/openarm_urdf_gen"
URDF_NAME="${ARM_TYPE}_bimanual.urdf"
XACRO_FILE="${ARM_TYPE}.urdf.xacro"
WS_DIR=~/openarm_ros2_ws
XACRO_PATH="$WS_DIR/src/openarm_description/urdf/robot/$XACRO_FILE"
URDF_OUT="$TMPDIR/$URDF_NAME"
BIN_PATH=~/openarm_teleop/build/gravity_comp  # adjust if needed
# ===============================

# Check build binary
if [ ! -f "$BIN_PATH" ]; then
  echo "[ERROR] Compiled binary not found at: $BIN_PATH"
  exit 1
fi

# Generate URDF
echo "[INFO] Generating URDF using xacro..."
source $WS_DIR/install/setup.bash

mkdir -p "$TMPDIR"
xacro "$XACRO_PATH" bimanual:=true -o "$URDF_OUT"

if [ $? -ne 0 ]; then
    echo "[ERROR] Failed to generate URDF."
    exit 1
fi

# Run gravity compensation binary
echo "[INFO] Launching gravity compensation..."
"$BIN_PATH" "$ARM_SIDE" "$CAN_IF" "$URDF_OUT"

# Cleanup
echo "[INFO] Cleaning up tmp dir..."
rm -rf "$TMPDIR"
