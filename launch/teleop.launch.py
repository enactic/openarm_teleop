# Copyright 2026 Enactic, Inc.
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
import os
import tempfile

import xacro
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# arm_type -> (folder under assets/robot, xacro file name)
# NOTE: currently only supports version 1.0
ARM_TYPE_MAP = {
    "v10": ("openarm_v1.0", "openarm_v10.urdf.xacro"),
    # "v20": ("openarm_v2.0", "openarm_v20.urdf.xacro"),
}

# control_type -> installed binary name
CONTROL_TYPE_MAP = {
    "unilateral": "unilateral_control",
    "bilateral": "bilateral_control",
}


def resolve_xacro_path(arm_type: str) -> str:
    if arm_type not in ARM_TYPE_MAP:
        raise RuntimeError(
            f"Invalid arm_type: '{arm_type}'. Valid values: {sorted(ARM_TYPE_MAP)}"
        )
    folder, file_name = ARM_TYPE_MAP[arm_type]
    desc_share = get_package_share_directory("openarm_description")
    return os.path.join(desc_share, "assets", "robot", folder, "urdf", file_name)


def launch_setup(context, *args, **kwargs):
    control_type = LaunchConfiguration("control_type").perform(context)
    arm_side = LaunchConfiguration("arm_side").perform(context)
    arm_type = LaunchConfiguration("arm_type").perform(context)

    if control_type not in CONTROL_TYPE_MAP:
        raise RuntimeError(
            f"Invalid control_type: '{control_type}'. "
            f"Valid values: {sorted(CONTROL_TYPE_MAP)}"
        )
    binary_name = CONTROL_TYPE_MAP[control_type]

    if arm_side not in ("right_arm", "left_arm", "both"):
        raise RuntimeError(
            f"Invalid arm_side: '{arm_side}'. Must be right_arm, left_arm, or both."
        )

    # Generate the URDF once. The URDF does not depend on arm_side (bimanual
    # generates both arms; arm_side only selects the leaf link at runtime), and
    # leader/follower share the same description, so a single file serves all
    # control processes.
    xacro_path = resolve_xacro_path(arm_type)
    urdf_xml = xacro.process_file(
        xacro_path, mappings={"bimanual": "true"}
    ).toprettyxml(indent="  ")

    out_dir = os.path.join(tempfile.gettempdir(), "openarm_urdf_gen")
    os.makedirs(out_dir, exist_ok=True)
    urdf_path = os.path.join(out_dir, f"openarm_{arm_type}.urdf")
    with open(urdf_path, "w") as f:
        f.write(urdf_xml)

    # Installed (not build/) binary. cwd is the share dir so the binary's
    # relative "config/leader.yaml" / "config/follower.yaml" paths resolve.
    bin_path = os.path.join(
        get_package_prefix("openarm_teleop"),
        "lib",
        "openarm_teleop",
        binary_name,
    )
    cwd = get_package_share_directory("openarm_teleop")

    arms = []
    if arm_side in ("right_arm", "both"):
        arms.append(
            (
                "right_arm",
                LaunchConfiguration("leader_right_can").perform(context),
                LaunchConfiguration("follower_right_can").perform(context),
            )
        )
    if arm_side in ("left_arm", "both"):
        arms.append(
            (
                "left_arm",
                LaunchConfiguration("leader_left_can").perform(context),
                LaunchConfiguration("follower_left_can").perform(context),
            )
        )

    return [
        ExecuteProcess(
            cmd=[bin_path, urdf_path, urdf_path, side, leader_can, follower_can],
            cwd=cwd,
            name=f"{binary_name}_{side}",
            output="screen",
        )
        for side, leader_can, follower_can in arms
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control_type",
                description="Control mode to launch: unilateral or bilateral.",
            ),
            DeclareLaunchArgument(
                "arm_side",
                default_value="both",
                description="Which arm(s) to control: right_arm, left_arm, or both.",
            ),
            DeclareLaunchArgument(
                "arm_type",
                default_value="v10",
                description="Arm type (hardware version).",
            ),
            DeclareLaunchArgument(
                "leader_right_can",
                default_value="can0",
                description="CAN interface for the right-arm leader.",
            ),
            DeclareLaunchArgument(
                "leader_left_can",
                default_value="can1",
                description="CAN interface for the left-arm leader.",
            ),
            DeclareLaunchArgument(
                "follower_right_can",
                default_value="can2",
                description="CAN interface for the right-arm follower.",
            ),
            DeclareLaunchArgument(
                "follower_left_can",
                default_value="can3",
                description="CAN interface for the left-arm follower.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
