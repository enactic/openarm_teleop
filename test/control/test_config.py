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

# Configuration files of unilateral_control and bilateral_control.

from conftest import CAN_ERROR, CAN_INTERFACE, SOURCE_DIR

LEADER = str(SOURCE_DIR / "config" / "leader.yaml")
FOLLOWER = str(SOURCE_DIR / "config" / "follower.yaml")


def test_specified(run_teleop, write_urdf, tmp_path):
    urdf = write_urdf("openarm.urdf", "ee_base_link")
    # There is no config/*.yaml in the current directory.
    result = run_teleop(urdf, urdf, "right_arm", CAN_INTERFACE, CAN_INTERFACE,
                        LEADER, FOLLOWER, cwd=tmp_path)
    assert f"Leader config path: {LEADER}" in result.stdout
    assert f"Follower config path: {FOLLOWER}" in result.stdout
    assert CAN_ERROR in result.stdout


def test_specified_not_found(run_teleop, write_urdf, tmp_path):
    urdf = write_urdf("openarm.urdf", "ee_base_link")
    follower = str(tmp_path / "follower.yaml")
    result = run_teleop(urdf, urdf, "right_arm", CAN_INTERFACE, CAN_INTERFACE,
                        LEADER, follower)
    assert result.returncode == 1
    assert f"Failed to load YAML file: {follower}" in result.stdout
    assert CAN_ERROR not in result.stdout
