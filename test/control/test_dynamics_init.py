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

# The executables must stop before they touch any motor when they
# can't set up the dynamics from the given URDF.

import pytest

# A CAN interface that doesn't exist. An executable that passed the
# dynamics setup fails with CAN_ERROR instead of touching a real one.
CAN_INTERFACE = "openarm-none"
CAN_ERROR = "Failed to initialize socket for interface"


# Leaf links of OpenArm 2.0 and OpenArm 1.0
LEAVES = ["ee_base_link", "hand"]


def leaf_error(urdf, side):
    return (f"Failed to find end link: {urdf}: "
            f"openarm_{side}_ee_base_link, openarm_{side}_hand")


@pytest.mark.parametrize("leaf", LEAVES)
def test_gravity_comp(run_gravity_comp, write_urdf, leaf):
    urdf = write_urdf("openarm.urdf", leaf)
    result = run_gravity_comp("right_arm", CAN_INTERFACE, urdf)
    assert f"[Dynamics] End link: openarm_right_{leaf}" in result.stdout
    assert CAN_ERROR in result.stdout


def test_gravity_comp_no_leaf_link(run_gravity_comp, write_urdf):
    urdf = write_urdf("openarm.urdf", "tool")
    result = run_gravity_comp("right_arm", CAN_INTERFACE, urdf)
    assert result.returncode == 1
    assert leaf_error(urdf, "right") in result.stdout
    assert CAN_ERROR not in result.stdout


@pytest.mark.parametrize("leaf", LEAVES)
def test_teleop(run_teleop, write_urdf, leaf):
    urdf = write_urdf("openarm.urdf", leaf)
    result = run_teleop(urdf, urdf, "left_arm", CAN_INTERFACE, CAN_INTERFACE)
    assert (result.stdout.count(f"[Dynamics] End link: openarm_left_{leaf}")
            == 2)
    assert CAN_ERROR in result.stdout


def test_teleop_no_leader_leaf_link(run_teleop, write_urdf):
    leader_urdf = write_urdf("leader.urdf", "tool")
    follower_urdf = write_urdf("follower.urdf", "ee_base_link")
    result = run_teleop(leader_urdf, follower_urdf, "left_arm",
                        CAN_INTERFACE, CAN_INTERFACE)
    assert result.returncode == 1
    assert leaf_error(leader_urdf, "left") in result.stdout
    assert CAN_ERROR not in result.stdout


def test_teleop_no_follower_leaf_link(run_teleop, write_urdf):
    leader_urdf = write_urdf("leader.urdf", "ee_base_link")
    follower_urdf = write_urdf("follower.urdf", "tool")
    result = run_teleop(leader_urdf, follower_urdf, "left_arm",
                        CAN_INTERFACE, CAN_INTERFACE)
    assert result.returncode == 1
    assert leaf_error(follower_urdf, "left") in result.stdout
    assert CAN_ERROR not in result.stdout
