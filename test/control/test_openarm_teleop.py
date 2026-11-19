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

# The installed openarm-teleop command.

import signal

import pytest

from conftest import CAN_ERROR, CAN_INTERFACE, wait_until

@pytest.mark.parametrize("command", ["bilateral", "unilateral"])
def test_teleop(run_openarm_teleop, command):
    result = run_openarm_teleop(command,
                                "--arm-side", "left",
                                "--follower-can-interface", CAN_INTERFACE)
    assert result.returncode == 1
    # The installed URDF of OpenArm 2.0 is used by default.
    assert "/share/openarm-teleop/urdf/openarm-2.0.urdf" in result.stdout
    assert "Arm side         : left_arm" in result.stdout
    # The default of the left arm
    assert "Leader CAN       : can1" in result.stdout
    assert f"Follower CAN     : {CAN_INTERFACE}" in result.stdout
    # The installed configuration files are used.
    assert "/share/openarm-teleop/config/leader.yaml" in result.stdout
    assert "/share/openarm-teleop/config/follower.yaml" in result.stdout
    assert "[Dynamics] End link: openarm_left_ee_base_link" in result.stdout
    assert CAN_ERROR in result.stdout


def test_gravity_compensation(run_openarm_teleop):
    result = run_openarm_teleop("gravity-compensation",
                                "--openarm-version", "1.0",
                                "--can-interface", CAN_INTERFACE)
    assert result.returncode != 0
    assert "/share/openarm-teleop/urdf/openarm-1.0.urdf" in result.stdout
    assert "Arm side       : right_arm" in result.stdout
    assert "[Dynamics] End link: openarm_right_hand" in result.stdout
    assert f"{CAN_ERROR}: {CAN_INTERFACE}" in result.stdout


def test_interrupt(start_openarm_teleop, leader, follower):
    openarm_teleop = start_openarm_teleop(
        "bilateral",
        "--leader-can-interface", leader.interface,
        "--follower-can-interface", follower.interface)
    assert wait_until(lambda: all(m.enabled for m in follower.motors))

    # Ctrl+C
    openarm_teleop.send_signal(signal.SIGINT)
    assert openarm_teleop.wait(timeout=10) == 0
    for arm in (leader, follower):
        # The fake motors may still be processing the last commands.
        assert wait_until(lambda: not any(m.enabled for m in arm.motors))
