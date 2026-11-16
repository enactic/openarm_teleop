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

# Teleoperation on virtual CAN interfaces with fake motors.

import math
import signal

from conftest import wait_until

J4 = 3
# INITIAL_POSITION in src/openarm_constants.hpp
J4_INITIAL_POSITION = math.pi / 5


def near(arm, joint, position):
    return abs(arm.motors[joint].position - position) < 0.05


def test_follow(start_teleop, write_urdf, leader, follower):
    urdf = write_urdf("openarm.urdf", "ee_base_link")
    teleop = start_teleop(urdf, urdf, "right_arm",
                          leader.interface, follower.interface)

    # Both arms move to the initial position.
    for arm in (leader, follower):
        assert wait_until(lambda: near(arm, J4, J4_INITIAL_POSITION))

    # A human moves the leader arm. The follower arm follows it.
    leader.hold(J4, 1.0)
    assert wait_until(lambda: near(follower, J4, 1.0))

    # Ctrl+C
    teleop.send_signal(signal.SIGINT)
    assert teleop.wait(timeout=10) == 0
    for arm in (leader, follower):
        # The fake motors may still be processing the last commands.
        assert wait_until(lambda: not any(m.enabled for m in arm.motors))
        assert arm.n_frames > 0
        assert arm.n_fd_frames == arm.n_frames
