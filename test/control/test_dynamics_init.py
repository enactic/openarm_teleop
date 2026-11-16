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

# The executables must set up the dynamics from the given URDF.

# A CAN interface that doesn't exist. An executable that passed the
# dynamics setup fails with CAN_ERROR instead of touching a real one.
CAN_INTERFACE = "openarm-none"
CAN_ERROR = "Failed to initialize socket for interface"


def test_gravity_comp(run_gravity_comp, write_urdf):
    urdf = write_urdf("openarm.urdf", "hand")
    result = run_gravity_comp("right_arm", CAN_INTERFACE, urdf)
    assert "Failed to get KDL chain" not in result.stdout
    assert CAN_ERROR in result.stdout


def test_teleop(run_teleop, write_urdf):
    urdf = write_urdf("openarm.urdf", "hand")
    result = run_teleop(urdf, urdf, "left_arm", CAN_INTERFACE, CAN_INTERFACE)
    assert "Failed to get KDL chain" not in result.stdout
    assert CAN_ERROR in result.stdout
