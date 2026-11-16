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

# OPENARM_TELEOP_BUILD_DIR is the directory that has the executables
# to test. The tests must run in the top source directory because the
# executables load config/*.yaml from the current directory.

import os
import subprocess

import pytest


def _arm(side, leaf):
    """7 revolute joints from openarm_body_link0 to openarm_{side}_{leaf}."""
    links = (["openarm_body_link0"] +
             [f"openarm_{side}_link{i}" for i in range(1, 7)] +
             [f"openarm_{side}_{leaf}"])
    xml = ""
    for i, (parent, child) in enumerate(zip(links, links[1:]), start=1):
        xml += f"""
  <joint name="openarm_{side}_joint{i}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <axis xyz="0 0 1"/>
    <limit effort="10" lower="-1" upper="1" velocity="10"/>
  </joint>
  <link name="{child}"/>"""
    return xml


@pytest.fixture
def write_urdf(tmp_path):
    """Writes a bimanual URDF whose arms end with the given leaf link.

    The executables expect "hand" for now.
    """

    def write(name, leaf):
        path = tmp_path / name
        path.write_text(f"""<?xml version="1.0"?>
<robot name="openarm">
  <link name="openarm_body_link0"/>{_arm("left", leaf)}{_arm("right", leaf)}
</robot>
""")
        return str(path)
    return write


def _runner(name):
    build_dir = os.environ.get("OPENARM_TELEOP_BUILD_DIR")
    if not build_dir:
        pytest.skip("OPENARM_TELEOP_BUILD_DIR is not set")

    def run(*args):
        return subprocess.run([os.path.join(build_dir, name), *args],
                              stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,
                              text=True,
                              timeout=30)
    return run


@pytest.fixture
def run_gravity_comp():
    return _runner("gravity_comp")


@pytest.fixture(params=["unilateral_control", "bilateral_control"])
def run_teleop(request):
    """Runs unilateral_control and bilateral_control in turn."""
    return _runner(request.param)
