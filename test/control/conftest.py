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

# OPENARM_TELEOP_BUILD_DIR is the build directory that has the
# executables to test. OPENARM_TELEOP_TEST_LEADER_INTERFACE and
# OPENARM_TELEOP_TEST_FOLLOWER_INTERFACE are virtual CAN interfaces
# the tests can use exclusively. test/with-vcan.sh sets up them.

import os
import pathlib
import subprocess
import sys
import time

import pytest

from damiao_simulator import DamiaoSimulator

# The executables load config/*.yaml from the current directory by
# default.
SOURCE_DIR = pathlib.Path(__file__).resolve().parents[2]

# A CAN interface that doesn't exist. An executable that reached the
# CAN interface setup fails with CAN_ERROR instead of touching a real
# one.
CAN_INTERFACE = "openarm-none"
CAN_ERROR = "Failed to initialize socket for interface"


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

    The executables expect "ee_base_link" (OpenArm 2.0) or "hand"
    (OpenArm 1.0).
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


def _build_dir():
    build_dir = os.environ.get("OPENARM_TELEOP_BUILD_DIR")
    if not build_dir:
        pytest.skip("OPENARM_TELEOP_BUILD_DIR is not set")
    return build_dir


def _executable(name):
    return os.path.join(_build_dir(), name)


def _runner(name):
    executable = _executable(name)

    def run(*args, cwd=SOURCE_DIR):
        return subprocess.run([executable, *args],
                              stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,
                              cwd=cwd,
                              text=True,
                              timeout=30)
    return run


def wait_until(condition, timeout=10):
    """Waits until condition() is true. Returns whether it got true."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return condition()


def _simulator(role):
    variable = f"OPENARM_TELEOP_TEST_{role}_INTERFACE"
    interface = os.environ.get(variable)
    if not interface:
        pytest.skip(f"{variable} is not set")
    return DamiaoSimulator(interface)


@pytest.fixture
def leader():
    """Fake motors of the leader arm."""
    with _simulator("LEADER") as simulator:
        yield simulator


@pytest.fixture
def follower():
    """Fake motors of the follower arm."""
    with _simulator("FOLLOWER") as simulator:
        yield simulator


@pytest.fixture
def run_gravity_comp():
    return _runner("gravity_comp")


@pytest.fixture(params=["unilateral_control", "bilateral_control"])
def run_teleop(request):
    """Runs unilateral_control and bilateral_control in turn."""
    return _runner(request.param)


@pytest.fixture(scope="session")
def openarm_teleop(tmp_path_factory):
    """Installed openarm-teleop."""
    prefix = tmp_path_factory.mktemp("install")
    subprocess.run(["cmake", "--install", _build_dir(), "--prefix", prefix],
                   stdout=subprocess.DEVNULL,
                   check=True)
    return str(prefix / "bin" / "openarm-teleop")


@pytest.fixture
def run_openarm_teleop(openarm_teleop, tmp_path):
    """Runs openarm-teleop out of the source directory."""

    def run(*args):
        return subprocess.run([sys.executable, openarm_teleop, *args],
                              stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,
                              cwd=tmp_path,
                              text=True,
                              timeout=30)
    return run


@pytest.fixture
def start_openarm_teleop(openarm_teleop, tmp_path):
    """Starts openarm-teleop out of the source directory."""
    yield from _starter([sys.executable, openarm_teleop], tmp_path)


@pytest.fixture(params=["unilateral_control", "bilateral_control"])
def start_teleop(request):
    """Starts unilateral_control and bilateral_control in turn."""
    yield from _starter([_executable(request.param)], SOURCE_DIR)


def _starter(command, cwd):
    processes = []

    def start(*args):
        # pytest shows the output when a test fails.
        process = subprocess.Popen([*command, *args], cwd=cwd)
        processes.append(process)
        return process
    yield start
    for process in processes:
        process.kill()
        process.wait()
