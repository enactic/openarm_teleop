#!/bin/sh
#
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

# Runs COMMAND with OPENARM_TELEOP_TEST_LEADER_INTERFACE and
# OPENARM_TELEOP_TEST_FOLLOWER_INTERFACE set to virtual CAN interfaces
# that nothing else uses.
#
# Usage: with-vcan.sh COMMAND [ARG...]
#
# If they are already set, they are used as is. Otherwise vcan0 and
# vcan1 are created in a new network namespace, which needs
# unprivileged user namespaces and the vcan kernel module. If neither
# is available, COMMAND is run without them: tests that need them are
# skipped.

set -eu

without_vcan() {
    echo "No virtual CAN interface: $1" >&2
    shift
    exec "$@"
}

if [ -n "${OPENARM_TELEOP_TEST_LEADER_INTERFACE:-}" ] &&
    [ -n "${OPENARM_TELEOP_TEST_FOLLOWER_INTERFACE:-}" ]; then
    exec "$@"
fi

if [ "${1:-}" != "--in-namespace" ]; then
    if ! type ip >/dev/null 2>&1; then
        without_vcan "ip (iproute2) is not installed" "$@"
    fi
    if ! unshare --user --map-root-user --net true 2>/dev/null; then
        without_vcan "unprivileged user namespaces are not available" "$@"
    fi
    exec unshare --user --map-root-user --net "$0" --in-namespace "$@"
fi
shift

for interface in vcan0 vcan1; do
    if ! ip link add dev "${interface}" type vcan 2>/dev/null; then
        without_vcan "${interface} can't be created: is the vcan kernel module loaded?" "$@"
    fi
    # For CAN FD
    ip link set dev "${interface}" mtu 72
    ip link set dev "${interface}" up
done
export OPENARM_TELEOP_TEST_LEADER_INTERFACE=vcan0
export OPENARM_TELEOP_TEST_FOLLOWER_INTERFACE=vcan1
exec "$@"
