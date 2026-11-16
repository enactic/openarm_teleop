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

"""Fake Damiao motors of one OpenArm on a SocketCAN interface.

Only the enable command, the disable command and the MIT control
command are emulated. Each of them is answered with a state frame
that has the motor's master ID as the CAN ID.

Each motor is an independent joint that has only friction: no
inertia, no gravity and no link to the other joints. It doesn't move
while the torque is smaller than the static friction. Otherwise:

    (viscous friction + kd) * dq = kp * (q_des - q) + tau - static friction

dq_des is ignored.

So this is for checking what the executables send and how they react
to what they receive. A passing test does not replace a check on
real motors.
"""

import math
import socket
import struct
import threading
import time

_CAN_FRAME = struct.Struct("=IB3x8s")
_CANFD_FRAME = struct.Struct("=IB3x64s")
_ENABLE = bytes([0xFF] * 7 + [0xFC])
_DISABLE = bytes([0xFF] * 7 + [0xFD])

_POSITION_MAX = 12.5
# (velocity max, torque max) of J1, ..., J7 and the gripper: DM8009 x
# 2, DM4340 x 2 and DM4310 x 4. They must be the same as
# MOTOR_LIMIT_PARAMS in openarm_can.
_LIMITS = [(45, 54)] * 2 + [(10, 28)] * 2 + [(30, 10)] * 4

_VISCOUS_FRICTION = 0.5  # N m s/rad
_STATIC_FRICTION = 0.5  # N m


def _to_uint(x, x_min, x_max, bits):
    x = min(max(x, x_min), x_max)
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))


def _to_float(x, x_min, x_max, bits):
    return x / ((1 << bits) - 1) * (x_max - x_min) + x_min


class Motor:
    def __init__(self, slave_id, velocity_max, torque_max):
        self.slave_id = slave_id
        self.master_id = slave_id + 0x10
        self.velocity_max = velocity_max
        self.torque_max = torque_max
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.enabled = False
        self.held = False
        self._updated_at = None

    def control(self, data):
        """Applies a MIT control command for the time since the last one."""
        q = _to_float((data[0] << 8) | data[1],
                      -_POSITION_MAX, _POSITION_MAX, 16)
        kp = _to_float(((data[3] & 0xF) << 8) | data[4], 0, 500, 12)
        kd = _to_float((data[5] << 4) | (data[6] >> 4), 0, 5, 12)
        tau = _to_float(((data[6] & 0xF) << 8) | data[7],
                        -self.torque_max, self.torque_max, 12)

        now = time.monotonic()
        # Don't follow a long pause such as the one before the first
        # command.
        elapsed = min(now - (self._updated_at or now), 0.02)
        self._updated_at = now
        if not self.enabled or self.held:
            return
        self.torque = kp * (q - self.position) + tau
        if abs(self.torque) < _STATIC_FRICTION:
            self.velocity = 0.0
            return
        driving = self.torque - math.copysign(_STATIC_FRICTION, self.torque)
        self.velocity = driving / (_VISCOUS_FRICTION + kd)
        self.position += self.velocity * elapsed

    def state(self):
        q = _to_uint(self.position, -_POSITION_MAX, _POSITION_MAX, 16)
        dq = _to_uint(self.velocity,
                      -self.velocity_max, self.velocity_max, 12)
        tau = _to_uint(self.torque, -self.torque_max, self.torque_max, 12)
        # The ID in the low nibble, the state (1 = enabled, 0 =
        # disabled) in the high one, and then position, velocity,
        # torque and temperatures.
        return bytes([(self.slave_id & 0x0F) | (self.enabled << 4),
                      q >> 8, q & 0xFF,
                      dq >> 4, ((dq & 0xF) << 4) | (tau >> 8), tau & 0xFF,
                      25, 25])


class DamiaoSimulator:
    """Serves the motors from a background thread while in a with block."""

    def __init__(self, interface):
        self.interface = interface
        self.motors = [Motor(i + 1, *limits)
                       for i, limits in enumerate(_LIMITS)]
        # The number of received frames for the motors and CAN FD ones
        # in them.
        self.n_frames = 0
        self.n_fd_frames = 0
        self._lock = threading.Lock()
        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                     socket.CAN_RAW)
        self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES,
                                1)
        self._socket.settimeout(0.05)
        self._socket.bind((interface,))
        self._running = False
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self._running = True
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._running = False
        self._thread.join()
        self._socket.close()

    def hold(self, index, position):
        """Holds a joint at the position like a hand does.

        The joint ignores commands after this.
        """
        with self._lock:
            motor = self.motors[index]
            motor.position = position
            motor.velocity = 0.0
            motor.held = True

    def _serve(self):
        while self._running:
            try:
                frame = self._socket.recv(_CANFD_FRAME.size)
            except socket.timeout:
                continue
            is_fd = len(frame) == _CANFD_FRAME.size
            can_id, length = struct.unpack_from("=IB", frame)
            data = frame[8:8 + length]
            if not (1 <= can_id <= len(self.motors)) or length != 8:
                continue
            with self._lock:
                self.n_frames += 1
                if is_fd:
                    self.n_fd_frames += 1
                motor = self.motors[can_id - 1]
                if data == _ENABLE:
                    motor.enabled = True
                elif data == _DISABLE:
                    motor.enabled = False
                else:
                    motor.control(data)
                state = motor.state()
            if is_fd:
                self._socket.send(_CANFD_FRAME.pack(motor.master_id, 8, state))
            else:
                self._socket.send(_CAN_FRAME.pack(motor.master_id, 8, state))
