import sys
import time
from pathlib import Path

import numpy as np
from moteus.protocol import Mode, Register

from backends import feedback_mode_name
from controller import MechanismController
from params import AppConfig


class FakeBackend:
    simulation = False
    mode_name = "hardware"

    def __init__(self, ids=(1, 2, 3)):
        self.ids = tuple(ids)
        self.is_open = True
        self.closed = False
        self.stop_calls = 0

    async def close(self, send_stop=True):
        self.is_open = False
        self.closed = True
        if send_stop:
            self.stop_calls += 1

    def reconfigure_ids(self, ids):
        self.ids = tuple(ids)
        self.is_open = False


def fake_controller(config=None):
    config = config or AppConfig()
    if config.calibration.firmware_max_current_a is None:
        config.calibration.firmware_max_current_a = 10.0
    if config.calibration.motor_kv is None:
        config.calibration.motor_kv = 100.0
    backend = FakeBackend(config.transmission.motor_ids)
    controller = MechanismController(config, backend, auto_calibrate_simulation=False)
    feed(controller, [10.25, -3.5, 0.75])
    return controller


def feed(controller, raw_positions, mode=Mode.STOPPED, fault=0,
         velocities=None, torques=None, q_currents=None):
    velocities = [0.0] * 3 if velocities is None else list(velocities)
    torques = [0.0] * 3 if torques is None else list(torques)
    q_currents = [0.0] * 3 if q_currents is None else list(q_currents)
    now = time.monotonic()
    for motor_id, position, velocity, torque, current in zip(
            controller.ids, raw_positions, velocities, torques, q_currents):
        motor = controller.motors[motor_id]
        motor.values = {
            Register.POSITION: float(position),
            Register.MODE: int(mode),
            Register.FAULT: int(fault),
            Register.VELOCITY: float(velocity),
            Register.TORQUE: float(torque),
            Register.Q_CURRENT: float(current),
            Register.VOLTAGE: 24.0,
            Register.TEMPERATURE: 30.0,
            Register.TRAJECTORY_COMPLETE: 1,
        }
        motor.online = True
        motor.ever_online = True
        motor.last_update = now
    if controller.calibrated:
        controller._update_actual_pose()
