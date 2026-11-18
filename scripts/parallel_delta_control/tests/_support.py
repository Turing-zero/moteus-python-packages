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


def controlled_config():
    """Deterministic low-speed test setup, independent of production defaults."""
    config = AppConfig()
    config.transmission.directions = (1, 1, 1)
    config.transmission.belt_reduction = 2.0
    config.safety.feedback_rpy_margin_deg = 1.0
    config.safety.motor_min_deg = (-1300.0, -1300.0, -1300.0)
    config.safety.motor_max_deg = (1300.0, 1300.0, 1300.0)
    config.safety.max_move_deg = 2600.0
    config.safety.velocity_limit_rev_s = 0.5
    config.safety.accel_limit_rev_s2 = 2.0
    config.safety.maximum_torque_nm = None
    config.safety.kp_scale = None
    config.safety.kd_scale = None
    config.safety.state_publish_hz = 20.0
    config.safety.fk_tolerance = 1e-10
    config.safety.fk_max_iterations = 30
    config.safety.max_pose_step_deg = 30.0
    config.safety.max_feedback_step_deg = 30.0
    config.calibration.velocity_rev_s = 0.5
    config.calibration.accel_rev_s2 = 0.5
    return config


def fake_controller(config=None):
    config = config or controlled_config()
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
