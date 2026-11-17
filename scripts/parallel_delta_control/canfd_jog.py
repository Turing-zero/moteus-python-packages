"""Conservative one-motor +degree-and-return commissioning test.

All three motors are first held at their measured positions.  Only the selected
ID moves; the others remain at their initial targets.  The selected motor then
returns and all controllers are stopped.  No zero or persistent configuration
is changed.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from pathlib import Path

import numpy as np


_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
_REPO_MOTEUS = _HERE.parent.parent / "moteus"
if (_REPO_MOTEUS / "moteus" / "__init__.py").is_file():
    sys.path.insert(0, str(_REPO_MOTEUS))

import moteus
from moteus.protocol import Mode, Register
from canfd_query import DirectCandleTransport
from kinematics import forward_kinematics
from params import AppConfig


IDS = (1, 2, 3)
VELOCITY_LIMIT = 0.05  # motor rev/s = 18 deg/s
ACCEL_LIMIT = 0.2      # motor rev/s^2
MAX_TORQUE = 0.5       # Nm at the moteus motor-shaft/output coordinate
WATCHDOG_S = 0.25


def _value(result, register: Register):
    return result.values.get(register, result.values.get(int(register)))


def _position(result) -> float:
    return float(_value(result, Register.POSITION))


async def _query(transport, controllers) -> dict[int, moteus.protocol.Result]:
    results = await asyncio.wait_for(
        transport.cycle([c.make_query() for c in controllers.values()]), 2.0)
    return {int(result.id): result for result in results}


async def _check_ratio(controllers) -> None:
    for motor_id, controller in controllers.items():
        reply = await asyncio.wait_for(
            moteus.Stream(controller).command(
                b"conf get motor_position.rotor_to_output_ratio",
                allow_any_response=True), 5.0)
        ratio = float(reply.strip())
        print(f"ID {motor_id} 固件 rotor_to_output_ratio={ratio:g}")
        if abs(ratio - 1.0) > 1e-9:
            raise RuntimeError("固件位置比例不是 1，禁止直接按电机轴角度测试")


async def _stop(transport, controllers) -> None:
    try:
        await asyncio.wait_for(transport.cycle([
            controller.make_stop(query=False) for controller in controllers.values()
        ]), 1.0)
    except Exception as exc:
        print(f"停止命令发送失败：{exc}", file=sys.stderr)


async def _phase(transport, controllers, targets: dict[int, float],
                 duration_s: float, initial: dict[int, float], selected: int) -> None:
    deadline = time.monotonic() + duration_s
    latest = None
    while time.monotonic() < deadline:
        commands = []
        for motor_id, controller in controllers.items():
            commands.append(controller.make_position(
                position=targets[motor_id], velocity=0.0,
                velocity_limit=VELOCITY_LIMIT, accel_limit=ACCEL_LIMIT,
                maximum_torque=MAX_TORQUE, watchdog_timeout=WATCHDOG_S,
                query=True))
        results = await asyncio.wait_for(transport.cycle(commands), 0.2)
        latest = {int(result.id): result for result in results}
        for motor_id in IDS:
            result = latest.get(motor_id)
            if result is None:
                raise RuntimeError(f"ID {motor_id} 丢失反馈")
            fault = int(_value(result, Register.FAULT) or 0)
            mode = int(_value(result, Register.MODE))
            position = _position(result)
            if fault or mode in (Mode.FAULT, Mode.TIMEOUT) or not math.isfinite(position):
                raise RuntimeError(
                    f"ID {motor_id} 故障 mode={mode} fault={fault}")
            # Catch an unexpected large motion immediately.  Normal selected
            # motion is at most 64 deg; unselected axes should remain near home.
            if motor_id != selected and abs(position - initial[motor_id]) > 0.15:
                raise RuntimeError(f"非测试电机 ID {motor_id} 出现异常位移")
            if abs(position - initial[motor_id]) > 0.2:
                raise RuntimeError(f"ID {motor_id} 实际位移超过安全阈值")
        await asyncio.sleep(0.01)
    if latest:
        print("  当前位置 rev:", {motor_id: round(_position(latest[motor_id]), 6)
                                  for motor_id in IDS})


async def run_test(args) -> None:
    device = DirectCandleTransport(disable_brs=True)
    controllers = {}
    try:
        transport = moteus.Transport([device])
        controllers = {motor_id: moteus.Controller(id=motor_id, transport=transport)
                       for motor_id in IDS}
        await _check_ratio(controllers)
        # Clear a prior watchdog TIMEOUT before capturing and holding position.
        await _stop(transport, controllers)
        await asyncio.sleep(0.3)
        results = await _query(transport, controllers)
        initial = {}
        for motor_id in IDS:
            result = results.get(motor_id)
            if result is None:
                raise RuntimeError(f"ID {motor_id} 无应答")
            fault = int(_value(result, Register.FAULT) or 0)
            mode = int(_value(result, Register.MODE))
            if fault or mode in (Mode.FAULT, Mode.TIMEOUT):
                raise RuntimeError(f"ID {motor_id} 初始 mode={mode} FAULT={fault}")
            initial[motor_id] = _position(result)
        print("初始位置 rev:", {key: round(value, 6) for key, value in initial.items()})

        theoretical = [0.0, 0.0, 0.0]
        theoretical[args.id - 1] = args.delta_deg
        prediction = forward_kinematics(AppConfig(), theoretical)
        print(f"理论预测：ID {args.id} 相对当前位置 {args.delta_deg:+g}°，"
              f"RPY≈{np.round(prediction.rpy_deg, 6).tolist()}°")
        print("先保持三轴原位 1 秒，再低速移动、停留并回位；请观察机构并准备硬件急停。")

        targets = dict(initial)
        await _phase(transport, controllers, targets, 1.0, initial, args.id)
        targets[args.id] = initial[args.id] + args.delta_deg / 360.0
        print(f"ID {args.id} 开始 {args.delta_deg:+g}° 动作")
        move_duration = max(2.5, abs(args.delta_deg) / 360.0 / VELOCITY_LIMIT + 1.0)
        await _phase(transport, controllers, targets, move_duration, initial, args.id)
        print("到位停留 2 秒")
        await _phase(transport, controllers, targets, 2.0, initial, args.id)
        targets[args.id] = initial[args.id]
        print(f"ID {args.id} 回位")
        await _phase(transport, controllers, targets, move_duration, initial, args.id)
        print("回位动作结束，发送整组 stop")
        await _stop(transport, controllers)
    except BaseException:
        if controllers:
            await _stop(moteus.Transport([device]), controllers)
        raise
    finally:
        device.close()
        print("Candle 设备已关闭")


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="单电机小角度往返实物调试")
    parser.add_argument("--id", type=int, required=True, choices=IDS)
    parser.add_argument("--delta-deg", type=float, default=16.0,
                        help="相对当前位置的电机轴角度，范围 ±64°")
    args = parser.parse_args()
    if not math.isfinite(args.delta_deg) or abs(args.delta_deg) > 64.0:
        raise SystemExit("delta-deg 必须是 ±64° 内的有限数")
    asyncio.run(asyncio.wait_for(run_test(args), timeout=20.0))


if __name__ == "__main__":
    main()
