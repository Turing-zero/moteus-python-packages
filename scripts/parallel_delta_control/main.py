"""CLI entry point: python main.py --sim, or python main.py ik 0 0 60."""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import json
import logging
import math
import os
import sys
from pathlib import Path


_HERE = Path(__file__).resolve().parent
# Also support Python's isolated (-I) mode used for checked-out source trees.
sys.path.insert(0, str(_HERE))
_REPO_MOTEUS = _HERE.parent.parent / "moteus"
if (_REPO_MOTEUS / "moteus" / "__init__.py").is_file():
    sys.path.insert(0, str(_REPO_MOTEUS))

from kinematics import forward_kinematics, inverse_from_rpy, verify_inverse  # noqa: E402
from params import AppConfig, ConfigError, load_config  # noqa: E402


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="同轴三自由度并联机构控制与可视化")
    parser.add_argument("action", nargs="?", choices=("serve", "fk", "ik"), default="serve")
    parser.add_argument("angles", nargs="*", type=float,
                        help="fk: 三个相对电机角；ik: roll pitch yaw，单位度")
    parser.add_argument("--config", default=str(_HERE / "config.json"))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="使用三电机 moteus 模拟器（默认）")
    mode.add_argument("--hardware", action="store_true", help="显式启用 Linux SocketCAN 实物")
    parser.add_argument("--channel", default="can1")
    parser.add_argument("--ids", help="覆盖配置中的三个电机 ID，例如 1,2,3")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--disable-brs", action="store_true")
    parser.add_argument("--bitrate", type=int)
    parser.add_argument("--dbitrate", type=int)
    parser.add_argument("--restart-ms", type=int, default=100)
    parser.add_argument("--log-level", default="info")
    return parser


def _load(args: argparse.Namespace) -> AppConfig:
    config = load_config(args.config)
    if args.ids:
        try:
            ids = tuple(int(value) for value in args.ids.split(","))
        except ValueError as exc:
            raise ConfigError("--ids 必须是逗号分隔整数") from exc
        config.transmission = dataclasses.replace(config.transmission, motor_ids=ids)
        config.validate()
    return config


def _print_result(result) -> None:
    def convert(value):
        import numpy as np
        if dataclasses.is_dataclass(value):
            return convert(dataclasses.asdict(value))
        if isinstance(value, np.ndarray):
            return convert(value.tolist())
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, dict):
            return {key: convert(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [convert(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    print(json.dumps(convert(result), ensure_ascii=False, indent=2))


async def _serve(args: argparse.Namespace, config: AppConfig) -> None:
    import uvicorn
    from backends import CandleApiBackend, SimulatedBackend, SocketCanBackend
    from controller import MechanismController
    from server import create_app

    simulation = not args.hardware
    if simulation:
        backend = SimulatedBackend(config.transmission.motor_ids)
    elif os.name == "nt":
        backend = CandleApiBackend(config.transmission.motor_ids)
    else:
        backend = SocketCanBackend(
            config.transmission.motor_ids, channel=args.channel,
            restart_ms=args.restart_ms, disable_brs=args.disable_brs,
            bitrate=args.bitrate, dbitrate=args.dbitrate)
    controller = MechanismController(config, backend)
    app = create_app(controller, args.config)
    await controller.start()
    server = uvicorn.Server(uvicorn.Config(
        app, host=args.host, port=args.port, log_level=args.log_level,
        access_log=False))
    try:
        print(f"{'模拟' if simulation else '实物'}模式：http://{args.host}:{args.port}")
        await server.serve()
    finally:
        await controller.shutdown()


def main(argv: list[str] | None = None) -> None:
    args = _parser().parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    config = _load(args)
    if args.action in ("fk", "ik"):
        if len(args.angles) != 3:
            raise SystemExit(f"{args.action} 需要恰好三个角度参数")
        if args.action == "fk":
            result = forward_kinematics(config, args.angles)
        else:
            import numpy as np
            inverse = inverse_from_rpy(config, args.angles)
            result = verify_inverse(config, inverse, np.zeros(3), np.eye(3))
        _print_result(result)
        return
    if args.angles:
        raise SystemExit("serve 不接受位置参数")
    asyncio.run(_serve(args, config))


if __name__ == "__main__":
    main()
