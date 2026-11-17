"""Read-only moteus queries through candle_api on Windows.

This bypasses the unavailable python-can ``candle`` backend.  It only calls
Controller.make_query(); no position, stop, rezero, or configuration command is
constructed by this tool.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from pathlib import Path
from typing import Any, Iterable


_HERE = Path(__file__).resolve().parent
_REPO_MOTEUS = _HERE.parent.parent / "moteus"
if (_REPO_MOTEUS / "moteus" / "__init__.py").is_file():
    sys.path.insert(0, str(_REPO_MOTEUS))

import moteus
from moteus.protocol import Register
from moteus.transport_device import Frame, TransportDevice
from candle_api import CandleCanFrame, CandleFrameType, list_device


_DLC_TO_BYTES = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def _dlc_code(byte_length: int) -> int:
    for code, size in enumerate(_DLC_TO_BYTES):
        if size == byte_length:
            return code
    raise ValueError(f"无效 CAN FD 数据长度 {byte_length}")


class DirectCandleTransport(TransportDevice):
    """Minimal candle_api TransportDevice supporting multiplex queries."""

    def __init__(self, serial_number: str | None = None, channel_index: int = 0,
                 disable_brs: bool = False, listen_only: bool = False):
        super().__init__()
        devices = list_device()
        if not devices:
            raise RuntimeError("未检测到 candleLightFD/CANable FD 设备")
        if serial_number is not None:
            devices = [d for d in devices if d.serial_number == serial_number]
            if not devices:
                raise RuntimeError(f"未找到序列号 {serial_number} 的 Candle 设备")
        self.device = devices[0]
        self.channel_index = channel_index
        self.disable_brs = disable_brs
        self.listen_only = listen_only
        self.device.open()
        self.channel = self.device[self.channel_index]
        self._configure_timing()
        self.channel.start(self.listen_only, False, False, False,
                           True, False, True, False)

    def _configure_timing(self) -> None:
        # This candleLightFD has a 40 MHz clock.  40 time quanta at BRP=1
        # gives 1 Mbit/s nominal; 8 quanta at BRP=1 gives 5 Mbit/s data.
        # Sample points are 62.5%, the closest practical approximation on this
        # clock to the moteus/PEAK timing guidance in pythoncan_device.py.
        if self.channel.clock_frequency != 40_000_000:
            raise RuntimeError(
                f"未适配的 Candle 时钟 {self.channel.clock_frequency} Hz；"
                "请核对位时序后再运行")
        self.channel.set_bit_timing(12, 12, 15, 10, 1)
        self.channel.set_data_bit_timing(2, 2, 3, 3, 1)

    def empty_bus_tx_safe(self) -> bool:
        return not self.listen_only

    def _to_candle(self, frame: Frame) -> CandleCanFrame:
        data_length = self._round_up_dlc(len(frame.data))
        data = frame.data + bytes([0x50]) * (data_length - len(frame.data))
        frame_type = CandleFrameType(
            extended_id=frame.is_extended_id,
            fd=frame.is_fd,
            bitrate_switch=frame.bitrate_switch and not self.disable_brs,
        )
        return CandleCanFrame(frame_type, frame.arbitration_id,
                              _dlc_code(data_length), data)

    def _from_candle(self, frame: CandleCanFrame) -> Frame:
        return Frame(
            arbitration_id=frame.can_id,
            data=frame.data,
            dlc=len(frame.data),
            is_extended_id=frame.frame_type.extended_id,
            is_fd=frame.frame_type.fd,
            bitrate_switch=frame.frame_type.bitrate_switch,
            channel=self,
        )

    async def send_frame(self, frame: Frame) -> None:
        if self.listen_only:
            raise RuntimeError("listen_only 模式禁止发送")
        self.channel.send_nowait(self._to_candle(frame))

    async def transaction(self, requests: list[TransportDevice.Request], **kwargs):
        subscriptions = []
        for request in requests:
            if request.frame_filter is None:
                continue
            future = asyncio.Future()

            async def handler(frame, req=request, fut=future):
                if not fut.done():
                    req.responses.append(frame)
                    fut.set_result(None)

            subscriptions.append((self._subscribe(request.frame_filter, handler), future))
        try:
            for request in requests:
                if request.frame is not None:
                    await self.send_frame(request.frame)
            pending = [future for _, future in subscriptions]
            while any(not future.done() for future in pending):
                frame = self.channel.receive_nowait()
                if frame is not None:
                    await self._handle_received_frame(self._from_candle(frame))
                await asyncio.sleep(0.001)
        finally:
            for subscription, _ in subscriptions:
                subscription.cancel()

    def close(self) -> None:
        # Stop/reset the channel before closing the USB device.  Without this,
        # candleLightFD can wait indefinitely after high-rate position cycles.
        try:
            self.channel.reset()
        except Exception:
            pass
        try:
            self.device.close()
        except Exception:
            pass


def _format_value(register: Register, value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


async def query_ids(args: argparse.Namespace) -> None:
    device = DirectCandleTransport(
        serial_number=args.serial, channel_index=args.channel,
        disable_brs=args.disable_brs, listen_only=args.listen_only)
    try:
        transport = moteus.Transport([device])
        controllers = [moteus.Controller(id=motor_id, transport=transport)
                       for motor_id in args.ids]
        if args.listen_only:
            print("仅监听总线 2 秒，不发送任何帧……")
            deadline = time.monotonic() + 2.0
            count = 0
            while time.monotonic() < deadline:
                frame = device.channel.receive_nowait()
                if frame is not None:
                    count += 1
                    print(f"RX id=0x{frame.can_id:x} fd={frame.frame_type.fd} "
                          f"data={frame.data.hex()}")
                await asyncio.sleep(0.001)
            print(f"共收到 {count} 帧")
            return
        print(f"仅查询电机 ID {args.ids}；不发送位置、使能或配置命令……")
        for motor_id, controller in zip(args.ids, controllers):
            try:
                result = await asyncio.wait_for(controller.query(), timeout=1.0)
            except asyncio.TimeoutError:
                result = None
            if result is None:
                print(f"ID {motor_id}: 无应答")
                continue
            print(f"ID {motor_id}: 收到应答")
            for register, value in sorted(result.values.items(), key=lambda item: int(item[0])):
                try:
                    name = Register(int(register)).name
                except ValueError:
                    name = f"REGISTER_0x{int(register):02x}"
                print(f"  {name}: {_format_value(register, value)}")
    finally:
        device.close()


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="Windows candle_api 只读 moteus 查询")
    parser.add_argument("--ids", type=lambda value: [int(v) for v in value.split(",")],
                        default=[1, 2, 3])
    parser.add_argument("--serial")
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--disable-brs", action="store_true")
    parser.add_argument("--listen-only", action="store_true")
    args = parser.parse_args()
    if any(motor_id < 1 or motor_id > 126 for motor_id in args.ids):
        raise SystemExit("电机 ID 必须在 1…126")
    asyncio.run(query_ids(args))


if __name__ == "__main__":
    main()
