# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

import asyncio
import collections
import logging
import time
import typing

from .transport_device import Frame, FrameFilter, TransportDevice

can: typing.Any = None


class CandleDevice(TransportDevice):
    """Candle CAN-FD USB adapter (Windows/Mac via python-can candle backend).

    Supports single and multi-channel candle devices.  Each instance
    represents one physical channel.  On Windows the candle backend
    auto-discovers attached devices via candle_api.
    """

    def __init__(
        self,
        serial_number: typing.Optional[str] = None,
        channel_index: int = 0,
        fd: bool = True,
        bitrate: int = 1_000_000,
        data_bitrate: int = 5_000_000,
        sample_point: float = 87.5,
        data_sample_point: float = 87.5,
        disable_brs: bool = False,
        listen_only: bool = False,
        termination: typing.Optional[bool] = None,
        debug_log=None,
        max_buffer_size: int = 50,
        padding_hex: str = '50',
    ):
        """Create a CandleDevice for one channel.

        Args:
          serial_number: USB serial number of the candle device.  When
            None the first available device is used.
          channel_index: Zero-based channel index on the device.
          fd: Enable CAN-FD mode.
          bitrate: Nominal (arbitration phase) bit rate in bps.
          data_bitrate: Data phase bit rate in bps (FD only).
          sample_point: Nominal sample point in percent (e.g. 87.5).
          data_sample_point: Data sample point in percent (FD only).
          disable_brs: If True, suppress the Bit Rate Switch flag even
            for FD frames (useful when the peer does not support BRS).
          listen_only: Enable listen-only mode for this channel.
          termination: Optional channel termination setting.
          debug_log: Optional file-like object; raw CAN log is written
            to it when provided.
          max_buffer_size: Maximum number of frames to buffer.
          padding_hex: Hex string for CAN-FD frame padding byte.
        """
        super().__init__(
            max_buffer_size=max_buffer_size,
            padding_hex=padding_hex,
        )

        self._padding_byte = bytes.fromhex(self._padding_hex)
        self._debug_log = debug_log
        self._disable_brs = disable_brs
        self._serial_number = serial_number
        self._channel_index = channel_index
        self._fd = fd
        self._setup = False
        self._notifier = None

        self._log_prefix = (
            f"candle:{serial_number}:{channel_index}"
            if serial_number else f"candle:{channel_index}"
        )

        global can
        if not can:
            import can as _can
            can = _can
            try:
                can.rc = can.util.load_config()
            except can.CanInterfaceNotImplementedError as e:
                if 'Unknown interface type "None"' not in str(e):
                    raise

        bus_kwargs: dict = dict(
            interface='candle',
            channel=channel_index,
            fd=fd,
            bitrate=bitrate,
            sample_point=sample_point,
            channel_configs={
                channel_index: {
                    'bitrate': bitrate,
                    'data_bitrate': data_bitrate if fd else None,
                    'sample_point': sample_point,
                    'data_sample_point': data_sample_point if fd else None,
                    'fd': fd,
                    'listen_only': listen_only,
                    'termination': termination,
                },
            },
        )
        if fd:
            bus_kwargs['data_bitrate'] = data_bitrate
            bus_kwargs['data_sample_point'] = data_sample_point
        if serial_number is not None:
            bus_kwargs['serial_number'] = serial_number

        self._can = can.Bus(**bus_kwargs)
        self._maybe_start_channel()

    def _maybe_start_channel(self):
        """Best-effort candle backend initialization and channel start."""
        # Different python-can/candle_api versions expose startup hooks
        # on different objects. Try known candidates and ignore unsupported
        # operations so older backends keep working.
        candidates = [self._can]
        for attr in ('candle', '_candle', 'device', '_device'):
            obj = getattr(self._can, attr, None)
            if obj is not None:
                candidates.append(obj)

        for obj in candidates:
            for method_name in ('init', 'initialize'):
                method = getattr(obj, method_name, None)
                if not callable(method):
                    continue
                try:
                    method(self._channel_index)
                except TypeError:
                    method()
                except Exception as e:
                    logging.debug(
                        f"candle init skipped for {self._log_prefix}: {e}"
                    )

        for obj in candidates:
            for method_name in ('start', 'startup'):
                method = getattr(obj, method_name, None)
                if not callable(method):
                    continue
                try:
                    method(self._channel_index)
                except TypeError:
                    method()
                except Exception as e:
                    logging.debug(
                        f"candle start skipped for {self._log_prefix}: {e}"
                    )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def serial_number(self) -> typing.Optional[str]:
        return self._serial_number

    @property
    def channel_index(self) -> int:
        return self._channel_index

    def __repr__(self):
        return f"CandleDevice('{self._log_prefix}')"

    # ------------------------------------------------------------------
    # TransportDevice interface
    # ------------------------------------------------------------------

    def empty_bus_tx_safe(self) -> bool:
        return True

    def close(self):
        if self._notifier:
            self._notifier.stop()
            self._notifier = None
        self._can.shutdown()

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _maybe_setup(self):
        if self._setup:
            return
        self._notifier = can.Notifier(
            self._can, [self._receive_handler],
            loop=asyncio.get_event_loop(),
        )
        self._setup = True

    async def _receive_handler(self, message):
        if message.is_error_frame:
            return
        frame = self._can_message_to_frame(message)
        if self._debug_log:
            self._write_log(
                f'< {frame.arbitration_id:04X} {frame.data.hex().upper()}'
                .encode('latin1')
            )
        await self._handle_received_frame(frame)

    def _can_message_to_frame(self, message) -> Frame:
        return Frame(
            arbitration_id=message.arbitration_id,
            data=message.data,
            dlc=message.dlc,
            is_extended_id=message.is_extended_id,
            is_fd=message.is_fd,
            bitrate_switch=getattr(message, 'bitrate_switch', False),
            channel=self,
        )

    def _frame_to_can_message(self, frame: Frame):
        dlc = self._round_up_dlc(len(frame.data))
        padding_bytes = dlc - len(frame.data)
        return can.Message(
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            dlc=dlc,
            data=frame.data + bytes(self._padding_byte) * padding_bytes,
            is_fd=frame.is_fd,
            bitrate_switch=frame.bitrate_switch and not self._disable_brs,
        )

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(
            f'{time.time():.6f}/{self._log_prefix} '.encode('latin1')
            + output + b'\n'
        )

    # ------------------------------------------------------------------
    # Frame I/O
    # ------------------------------------------------------------------

    async def send_frame(self, frame: Frame):
        self._maybe_setup()
        msg = self._frame_to_can_message(frame)
        if self._debug_log:
            self._write_log(
                f'> {frame.arbitration_id:04x} {msg.data.hex().upper()}'
                .encode('latin1')
            )
        self._can.send(msg)

    async def receive_frame(self) -> Frame:
        self._maybe_setup()
        return await super().receive_frame()

    async def transaction(
        self,
        requests: typing.List[TransportDevice.Request],
        **kwargs,
    ):
        self._maybe_setup()

        def make_subscription(request):
            future: asyncio.Future = asyncio.Future()

            async def handler(frame, _req=request, _fut=future):
                if _fut.done():
                    return
                _req.responses.append(frame)
                _fut.set_result(None)

            return self._subscribe(request.frame_filter, handler), future

        subscriptions = [
            make_subscription(req)
            for req in requests
            if req.frame_filter is not None
        ]

        try:
            for req in requests:
                if req.frame is not None:
                    await self.send_frame(req.frame)
            if subscriptions:
                await asyncio.gather(*[s[1] for s in subscriptions])
        finally:
            for s in subscriptions:
                s[0].cancel()

    # ------------------------------------------------------------------
    # Device enumeration
    # ------------------------------------------------------------------

    @staticmethod
    def enumerate_devices(**kwargs) -> typing.List['CandleDevice']:
        """Return one CandleDevice per channel found on all attached
        candle devices.

        Keyword args are forwarded to the CandleDevice constructor
        (fd, bitrate, data_bitrate, sample_point, data_sample_point,
        disable_brs, debug_log).

        Returns:
            List of CandleDevice instances, sorted by serial number and
            channel index.
        """
        global can
        if not can:
            import can as _can
            can = _can

        try:
            configs = can.detect_available_configs("candle")
        except Exception as e:
            logging.debug(f"candle enumeration failed: {e}")
            return []

        if not configs:
            return []

        # Group channels by serial number.
        # python-can candle reports channel as "serial_number:index".
        sn_channels: collections.defaultdict = collections.defaultdict(set)
        for cfg in configs:
            ch = cfg.get('channel', '')
            ch_str = str(ch) if ch is not None else ''
            if ':' in ch_str:
                sn, idx = ch_str.split(':', 1)
                try:
                    sn_channels[sn].add(int(idx))
                except ValueError:
                    pass
            else:
                try:
                    sn_channels[None].add(int(ch_str))
                except (ValueError, TypeError):
                    sn_channels[None].add(0)

        devices: typing.List[CandleDevice] = []
        ctor_keys = {
            'fd', 'bitrate', 'data_bitrate', 'sample_point',
            'data_sample_point', 'disable_brs', 'debug_log',
        }
        ctor_kwargs = {k: v for k, v in kwargs.items() if k in ctor_keys}

        for sn in sorted(sn_channels.keys(), key=lambda x: str(x or '')):
            for ch_idx in sorted(sn_channels[sn]):
                try:
                    dev = CandleDevice(
                        serial_number=sn,
                        channel_index=ch_idx,
                        **ctor_kwargs,
                    )
                    devices.append(dev)
                except Exception as e:
                    logging.warning(
                        f"Failed to open candle device {sn}:{ch_idx}: {e}"
                    )

        return devices
