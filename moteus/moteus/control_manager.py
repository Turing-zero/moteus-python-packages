# Copyright 2025 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Thread-safe moteus control manager for high-frequency CAN control.

Provides ControlManager (runs asyncio in background thread with thread-safe API)
and CsvLogger (plug-in listener for high-frequency data recording).

Supported command types
-----------------------
- stop        : disable motor, clear persistent commands
- brake       : hold motor at zero (regenerative braking)
- zero_vel    : hold zero velocity with kd gain
- position    : position/velocity tracking (most common mode)
- vfoc        : voltage FOC (open-loop voltage command)
- current     : direct dq current control
- stay_within : soft position bounds with torque feedforward
- rezero      : re-zero the output position estimate
"""

import asyncio
import csv
import dataclasses
import math
import queue
import threading
import time
from enum import Enum, auto
from typing import Any, Callable, Dict, List, Optional

from moteus import multiplex as mp
from moteus.moteus import Controller, QueryResolution
from moteus.protocol import Register
from moteus.transport import Transport as _Transport
from moteus.fdcanusb_device import FdcanusbDevice
from moteus.pythoncan_device import PythonCanDevice
from moteus.candle_device import CandleDevice


class ManagerState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


@dataclasses.dataclass
class ControllerStatus:
    """Snapshot of a single controller's state (safe to read from any thread)."""
    controller_id: int
    mode: int = 0
    position: float = float('nan')
    velocity: float = float('nan')
    torque: float = float('nan')
    voltage: float = float('nan')
    temperature: float = float('nan')
    fault: int = 0
    trajectory_complete: int = 0
    abs_position: float = float('nan')
    encoder_0_position: float = float('nan')
    encoder_0_velocity: float = float('nan')
    encoder_1_position: float = float('nan')
    encoder_1_velocity: float = float('nan')
    encoder_2_position: float = float('nan')
    encoder_2_velocity: float = float('nan')
    encoder_validity: int = 0
    last_update: float = 0.0


@dataclasses.dataclass
class _CommandItem:
    # Core fields
    command_type: str          # see module docstring for valid values
    controller_ids: List[int]  # empty = all managed controllers
    persistent: bool = False   # repeat every cycle until cleared
    key: Optional[str] = None  # deduplication key for persistent cmds

    # position / stay_within / zero_vel shared
    kp_scale: Optional[float] = None
    kd_scale: Optional[float] = None
    maximum_torque: Optional[float] = None
    feedforward_torque: Optional[float] = None
    watchdog_timeout: Optional[float] = None

    # position
    position: Optional[float] = None
    velocity: Optional[float] = None
    stop_position: Optional[float] = None
    velocity_limit: Optional[float] = None
    accel_limit: Optional[float] = None

    # vfoc
    vfoc_theta: Optional[float] = None
    vfoc_voltage: Optional[float] = None
    vfoc_theta_rate: Optional[float] = None

    # current
    d_A: Optional[float] = None
    q_A: Optional[float] = None

    # stay_within
    lower_bound: Optional[float] = None
    upper_bound: Optional[float] = None

    # rezero
    rezero_pos: Optional[float] = None


def make_query_resolution(profile: str = 'full') -> QueryResolution:
    """Build a ``QueryResolution`` for ControlManager cycles.

    Profiles
    --------
    - ``lite``: mode/position/velocity/torque/voltage/temperature/fault.
      Smaller CAN replies; typically ~2–4× faster than ``full`` on USB adapters.
    - ``full``: also trajectory_complete, abs_position, and encoder 0–2.
    """
    profile = (profile or 'full').lower()
    if profile == 'lite':
        return QueryResolution()

    if profile != 'full':
        raise ValueError(f"Unknown query profile {profile!r}; use 'lite' or 'full'")

    qr = QueryResolution()
    qr.trajectory_complete = mp.INT8
    qr.abs_position = mp.F32
    qr._extra = {
        int(Register.ENCODER_0_POSITION): mp.F32,
        int(Register.ENCODER_0_VELOCITY): mp.F32,
        int(Register.ENCODER_1_POSITION): mp.F32,
        int(Register.ENCODER_1_VELOCITY): mp.F32,
        int(Register.ENCODER_2_POSITION): mp.F32,
        int(Register.ENCODER_2_VELOCITY): mp.F32,
        int(Register.ENCODER_VALIDITY): mp.INT8,
    }
    return qr


def _make_default_qr() -> QueryResolution:
    """Backward-compatible alias for ``make_query_resolution('full')``."""
    return make_query_resolution('full')


class ControlManager:
    """High-frequency moteus control manager.

    Runs an asyncio event loop in a background thread so that the caller
    (e.g. a GUI main thread) can use a purely synchronous, thread-safe API.

    Typical usage::

        manager = ControlManager()
        manager.connect([1, 2], can_type='socketcan', can_chan='can0')
        time.sleep(0.1)  # wait for CONNECTED
        manager.command_position([1], position=0.5, persistent=True)
        ...
        manager.command_stop()
        manager.disconnect()
    """

    def __init__(
        self,
        cycle_hz: float = 200.0,
        query_profile: str = 'full',
    ):
        self._cycle_period_s = 1.0 / cycle_hz
        self._query_profile = query_profile

        self._state = ManagerState.DISCONNECTED
        self._state_lock = threading.RLock()
        self._last_error: Optional[str] = None

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._cmd_queue: Optional[asyncio.Queue] = None
        self._stop_event: Optional[asyncio.Event] = None

        self._status: Dict[int, ControllerStatus] = {}
        self._status_lock = threading.RLock()

        self._listeners: List[Callable[[Dict[int, ControllerStatus]], None]] = []
        self._listeners_lock = threading.RLock()

    # ── Public API (thread-safe) ──────────────────────────────────────────────

    def connect(
        self,
        controller_ids: List[int],
        can_type: str = 'socketcan',
        can_iface: Optional[str] = None,
        can_chan: Optional[str] = 'can0',
        can_disable_brs: bool = False,
    ) -> None:
        """Open CAN transport and start control loop in background thread.

        Non-blocking.  Poll ``get_state()`` to detect when ready.

        Args:
            controller_ids: CAN IDs to manage, e.g. ``[1, 2, 3]``
            can_type: ``'socketcan'`` | ``'fdcanusb'`` | ``'candle'``
            can_iface: python-can interface string (ignored for fdcanusb/candle)
            can_chan: Channel.  socketcan → ``'can0'``;
                      fdcanusb → ``'/dev/ttyUSB0'`` or ``'COM3'``;
                      candle → integer index string ``'0'``
            can_disable_brs: Disable CAN-FD BRS flag on transmit frames.
        """
        with self._state_lock:
            if self._state in (ManagerState.CONNECTING, ManagerState.CONNECTED):
                return
            self._state = ManagerState.CONNECTING
            self._last_error = None

        self._loop_thread = threading.Thread(
            target=self._start_loop,
            args=(list(controller_ids), can_type, can_iface, can_chan, can_disable_brs),
            daemon=True,
            name='moteus-control-loop',
        )
        self._loop_thread.start()

    def disconnect(self) -> None:
        """Stop control loop and close CAN transport.  Blocks up to 3 s."""
        if self._loop is not None and self._stop_event is not None:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._loop_thread is not None and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=3.0)
        self._loop = None
        self._loop_thread = None
        with self._state_lock:
            self._state = ManagerState.DISCONNECTED

    def get_state(self) -> ManagerState:
        with self._state_lock:
            return self._state

    def get_last_error(self) -> Optional[str]:
        with self._state_lock:
            return self._last_error

    def is_connected(self) -> bool:
        return self.get_state() == ManagerState.CONNECTED

    def set_query_profile(self, profile: str) -> None:
        """Set ``lite``/``full`` query profile (takes effect on next connect)."""
        # Validate early.
        make_query_resolution(profile)
        with self._state_lock:
            if self._state in (ManagerState.CONNECTING, ManagerState.CONNECTED):
                raise RuntimeError('Cannot change query_profile while connected')
            self._query_profile = profile

    def get_query_profile(self) -> str:
        return self._query_profile

    def get_status(self) -> Dict[int, ControllerStatus]:
        """Return a thread-safe snapshot of all controller statuses."""
        with self._status_lock:
            return {cid: dataclasses.replace(s) for cid, s in self._status.items()}

    def get_status_one(self, controller_id: int) -> Optional[ControllerStatus]:
        with self._status_lock:
            s = self._status.get(controller_id)
            return dataclasses.replace(s) if s is not None else None

    def add_listener(
        self,
        callback: Callable[[Dict[int, ControllerStatus]], None],
    ) -> None:
        """Register a status callback (called from asyncio thread, must be fast)."""
        with self._listeners_lock:
            if callback not in self._listeners:
                self._listeners.append(callback)

    def remove_listener(
        self,
        callback: Callable[[Dict[int, ControllerStatus]], None],
    ) -> None:
        with self._listeners_lock:
            try:
                self._listeners.remove(callback)
            except ValueError:
                pass

    # ── Command API ───────────────────────────────────────────────────────────

    def command_stop(self, controller_ids: Optional[List[int]] = None) -> None:
        """Disable motor and clear any persistent commands."""
        self._enqueue(_CommandItem('stop', list(controller_ids or [])))

    def command_brake(self, controller_ids: Optional[List[int]] = None) -> None:
        """Regenerative braking (one-shot; motor stays braked)."""
        self._enqueue(_CommandItem('brake', list(controller_ids or [])))

    def command_zero_velocity(
        self,
        controller_ids: Optional[List[int]] = None,
        kd_scale: Optional[float] = None,
    ) -> None:
        """Hold zero velocity using kd gain (one-shot by default)."""
        self._enqueue(_CommandItem('zero_vel', list(controller_ids or []),
                                   kd_scale=kd_scale))

    def command_position(
        self,
        controller_ids: List[int],
        position: float,
        velocity: Optional[float] = None,
        feedforward_torque: Optional[float] = None,
        kp_scale: Optional[float] = None,
        kd_scale: Optional[float] = None,
        maximum_torque: Optional[float] = None,
        stop_position: Optional[float] = None,
        watchdog_timeout: Optional[float] = None,
        velocity_limit: Optional[float] = None,
        accel_limit: Optional[float] = None,
        persistent: bool = True,
        key: Optional[str] = None,
    ) -> None:
        """Position/velocity tracking command.

        ``persistent=True`` (default) re-sends the command every control cycle,
        which is required to prevent the moteus watchdog from triggering.
        """
        if key is None:
            key = 'pos_' + '_'.join(str(i) for i in sorted(controller_ids))
        self._enqueue(_CommandItem(
            'position', list(controller_ids),
            position=position, velocity=velocity,
            feedforward_torque=feedforward_torque,
            kp_scale=kp_scale, kd_scale=kd_scale,
            maximum_torque=maximum_torque,
            stop_position=stop_position,
            watchdog_timeout=watchdog_timeout,
            velocity_limit=velocity_limit,
            accel_limit=accel_limit,
            persistent=persistent, key=key,
        ))

    def command_vfoc(
        self,
        controller_ids: List[int],
        theta: float,
        voltage: float,
        theta_rate: float = 0.0,
        persistent: bool = False,
        key: Optional[str] = None,
    ) -> None:
        """Voltage FOC command (open-loop voltage at given electrical angle).

        Args:
            theta: Electrical angle in radians.
            voltage: Voltage magnitude (V).
            theta_rate: Rate of change of theta (rad/s).
        """
        if key is None:
            key = 'vfoc_' + '_'.join(str(i) for i in sorted(controller_ids))
        self._enqueue(_CommandItem(
            'vfoc', list(controller_ids),
            vfoc_theta=theta, vfoc_voltage=voltage, vfoc_theta_rate=theta_rate,
            persistent=persistent, key=key,
        ))

    def command_current(
        self,
        controller_ids: List[int],
        d_A: float,
        q_A: float,
        persistent: bool = False,
        key: Optional[str] = None,
    ) -> None:
        """Direct dq-frame current control.

        Args:
            d_A: d-axis current (A).
            q_A: q-axis current (A) — controls torque.
        """
        if key is None:
            key = 'cur_' + '_'.join(str(i) for i in sorted(controller_ids))
        self._enqueue(_CommandItem(
            'current', list(controller_ids),
            d_A=d_A, q_A=q_A,
            persistent=persistent, key=key,
        ))

    def command_stay_within(
        self,
        controller_ids: List[int],
        lower_bound: Optional[float] = None,
        upper_bound: Optional[float] = None,
        feedforward_torque: Optional[float] = None,
        kp_scale: Optional[float] = None,
        kd_scale: Optional[float] = None,
        maximum_torque: Optional[float] = None,
        watchdog_timeout: Optional[float] = None,
        persistent: bool = True,
        key: Optional[str] = None,
    ) -> None:
        """Soft position bounds — only applies torque near the limits.

        The motor moves freely between ``lower_bound`` and ``upper_bound``
        and applies a restoring torque when outside the bounds.
        """
        if key is None:
            key = 'sw_' + '_'.join(str(i) for i in sorted(controller_ids))
        self._enqueue(_CommandItem(
            'stay_within', list(controller_ids),
            lower_bound=lower_bound, upper_bound=upper_bound,
            feedforward_torque=feedforward_torque,
            kp_scale=kp_scale, kd_scale=kd_scale,
            maximum_torque=maximum_torque,
            watchdog_timeout=watchdog_timeout,
            persistent=persistent, key=key,
        ))

    def command_velocity(
        self,
        controller_ids: List[int],
        velocity: float,
        feedforward_torque: Optional[float] = None,
        kp_scale: Optional[float] = None,
        kd_scale: Optional[float] = None,
        maximum_torque: Optional[float] = None,
        watchdog_timeout: Optional[float] = None,
        persistent: bool = True,
        key: Optional[str] = None,
    ) -> None:
        """Velocity tracking command (position = NaN → velocity-only mode).

        The motor tracks the target velocity without position feedback.
        Set ``persistent=True`` (default) to keep sending the command and
        avoid the watchdog timeout.
        """
        if key is None:
            key = 'vel_' + '_'.join(str(i) for i in sorted(controller_ids))
        self._enqueue(_CommandItem(
            'position', list(controller_ids),
            position=math.nan,
            velocity=velocity,
            feedforward_torque=feedforward_torque,
            kp_scale=kp_scale,
            kd_scale=kd_scale,
            maximum_torque=maximum_torque,
            watchdog_timeout=watchdog_timeout,
            persistent=persistent, key=key,
        ))

    def command_rezero(
        self,
        controller_ids: Optional[List[int]] = None,
        rezero: float = 0.0,
    ) -> None:
        """Set the output position estimate to ``rezero`` (one-shot).

        Equivalent to ``set_output_nearest`` — snaps the position encoder
        to the nearest output consistent with the given value.
        """
        self._enqueue(_CommandItem(
            'rezero', list(controller_ids or []),
            rezero_pos=rezero,
        ))

    # ── Private implementation ────────────────────────────────────────────────

    def _enqueue(self, item: _CommandItem) -> None:
        loop = self._loop
        q = self._cmd_queue
        if loop is not None and not loop.is_closed() and q is not None:
            loop.call_soon_threadsafe(q.put_nowait, item)

    def _start_loop(
        self,
        controller_ids: List[int],
        can_type: str,
        can_iface: Optional[str],
        can_chan: Optional[str],
        can_disable_brs: bool,
    ) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._cmd_queue = asyncio.Queue()
        self._stop_event = asyncio.Event()
        try:
            loop.run_until_complete(
                self._async_main(
                    controller_ids,
                    can_type,
                    can_iface,
                    can_chan,
                    can_disable_brs,
                )
            )
        except Exception as exc:
            with self._state_lock:
                self._state = ManagerState.ERROR
                self._last_error = str(exc)
        finally:
            try:
                loop.run_until_complete(loop.shutdown_asyncgens())
            except Exception:
                pass
            loop.close()
            with self._state_lock:
                if self._state == ManagerState.CONNECTED:
                    self._state = ManagerState.DISCONNECTED

    async def _async_main(
        self,
        controller_ids: List[int],
        can_type: str,
        can_iface: Optional[str],
        can_chan: Optional[str],
        can_disable_brs: bool,
    ) -> None:
        transport = await self._build_transport(
            can_type, can_iface, can_chan, can_disable_brs
        )
        qr = make_query_resolution(self._query_profile)
        controllers = {
            cid: Controller(id=cid, transport=transport, query_resolution=qr)
            for cid in controller_ids
        }
        with self._state_lock:
            self._state = ManagerState.CONNECTED
        try:
            await self._control_loop(transport, controllers)
        finally:
            try:
                transport.close()
            except Exception:
                pass

    async def _build_transport(
        self,
        can_type: str,
        can_iface: Optional[str],
        can_chan: Optional[str],
        can_disable_brs: bool,
    ) -> _Transport:
        if can_type == 'fdcanusb':
            device = FdcanusbDevice(can_chan, disable_brs=can_disable_brs)
        elif can_type == 'candle':
            device = CandleDevice(
                channel_index=int(can_chan or '0'),
                disable_brs=can_disable_brs,
            )
        else:
            kwargs: Dict[str, Any] = {}
            if can_chan:
                kwargs['channel'] = can_chan
            if can_iface:
                kwargs['interface'] = can_iface
            if can_disable_brs:
                kwargs['disable_brs'] = True
            device = PythonCanDevice(**kwargs)
        return _Transport([device])

    async def _control_loop(
        self,
        transport: _Transport,
        controllers: Dict[int, Controller],
    ) -> None:
        stop_event = self._stop_event
        cmd_queue = self._cmd_queue
        if stop_event is None or cmd_queue is None:
            return

        persistent_cmds: Dict[str, _CommandItem] = {}
        one_shot: Dict[int, _CommandItem] = {}
        loop = asyncio.get_event_loop()

        # Keep transport transactions bounded so disconnect can terminate
        # promptly even if a backend call blocks unexpectedly.
        cycle_timeout = max(0.05, self._cycle_period_s * 2.0)

        while not stop_event.is_set():
            t0 = loop.time()

            # Drain command queue
            while True:
                try:
                    item = cmd_queue.get_nowait()
                    _apply_command(item, persistent_cmds, one_shot, controllers)
                except asyncio.QueueEmpty:
                    break

            # Build command list: one_shot > persistent > query
            cmds = []
            for cid, ctrl in controllers.items():
                if cid in one_shot:
                    cmds.append(_build_cmd(ctrl, one_shot[cid]))
                else:
                    pcmd = _find_persistent(cid, persistent_cmds)
                    cmds.append(_build_cmd(ctrl, pcmd))
            one_shot.clear()

            if cmds:
                try:
                    results = await asyncio.wait_for(
                        transport.cycle(cmds),
                        timeout=cycle_timeout,
                    )
                    self._update_status(results)
                    self._fire_listeners()
                except asyncio.TimeoutError:
                    # Drop this cycle and continue so shutdown remains responsive.
                    pass
                except Exception:
                    pass

            elapsed = loop.time() - t0
            await asyncio.sleep(max(0.0, self._cycle_period_s - elapsed))

    def _update_status(self, results: Any) -> None:
        now = time.monotonic()
        updates: Dict[int, ControllerStatus] = {}
        for r in results:
            if r is None or not hasattr(r, 'values') or not r.values:
                continue
            cid = getattr(r, 'id', None)
            if cid is None:
                continue
            v = r.values
            updates[cid] = ControllerStatus(
                controller_id=cid,
                mode=int(v.get(Register.MODE, 0)),
                position=float(v.get(Register.POSITION, math.nan)),
                velocity=float(v.get(Register.VELOCITY, math.nan)),
                torque=float(v.get(Register.TORQUE, math.nan)),
                voltage=float(v.get(Register.VOLTAGE, math.nan)),
                temperature=float(v.get(Register.TEMPERATURE, math.nan)),
                fault=int(v.get(Register.FAULT, 0)),
                trajectory_complete=int(v.get(Register.TRAJECTORY_COMPLETE, 0)),
                abs_position=float(v.get(Register.ABS_POSITION, math.nan)),
                encoder_0_position=float(v.get(Register.ENCODER_0_POSITION, math.nan)),
                encoder_0_velocity=float(v.get(Register.ENCODER_0_VELOCITY, math.nan)),
                encoder_1_position=float(v.get(Register.ENCODER_1_POSITION, math.nan)),
                encoder_1_velocity=float(v.get(Register.ENCODER_1_VELOCITY, math.nan)),
                encoder_2_position=float(v.get(Register.ENCODER_2_POSITION, math.nan)),
                encoder_2_velocity=float(v.get(Register.ENCODER_2_VELOCITY, math.nan)),
                encoder_validity=int(v.get(Register.ENCODER_VALIDITY, 0)),
                last_update=now,
            )
        if updates:
            with self._status_lock:
                self._status.update(updates)

    def _fire_listeners(self) -> None:
        snapshot = self.get_status()
        with self._listeners_lock:
            fns = list(self._listeners)
        for fn in fns:
            try:
                fn(snapshot)
            except Exception:
                pass


# ── Module-level helpers ──────────────────────────────────────────────────────

def _apply_command(
    item: _CommandItem,
    persistent: Dict[str, _CommandItem],
    one_shot: Dict[int, _CommandItem],
    controllers: Dict[int, Controller],
) -> None:
    targets = item.controller_ids if item.controller_ids else list(controllers.keys())

    if item.command_type == 'stop':
        # Clear persistent commands for these targets
        for k in list(persistent.keys()):
            v = persistent[k]
            if not v.controller_ids or any(c in v.controller_ids for c in targets):
                del persistent[k]
        for cid in targets:
            one_shot[cid] = item
    elif item.persistent and item.key:
        persistent[item.key] = item
    else:
        for cid in targets:
            one_shot[cid] = item


def _find_persistent(
    cid: int,
    cmds: Dict[str, _CommandItem],
) -> Optional[_CommandItem]:
    for item in cmds.values():
        if not item.controller_ids or cid in item.controller_ids:
            return item
    return None


def _opt_kwargs(item: _CommandItem, **fields) -> Dict[str, Any]:
    """Build kwargs dict from _CommandItem, omitting None values."""
    return {k: getattr(item, src) for k, src in fields.items()
            if getattr(item, src) is not None}


def _build_cmd(ctrl: Controller, item: Optional[_CommandItem]):
    """Translate a _CommandItem into the appropriate moteus Command."""
    if item is None:
        return ctrl.make_query()

    ct = item.command_type

    if ct == 'stop':
        return ctrl.make_stop(query=True)

    if ct == 'brake':
        return ctrl.make_brake(query=True)

    if ct == 'zero_vel':
        kwargs: Dict[str, Any] = {'query': True}
        if item.kd_scale is not None:
            kwargs['kd_scale'] = item.kd_scale
        return ctrl.make_zero_velocity(**kwargs)

    if ct == 'position':
        kwargs = {'query': True}
        kwargs.update(_opt_kwargs(item,
            position='position', velocity='velocity',
            feedforward_torque='feedforward_torque',
            kp_scale='kp_scale', kd_scale='kd_scale',
            maximum_torque='maximum_torque',
            stop_position='stop_position',
            watchdog_timeout='watchdog_timeout',
            velocity_limit='velocity_limit',
            accel_limit='accel_limit',
        ))
        return ctrl.make_position(**kwargs)

    if ct == 'vfoc':
        return ctrl.make_vfoc(
            theta=item.vfoc_theta,
            voltage=item.vfoc_voltage,
            theta_rate=item.vfoc_theta_rate or 0.0,
            query=True,
        )

    if ct == 'current':
        return ctrl.make_current(
            d_A=item.d_A,
            q_A=item.q_A,
            query=True,
        )

    if ct == 'stay_within':
        kwargs = {'query': True}
        kwargs.update(_opt_kwargs(item,
            lower_bound='lower_bound', upper_bound='upper_bound',
            feedforward_torque='feedforward_torque',
            kp_scale='kp_scale', kd_scale='kd_scale',
            maximum_torque='maximum_torque',
            watchdog_timeout='watchdog_timeout',
        ))
        return ctrl.make_stay_within(**kwargs)

    if ct == 'rezero':
        return ctrl.make_rezero(rezero=item.rezero_pos or 0.0, query=True)

    return ctrl.make_query()


# ── CsvLogger plugin ──────────────────────────────────────────────────────────

class CsvLogger:
    """Records controller status to a CSV file.

    Attach to a ``ControlManager`` via ``add_listener``::

        logger = CsvLogger('data.csv', controller_ids=[1, 2])
        manager.add_listener(logger.on_status_update)
        ...
        logger.close()

    By default writes happen on a background thread so disk I/O does not stall
    the control loop.  Pass ``async_write=False`` for synchronous line-buffered
    writes (useful for debugging).
    """

    DEFAULT_FIELDS = [
        'timestamp', 'id', 'mode',
        'position', 'velocity', 'torque',
        'voltage', 'temperature', 'fault',
    ]

    ALL_FIELDS = DEFAULT_FIELDS + [
        'trajectory_complete',
        'abs_position',
        'encoder_0_position', 'encoder_0_velocity',
        'encoder_1_position', 'encoder_1_velocity',
        'encoder_2_position', 'encoder_2_velocity',
        'encoder_validity',
    ]

    def __init__(
        self,
        filepath: str,
        controller_ids: Optional[List[int]] = None,
        fields: Optional[List[str]] = None,
        async_write: bool = True,
    ):
        self._filter_ids = set(controller_ids) if controller_ids else None
        self._fields = fields or self.DEFAULT_FIELDS
        # Larger buffer when async; line-buffer when sync for immediate visibility.
        bufsize = -1 if async_write else 1
        self._file = open(filepath, 'w', newline='', buffering=bufsize)
        self._writer = csv.DictWriter(self._file, fieldnames=self._fields)
        self._writer.writeheader()
        self._lock = threading.Lock()
        # Use perf_counter: on Windows, time.monotonic() is GetTickCount64()
        # with ~15.6ms resolution, which quantizes CSV timestamps and makes
        # high-rate logs look like ~64Hz even when the loop is faster.
        self._t0 = time.perf_counter()
        self._row_count = 0
        self._async_write = async_write
        self._write_q: Optional[queue.SimpleQueue] = None
        self._writer_thread: Optional[threading.Thread] = None
        if async_write:
            self._write_q = queue.SimpleQueue()
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name='moteus-csv-writer',
                daemon=True,
            )
            self._writer_thread.start()

    def on_status_update(self, status: Dict[int, ControllerStatus]) -> None:
        """Listener callback — called from the asyncio thread."""
        now = time.perf_counter() - self._t0
        rows = []
        for cid, s in status.items():
            if self._filter_ids is not None and cid not in self._filter_ids:
                continue
            row_data: Dict[str, Any] = {
                'timestamp': f'{now:.6f}',
                'id': cid,
                'mode': s.mode,
                'position': s.position,
                'velocity': s.velocity,
                'torque': s.torque,
                'voltage': s.voltage,
                'temperature': s.temperature,
                'fault': s.fault,
                'trajectory_complete': s.trajectory_complete,
                'abs_position': s.abs_position,
                'encoder_0_position': s.encoder_0_position,
                'encoder_0_velocity': s.encoder_0_velocity,
                'encoder_1_position': s.encoder_1_position,
                'encoder_1_velocity': s.encoder_1_velocity,
                'encoder_2_position': s.encoder_2_position,
                'encoder_2_velocity': s.encoder_2_velocity,
                'encoder_validity': s.encoder_validity,
            }
            rows.append({k: row_data[k] for k in self._fields if k in row_data})

        if not rows:
            return
        if self._write_q is not None:
            self._write_q.put(rows)
            with self._lock:
                self._row_count += len(rows)
        else:
            with self._lock:
                self._writer.writerows(rows)
                self._row_count += len(rows)

    def _writer_loop(self) -> None:
        assert self._write_q is not None
        while True:
            item = self._write_q.get()
            if item is None:
                break
            with self._lock:
                self._writer.writerows(item)

    @property
    def row_count(self) -> int:
        """Total rows written so far (thread-safe)."""
        with self._lock:
            return self._row_count

    def flush(self) -> None:
        with self._lock:
            self._file.flush()

    def close(self) -> None:
        if self._write_q is not None:
            self._write_q.put(None)
            if self._writer_thread is not None:
                self._writer_thread.join(timeout=5.0)
            self._write_q = None
            self._writer_thread = None
        with self._lock:
            self._file.flush()
            self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
