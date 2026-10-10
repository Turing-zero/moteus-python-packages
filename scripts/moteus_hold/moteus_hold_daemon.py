#!/usr/bin/env python3
# Copyright 2025 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""moteus 位置保持守护进程

开机后等待 CAN 接口（默认 can1）出现，周期性地向 ID 1/2/3 发送位置保持指令，
并通过 Unix socket 提供调试接口（见 moteus_holdctl.py）。

行为要点
--------
- 电机首次应答（或掉线后重新应答）时，以其**当前位置**作为保持目标，上电不跳变
- 所有在线电机在同一个 ``transport.cycle()`` 里下发，一轮 CAN 往返完成
- 某个电机不应答时逐个探测，只把不应答的标记为离线，不影响其余电机
- 离线电机按 ``--probe-interval`` 轮询探测（只发 query，不发运动指令）
- 电机进入 TIMEOUT（看门狗超时）时先发一次 stop 清除，再保持在当前位置
- 电机进入 FAULT 时停止发送位置指令，需用 ``clear`` 手动清除
- 总线长时间无应答（例如发送队列卡死）时，重建 transport 并重新配置接口
- 收到 SIGTERM/SIGINT 时默认向在线电机发送 stop（``--on-exit none`` 则交给
  moteus 自身的看门狗处理）

Usage::

    python3 scripts/moteus_hold/moteus_hold_daemon.py --channel can1 --ids 1,2,3
    python3 scripts/moteus_hold/moteus_hold_daemon.py --sim 1,2,3   # 无硬件模拟
"""

import argparse
import asyncio
import dataclasses
import json
import logging
import math
import os
import signal
import sys
import time
from typing import Any, Dict, List, Optional

# 在仓库内运行时优先使用仓库里的 moteus；单独拷贝部署时使用 pip 安装的 moteus
_REPO_MOTEUS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'moteus')
if os.path.isfile(os.path.join(_REPO_MOTEUS, 'moteus', '__init__.py')):
    sys.path.insert(0, _REPO_MOTEUS)

import moteus
from moteus import multiplex as mp
from moteus.protocol import Mode, Register


log = logging.getLogger('moteus_hold')

DEFAULT_SOCKETS = ['/run/moteus-hold/ctl.sock', '/tmp/moteus-hold.sock']

# 单个电机探测的应答超时
_PROBE_TIMEOUT_S = 0.03


def mode_name(mode: Optional[int]) -> str:
    if mode is None:
        return '---'
    try:
        return Mode(mode).name
    except ValueError:
        return str(mode)


def default_socket_path() -> str:
    """守护进程默认 socket：优先 systemd RuntimeDirectory，否则 /tmp。"""
    d = os.path.dirname(DEFAULT_SOCKETS[0])
    if os.path.isdir(d) and os.access(d, os.W_OK):
        return DEFAULT_SOCKETS[0]
    return DEFAULT_SOCKETS[1]


def make_query_resolution() -> moteus.QueryResolution:
    qr = moteus.QueryResolution()
    qr.q_current = mp.F32
    qr.d_current = mp.F32
    qr.trajectory_complete = mp.INT8
    return qr


# ── 运行时参数（可通过调试接口修改） ──────────────────────────────────────────

@dataclasses.dataclass
class Params:
    velocity_limit: float = 0.5      # rev/s，moteus 板上轨迹限速
    accel_limit: float = 2.0         # rev/s^2
    maximum_torque: Optional[float] = None   # Nm，None 用控制器配置
    kp_scale: Optional[float] = None
    kd_scale: Optional[float] = None
    watchdog_timeout: float = 0.25   # s，守护进程停止发送后多久进入 TIMEOUT
    max_step: float = 0.25           # rev，单次 step 允许的最大增量
    max_move: float = 1.0            # rev，目标与当前位置允许的最大偏差

    # name -> (是否允许 None, 是否必须为正)
    _SPEC = {
        'velocity_limit': (False, True),
        'accel_limit': (False, True),
        'maximum_torque': (True, True),
        'kp_scale': (True, False),
        'kd_scale': (True, False),
        'watchdog_timeout': (False, True),
        'max_step': (False, True),
        'max_move': (False, True),
    }

    def set(self, name: str, value: Any) -> None:
        if name not in self._SPEC:
            raise ValueError(
                f'未知参数 {name!r}，可用: {", ".join(self._SPEC)}')
        nullable, positive = self._SPEC[name]
        if value is None or (isinstance(value, str) and
                             value.lower() in ('none', 'null', '')):
            if not nullable:
                raise ValueError(f'{name} 不能为空')
            setattr(self, name, None)
            return
        v = float(value)
        if not math.isfinite(v) or (positive and v <= 0):
            raise ValueError(f'{name} 必须是{"正" if positive else "有限"}数')
        setattr(self, name, v)

    def to_dict(self) -> Dict[str, Any]:
        return {k: getattr(self, k) for k in self._SPEC}


# ── 单个电机状态 ──────────────────────────────────────────────────────────────

@dataclasses.dataclass
class Motor:
    id: int
    lo: float = -math.inf
    hi: float = math.inf

    online: bool = False
    holding: bool = False        # True: 每周期下发位置指令
    released: bool = False       # 用户手动 stop 过，重新上线时不自动保持
    target: Optional[float] = None
    pending_stop: bool = False   # 下一周期先发一次 stop（清 TIMEOUT/FAULT）
    hold_after_stop: bool = False
    settle_until: float = 0.0    # stop 后等待模式离开 FAULT/TIMEOUT 的截止时间

    values: Dict[int, Any] = dataclasses.field(default_factory=dict)
    last_update: float = 0.0     # time.monotonic()
    misses: int = 0
    last_mode: Optional[int] = None
    last_fault: int = 0

    @property
    def mode(self) -> Optional[int]:
        v = self.values.get(Register.MODE)
        return None if v is None else int(v)

    @property
    def position(self) -> float:
        return float(self.values.get(Register.POSITION, math.nan))

    def to_dict(self, now: float) -> Dict[str, Any]:
        v = self.values

        def f(reg):
            x = v.get(reg)
            return None if x is None else float(x)

        return {
            'id': self.id,
            'online': self.online,
            'holding': self.holding,
            'released': self.released,
            'target': self.target,
            'lo': None if math.isinf(self.lo) else self.lo,
            'hi': None if math.isinf(self.hi) else self.hi,
            'mode': self.mode,
            'mode_name': mode_name(self.mode),
            'position': f(Register.POSITION),
            'velocity': f(Register.VELOCITY),
            'torque': f(Register.TORQUE),
            'q_current': f(Register.Q_CURRENT),
            'd_current': f(Register.D_CURRENT),
            'voltage': f(Register.VOLTAGE),
            'temperature': f(Register.TEMPERATURE),
            'fault': int(v.get(Register.FAULT, 0) or 0),
            'trajectory_complete': int(v.get(Register.TRAJECTORY_COMPLETE, 0) or 0),
            'age_ms': (now - self.last_update) * 1e3 if self.last_update else None,
        }


class CommandError(Exception):
    pass


# ── 守护进程 ──────────────────────────────────────────────────────────────────

class HoldDaemon:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.params = Params(
            velocity_limit=args.velocity_limit,
            accel_limit=args.accel_limit,
            maximum_torque=args.max_torque,
            watchdog_timeout=args.watchdog_timeout,
            max_step=args.max_step,
            max_move=args.max_move,
        )
        self.motors: Dict[int, Motor] = {i: Motor(i) for i in args.ids}
        for spec in args.limit or []:
            mid, lo, hi = _parse_limit(spec)
            if mid not in self.motors:
                raise SystemExit(f'--limit 指定的 ID {mid} 不在 --ids 中')
            self.motors[mid].lo, self.motors[mid].hi = lo, hi

        self.qr = make_query_resolution()
        self.transport = None
        self.controllers: Dict[int, moteus.Controller] = {}

        self.period = 1.0 / args.rate
        self.stop_event = asyncio.Event()
        self.start_time = time.monotonic()

        # 统计
        self.cycle_ms = 0.0
        self.loop_hz = 0.0
        self.timeouts = 0
        self.resets = 0
        self.last_rx = 0.0           # 最近一次收到任意应答
        self.last_reset = 0.0
        self.rx_since_open = False
        self.send_error_since_open = False
        self.reset_backoff = 0
        self.iface_missing_logged = False
        self._probe_rr = 0
        self._last_probe = 0.0

    # ── transport ────────────────────────────────────────────────────────────

    def _iface_present(self) -> bool:
        if self.args.sim:
            return True
        return os.path.exists(f'/sys/class/net/{self.args.channel}')

    def _open_transport(self) -> None:
        if self.args.sim:
            from moteus.simulator import SimulatedTransport
            self.transport = SimulatedTransport(self.args.sim)
        else:
            kwargs: Dict[str, Any] = {
                'channel': self.args.channel,
                'interface': 'socketcan',
                'restart_ms': self.args.restart_ms,
            }
            if self.args.disable_brs:
                kwargs['disable_brs'] = True
            if self.args.bitrate:
                kwargs['bitrate'] = self.args.bitrate
            if self.args.dbitrate:
                kwargs['data_bitrate'] = self.args.dbitrate
            device = moteus.PythonCanDevice(**kwargs)
            self.transport = moteus.Transport([device])

        self.controllers = {
            mid: moteus.Controller(id=mid, transport=self.transport,
                                   query_resolution=self.qr)
            for mid in self.motors
        }
        self.last_rx = time.monotonic()
        self.rx_since_open = False
        self.send_error_since_open = False
        log.info('已打开 %s', 'simulator' if self.args.sim
                 else f'socketcan {self.args.channel}')

    def _close_transport(self) -> None:
        if self.transport is not None:
            try:
                self.transport.close()
            except Exception:
                pass
        self.transport = None
        self.controllers = {}

    def _set_offline(self, m: Motor, reason: str) -> None:
        if m.online:
            log.warning('电机 %d 离线（%s）', m.id, reason)
        m.online = False
        # 丢弃旧目标：重新上线时以届时的实际位置为准，避免断电重启后
        # 第一帧就按旧目标跳变。离线期间探测只发 query。
        m.holding = False
        m.target = None
        m.hold_after_stop = False
        m.last_mode = None

    def _mark_all_offline(self, reason: str) -> None:
        for m in self.motors.values():
            self._set_offline(m, reason)

    # ── 指令构造 ─────────────────────────────────────────────────────────────

    def _make_command(self, m: Motor):
        c = self.controllers[m.id]
        if m.pending_stop:
            return c.make_stop(query=True)
        if m.holding and m.target is not None:
            p = self.params
            kwargs: Dict[str, Any] = dict(
                position=m.target,
                velocity=0.0,
                velocity_limit=p.velocity_limit,
                accel_limit=p.accel_limit,
                watchdog_timeout=p.watchdog_timeout,
                query=True,
            )
            if p.maximum_torque is not None:
                kwargs['maximum_torque'] = p.maximum_torque
            if p.kp_scale is not None:
                kwargs['kp_scale'] = p.kp_scale
            if p.kd_scale is not None:
                kwargs['kd_scale'] = p.kd_scale
            return c.make_position(**kwargs)
        return c.make_query()

    def _after_send(self, m: Motor) -> None:
        """一条指令成功送达后的一次性状态推进。"""
        if m.pending_stop:
            m.pending_stop = False
            if m.hold_after_stop:
                m.hold_after_stop = False
                m.holding = True
                m.target = None   # 等模式离开 FAULT/TIMEOUT 后再以当前位置为目标
                m.settle_until = time.monotonic() + 0.2

    # ── 应答处理 ─────────────────────────────────────────────────────────────

    def _apply_result(self, m: Motor, values: Dict[int, Any], now: float) -> None:
        m.values = dict(values)
        m.last_update = now
        m.misses = 0
        self.last_rx = now
        self.rx_since_open = True
        self.reset_backoff = 0

        mode = m.mode
        pos = m.position
        fault = int(values.get(Register.FAULT, 0) or 0)

        if not m.online:
            m.online = True
            log.info('电机 %d 上线：mode=%s pos=%.4f fault=%d',
                     m.id, mode_name(mode), pos, fault)
            if not m.released:
                if mode == Mode.FAULT:
                    m.holding = False
                    log.warning('电机 %d 处于 FAULT(%d)，不自动保持，'
                                '请检查后执行 clear', m.id, fault)
                elif mode == Mode.TIMEOUT:
                    self._request_stop_then_hold(m)
                    log.info('电机 %d 处于 TIMEOUT，先发 stop 清除再保持', m.id)
                else:
                    m.holding = True
                    m.target = None

        if mode != m.last_mode:
            if m.last_mode is not None:
                log.info('电机 %d 模式 %s -> %s', m.id,
                         mode_name(m.last_mode), mode_name(mode))
            m.last_mode = mode

        if fault != m.last_fault:
            if fault:
                log.warning('电机 %d 报告故障码 %d', m.id, fault)
            m.last_fault = fault

        # 只对已经在按目标保持的电机做故障/超时判断；刚发完 stop 的那一帧
        # 应答可能仍显示旧模式，由下面的 settle 逻辑处理
        if m.holding and m.target is not None and not m.pending_stop:
            if mode == Mode.FAULT:
                m.holding = False
                m.target = None
                log.warning('电机 %d 进入 FAULT(%d)，停止保持，需 clear', m.id, fault)
            elif mode == Mode.TIMEOUT:
                log.warning('电机 %d 看门狗超时，stop 后在当前位置重新保持', m.id)
                self._request_stop_then_hold(m)

        # 目标未定时取当前位置（上电/重新保持时不跳变）
        if m.holding and m.target is None and not m.pending_stop:
            if mode not in (Mode.FAULT, Mode.TIMEOUT) and math.isfinite(pos):
                m.target = pos
                log.info('电机 %d 保持在 %.4f rev', m.id, pos)
            elif now > m.settle_until:
                m.holding = False
                log.warning('电机 %d stop 后仍处于 %s(fault=%d)，放弃保持',
                            m.id, mode_name(mode), fault)

    def _request_stop_then_hold(self, m: Motor) -> None:
        m.pending_stop = True
        m.hold_after_stop = True
        m.holding = False
        m.target = None

    def _results_by_id(self, results) -> Dict[int, Dict[int, Any]]:
        out: Dict[int, Dict[int, Any]] = {}
        for r in results or []:
            rid = getattr(r, 'id', None)
            vals = getattr(r, 'values', None)
            if isinstance(rid, int) and vals:
                out[rid] = vals
        return out

    async def _cycle(self, motors: List[Motor], timeout: float):
        cmds = [self._make_command(m) for m in motors]
        results = await asyncio.wait_for(self.transport.cycle(cmds), timeout)
        for m in motors:
            self._after_send(m)
        return self._results_by_id(results)

    async def _probe_one(self, m: Motor, now: float) -> bool:
        try:
            got = await self._cycle([m], _PROBE_TIMEOUT_S)
        except asyncio.TimeoutError:
            return False
        except Exception as e:
            self.send_error_since_open = True
            log.debug('探测电机 %d 失败: %s', m.id, e)
            return False
        if m.id in got:
            self._apply_result(m, got[m.id], time.monotonic())
            return True
        return False

    # ── 主循环 ───────────────────────────────────────────────────────────────

    async def control_loop(self) -> None:
        next_t = time.monotonic()
        last_t = next_t
        while not self.stop_event.is_set():
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            if dt > 0:
                self.loop_hz += (1.0 / dt - self.loop_hz) * 0.05

            try:
                await self._step(now)
            except Exception:
                log.exception('控制循环异常，重建 transport')
                self._mark_all_offline('异常')
                self._close_transport()
                await asyncio.sleep(1.0)

            next_t += self.period
            delay = next_t - time.monotonic()
            if delay < -self.period:
                next_t = time.monotonic()   # 落后太多，重新对齐
                delay = 0
            try:
                await asyncio.wait_for(self.stop_event.wait(), max(0.0, delay))
            except asyncio.TimeoutError:
                pass

    async def _step(self, now: float) -> None:
        if not self._iface_present():
            if not self.iface_missing_logged:
                log.warning('等待 CAN 接口 %s 出现...', self.args.channel)
                self.iface_missing_logged = True
            self._mark_all_offline('接口不存在')
            self._close_transport()
            await asyncio.sleep(0.5)
            return
        if self.iface_missing_logged:
            log.info('CAN 接口 %s 已出现', self.args.channel)
            self.iface_missing_logged = False

        if self.transport is None:
            try:
                self._open_transport()
            except Exception as e:
                log.error('打开 %s 失败: %s', self.args.channel, e)
                self._close_transport()
                await asyncio.sleep(2.0)
                return

        online = [m for m in self.motors.values() if m.online]
        if online:
            t0 = time.perf_counter()
            try:
                got = await self._cycle(online, self.args.cycle_timeout)
                self.cycle_ms += ((time.perf_counter() - t0) * 1e3
                                  - self.cycle_ms) * 0.05
                t = time.monotonic()
                for m in online:
                    if m.id in got:
                        self._apply_result(m, got[m.id], t)
            except (asyncio.TimeoutError, Exception) as e:
                self.timeouts += 1
                if not isinstance(e, asyncio.TimeoutError):
                    self.send_error_since_open = True
                    log.debug('cycle 异常: %s', e)
                # 逐个探测，找出不应答的电机
                for m in online:
                    if not await self._probe_one(m, now):
                        m.misses += 1
                        if m.misses >= self.args.offline_after:
                            self._set_offline(
                                m, f'连续 {m.misses} 次无应答')

        # 探测离线电机（只 query，或执行 pending stop）。有电机在线时每次只探测
        # 一个，避免拉长在线电机的控制周期；全部离线时一次探测所有。
        offline = [m for m in self.motors.values() if not m.online]
        if offline and now - self._last_probe >= self.args.probe_interval:
            self._last_probe = now
            if online:
                batch = [offline[self._probe_rr % len(offline)]]
                self._probe_rr += 1
            else:
                batch = offline
            for m in batch:
                await self._probe_one(m, now)

        # 总线长时间没有任何应答：重建 transport（会重新 down/up 接口，
        # 清掉卡死的发送队列）。只在本次打开后收到过应答、或出现发送错误时
        # 才重建，电机全部未上电时不反复重配接口；连续重建按指数退避。
        interval = min(self.args.reset_after * (2 ** self.reset_backoff), 60.0)
        if (not self.args.sim and
                (self.rx_since_open or self.send_error_since_open) and
                now - self.last_rx > interval and
                now - self.last_reset > interval):
            self.last_reset = now
            self.resets += 1
            self.reset_backoff += 1
            log.warning('%.1f s 无任何应答%s，重建 transport 并重新配置 %s',
                        now - self.last_rx,
                        '（发送出错）' if self.send_error_since_open else '',
                        self.args.channel)
            self._mark_all_offline('总线无应答')
            self._close_transport()

    async def shutdown(self) -> None:
        online = [m for m in self.motors.values() if m.online]
        if self.args.on_exit == 'stop' and self.transport is not None and online:
            try:
                await asyncio.wait_for(self.transport.cycle([
                    self.controllers[m.id].make_stop(query=False)
                    for m in online]), 0.5)
                log.info('已向电机 %s 发送 stop', [m.id for m in online])
            except Exception as e:
                log.warning('退出时发送 stop 失败: %s', e)
        self._close_transport()

    # ── 调试接口 ─────────────────────────────────────────────────────────────

    def _select(self, ids) -> List[Motor]:
        if ids in (None, 'all', ['all']):
            return list(self.motors.values())
        if not isinstance(ids, list):
            ids = [ids]
        out = []
        for i in ids:
            try:
                out.append(self.motors[int(i)])
            except (KeyError, ValueError):
                raise CommandError(f'未知电机 ID {i!r}，可用: {list(self.motors)}')
        return out

    def _check_movable(self, m: Motor) -> None:
        if not m.online:
            raise CommandError(f'电机 {m.id} 离线')
        if m.mode == Mode.FAULT:
            raise CommandError(f'电机 {m.id} 处于 FAULT，先执行 clear')
        if not m.holding or m.target is None:
            raise CommandError(f'电机 {m.id} 未处于保持状态，先执行 hold')

    def _move_to(self, m: Motor, new_target: float) -> str:
        if not math.isfinite(new_target):
            raise CommandError('目标位置无效')
        clamped = min(m.hi, max(m.lo, new_target))
        pos = m.position
        if math.isfinite(pos) and abs(clamped - pos) > self.params.max_move:
            raise CommandError(
                f'电机 {m.id} 目标 {clamped:.4f} 与当前位置 {pos:.4f} 相差超过 '
                f'max_move={self.params.max_move}')
        m.target = clamped
        note = f'（已按限位钳制，原 {new_target:.4f}）' if clamped != new_target else ''
        return f'电机 {m.id} 目标 -> {clamped:.4f}{note}'

    def handle_request(self, req: Dict[str, Any]) -> Dict[str, Any]:
        cmd = req.get('cmd')
        now = time.monotonic()
        msgs: List[str] = []
        errors: List[str] = []

        def for_each(motors, fn):
            for m in motors:
                try:
                    msgs.append(fn(m))
                except CommandError as e:
                    errors.append(str(e))

        if cmd == 'status':
            pass

        elif cmd == 'step':
            delta = float(req['delta'])
            if abs(delta) > self.params.max_step:
                raise CommandError(
                    f'step {delta} 超过 max_step={self.params.max_step}')

            def do_step(m):
                self._check_movable(m)
                return self._move_to(m, m.target + delta)
            for_each(self._select(req.get('ids')), do_step)

        elif cmd == 'set':
            position = float(req['position'])

            def do_set(m):
                self._check_movable(m)
                return self._move_to(m, position)
            for_each(self._select(req.get('ids')), do_set)

        elif cmd == 'hold':
            def do_hold(m):
                if not m.online:
                    raise CommandError(f'电机 {m.id} 离线')
                if m.mode == Mode.FAULT:
                    raise CommandError(f'电机 {m.id} 处于 FAULT，先执行 clear')
                m.released = False
                if m.mode == Mode.TIMEOUT:
                    self._request_stop_then_hold(m)
                    return f'电机 {m.id} 清除 TIMEOUT 后保持在当前位置'
                m.holding = True
                m.target = m.position if math.isfinite(m.position) else None
                return f'电机 {m.id} 保持在当前位置 {m.position:.4f}'
            for_each(self._select(req.get('ids')), do_hold)

        elif cmd == 'stop':
            def do_stop(m):
                m.released = True
                m.holding = False
                m.target = None
                m.pending_stop = True
                m.hold_after_stop = False
                return f'电机 {m.id} 已释放（stop），用 hold 恢复保持'
            for_each(self._select(req.get('ids')), do_stop)

        elif cmd == 'clear':
            def do_clear(m):
                if not m.online:
                    raise CommandError(f'电机 {m.id} 离线')
                m.released = False
                self._request_stop_then_hold(m)
                return f'电机 {m.id} 发送 stop 清除故障，随后保持在当前位置'
            for_each(self._select(req.get('ids')), do_clear)

        elif cmd == 'param':
            name = req.get('name')
            if name is not None:
                self.params.set(name, req.get('value'))
                msgs.append(f'{name} = {getattr(self.params, name)}')
                log.info('参数 %s = %s', name, getattr(self.params, name))

        elif cmd == 'limit':
            if req.get('id') is None:
                raise CommandError('limit 需要指定单个电机 ID')
            m = self._select(req.get('id'))[0]
            lo = req.get('lo')
            hi = req.get('hi')
            lo = -math.inf if lo is None else float(lo)
            hi = math.inf if hi is None else float(hi)
            if lo > hi:
                raise CommandError('下限大于上限')
            m.lo, m.hi = lo, hi
            msgs.append(f'电机 {m.id} 限位 [{lo}, {hi}]')
            if m.target is not None and not lo <= m.target <= hi:
                msgs.append(f'注意：当前目标 {m.target:.4f} 在限位外，未自动移动')

        else:
            raise CommandError(f'未知命令 {cmd!r}')

        if cmd not in ('status', 'param') or req.get('name') is not None:
            for x in msgs:
                log.info('[ctl] %s', x)
            for x in errors:
                log.warning('[ctl] %s', x)

        return {
            'ok': not errors,
            'msgs': msgs,
            'errors': errors,
            'daemon': {
                'channel': 'sim' if self.args.sim else self.args.channel,
                'rate': self.args.rate,
                'loop_hz': round(self.loop_hz, 1),
                'cycle_ms': round(self.cycle_ms, 2),
                'timeouts': self.timeouts,
                'resets': self.resets,
                'uptime_s': round(now - self.start_time, 1),
                'bus_open': self.transport is not None,
                'params': self.params.to_dict(),
            },
            'motors': [m.to_dict(now) for m in self.motors.values()],
        }

    async def serve_client(self, reader: asyncio.StreamReader,
                           writer: asyncio.StreamWriter) -> None:
        try:
            while not reader.at_eof():
                line = await reader.readline()
                if not line:
                    break
                try:
                    req = json.loads(line)
                    resp = self.handle_request(req)
                except (CommandError, ValueError, KeyError, TypeError) as e:
                    resp = {'ok': False, 'msgs': [], 'errors': [str(e)]}
                writer.write((json.dumps(resp, ensure_ascii=False) + '\n')
                             .encode('utf-8'))
                await writer.drain()
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            writer.close()


def _moteus_version() -> str:
    try:
        from importlib.metadata import version
        return version('moteus')
    except Exception:
        return '?'


def _parse_limit(spec: str):
    try:
        mid, lo, hi = spec.split(':')
        return int(mid), float(lo), float(hi)
    except ValueError:
        raise SystemExit(f'--limit 格式应为 ID:MIN:MAX，得到 {spec!r}')


def _parse_ids(text: str) -> List[int]:
    ids = [int(x) for x in text.split(',') if x.strip()]
    if not ids or not all(1 <= i <= 126 for i in ids):
        raise argparse.ArgumentTypeError('ID 必须是 1..126 的逗号分隔列表')
    return ids


async def amain(args: argparse.Namespace) -> None:
    daemon = HoldDaemon(args)

    sock_path = args.socket or default_socket_path()
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    server = await asyncio.start_unix_server(daemon.serve_client, path=sock_path)
    os.chmod(sock_path, 0o660)
    log.info('调试接口: %s', sock_path)
    log.info('moteus %s (%s)', _moteus_version(), os.path.dirname(moteus.__file__))
    log.info('电机 %s，频率 %.0f Hz，velocity_limit=%.3g accel_limit=%.3g',
             list(daemon.motors), args.rate,
             daemon.params.velocity_limit, daemon.params.accel_limit)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, daemon.stop_event.set)

    try:
        await daemon.control_loop()
    finally:
        log.info('退出中...')
        server.close()
        await daemon.shutdown()
        try:
            os.unlink(sock_path)
        except OSError:
            pass


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='moteus 位置保持守护进程（含 Unix socket 调试接口）')
    p.add_argument('--channel', default='can1', help='socketcan 接口（默认 can1）')
    p.add_argument('--ids', type=_parse_ids, default=[1, 2, 3],
                   help='电机 ID，逗号分隔（默认 1,2,3）')
    p.add_argument('--rate', type=float, default=100.0, help='控制频率 Hz（默认 100）')
    p.add_argument('--socket', default=None,
                   help=f'调试接口 socket 路径（默认 {DEFAULT_SOCKETS[0]}，'
                        f'目录不可写时 {DEFAULT_SOCKETS[1]}）')

    g = p.add_argument_group('CAN 接口')
    g.add_argument('--disable-brs', action='store_true',
                   help='发送不带 BRS 的帧（整条总线跑 1 Mbps）')
    g.add_argument('--bitrate', type=int, default=None,
                   help='仲裁段波特率（默认由库配置为 1M）')
    g.add_argument('--dbitrate', type=int, default=None,
                   help='数据段波特率（默认由库配置为 5M）')
    g.add_argument('--restart-ms', type=int, default=100,
                   help='bus-off 自动恢复时间 ms（默认 100）')

    g = p.add_argument_group('运动参数（运行时可用 holdctl param 修改）')
    g.add_argument('--velocity-limit', type=float, default=0.5, help='rev/s（默认 0.5）')
    g.add_argument('--accel-limit', type=float, default=2.0, help='rev/s^2（默认 2）')
    g.add_argument('--max-torque', type=float, default=None, help='Nm（默认用控制器配置）')
    g.add_argument('--watchdog-timeout', type=float, default=0.25,
                   help='moteus 看门狗 s（默认 0.25）')
    g.add_argument('--max-step', type=float, default=0.25,
                   help='单次 step 最大增量 rev（默认 0.25）')
    g.add_argument('--max-move', type=float, default=1.0,
                   help='目标与当前位置最大偏差 rev（默认 1.0）')
    g.add_argument('--limit', action='append', metavar='ID:MIN:MAX',
                   help='软件限位 rev，可重复，例如 --limit 1:-0.5:0.5')

    g = p.add_argument_group('容错')
    g.add_argument('--cycle-timeout', type=float, default=0.02,
                   help='一轮 cycle 的应答超时 s（默认 0.02）')
    g.add_argument('--offline-after', type=int, default=5,
                   help='连续多少次无应答判为离线（默认 5）')
    g.add_argument('--probe-interval', type=float, default=0.5,
                   help='离线电机探测间隔 s（默认 0.5）')
    g.add_argument('--reset-after', type=float, default=3.0,
                   help='总线无任何应答多久后重建 transport s（默认 3）')
    g.add_argument('--on-exit', choices=['stop', 'none'], default='stop',
                   help='退出时向在线电机发 stop（默认），或交给看门狗')

    p.add_argument('--sim', type=_parse_ids, default=None, metavar='IDS',
                   help='使用内置模拟器代替 CAN，参数为模拟存在的 ID，如 1,3')
    p.add_argument('--log-level', default='INFO')
    return p


def main() -> None:
    args = build_parser().parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format='%(asctime)s %(levelname)s %(message)s')
    # python-can 打开/关闭接口时的日志较多
    logging.getLogger('can').setLevel(logging.WARNING)

    asyncio.run(amain(args))


if __name__ == '__main__':
    main()
