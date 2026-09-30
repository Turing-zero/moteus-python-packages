# Copyright 2025 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""临时多电机调试 GUI

一条 CAN 总线挂多个 moteus，逐个使能、拖动滑条调位置：

- 自动探测：按 ID 区间逐个发 query，有应答的即在线（也可手动填 ID）
- 每个电机可设位置上下限（rev），滑条、目标输入框与指令路径多重钳制，
  超过限位的目标值不会下发
- 全局速度/加速度上限（默认较小），修改后立即作用于已使能电机
- 使能时先读当前位置，位置超出限位则拒绝使能，避免上电跳变
- 故障或反馈超时自动停止该电机；可随时停止全部

Usage::

    python3 scripts/multi_motor_gui.py
    python3 scripts/multi_motor_gui.py --can-type fdcanusb --can-chan /dev/ttyUSB0
    python3 scripts/multi_motor_gui.py --can-type socketcan --can-chan can0 --ids 1,2,3
"""

import argparse
import asyncio
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'moteus'))

import moteus
from moteus import ControlManager, ControllerStatus
from moteus.control_manager import ManagerState
from moteus.protocol import Mode


# ── Transport / scanning helpers ─────────────────────────────────────────────

async def open_transport(
    can_type: str,
    can_iface: Optional[str],
    can_chan: Optional[str],
    disable_brs: bool,
):
    """Build a Transport for the given backend (shared by control + scan)."""
    if can_type == 'fdcanusb':
        device = moteus.FdcanusbDevice(can_chan, disable_brs=disable_brs)
    elif can_type == 'candle':
        device = moteus.CandleDevice(
            channel_index=int(can_chan or '0'), disable_brs=disable_brs)
    else:
        kwargs = {}
        if can_chan:
            kwargs['channel'] = can_chan
        if can_iface:
            kwargs['interface'] = can_iface
        if disable_brs:
            kwargs['disable_brs'] = True
        device = moteus.PythonCanDevice(**kwargs)
    return moteus.Transport([device])


class ManagedControlManager(ControlManager):
    """ControlManager with a final stop frame before the CAN bus is closed."""

    async def _build_transport(self, can_type, can_iface, can_chan,
                               can_disable_brs):
        return await open_transport(
            can_type, can_iface, can_chan, can_disable_brs)

    async def _control_loop(self, transport, controllers):
        try:
            await super()._control_loop(transport, controllers)
        finally:
            if controllers:
                try:
                    await asyncio.wait_for(transport.cycle([
                        ctrl.make_stop(query=False)
                        for ctrl in controllers.values()
                    ]), timeout=0.5)
                except Exception:
                    pass  # A disconnected bus cannot acknowledge a stop.


_SCAN_TIMEOUT_S = 0.1


async def _scan_ids_async(can_type, can_iface, can_chan, lo, hi,
                          disable_brs=False) -> List[int]:
    """Query IDs individually; a silent ID cannot hide a responding neighbor."""
    if not 1 <= lo <= hi <= 126:
        raise ValueError('扫描 ID 必须在 1–126 内')
    transport = await open_transport(can_type, can_iface, can_chan, disable_brs)
    found: List[int] = []
    try:
        for cid in range(lo, hi + 1):
            cmd = moteus.Controller(id=cid, transport=transport).make_query()
            try:
                results = await asyncio.wait_for(
                    transport.cycle([cmd]), timeout=_SCAN_TIMEOUT_S)
            except asyncio.TimeoutError:
                continue
            for r in results:
                if getattr(r, 'id', None) == cid:
                    found.append(cid)
                    break
    finally:
        transport.close()
    return found


def scan_ids(can_type, can_iface, can_chan, lo, hi,
             disable_brs=False) -> List[int]:
    """Blocking scan (intended to run on a worker thread)."""
    return asyncio.run(
        _scan_ids_async(can_type, can_iface, can_chan, lo, hi, disable_brs))


# ── Small helpers ────────────────────────────────────────────────────────────

def _mode_name(mode_int: int) -> str:
    try:
        return Mode(mode_int).name
    except (ValueError, KeyError):
        return str(mode_int)


def _fmt(v: float, decimals: int = 3) -> str:
    if math.isnan(v) or math.isinf(v):
        return '---'
    return f'{v:+.{decimals}f}'


def _parse_float(s: str) -> Optional[float]:
    try:
        value = float(s.strip())
        return value if math.isfinite(value) else None
    except (ValueError, AttributeError):
        return None


# ── Per-motor row ────────────────────────────────────────────────────────────

class MotorRow:
    """One motor: enable checkbox + slider + limits + live status labels."""

    def __init__(self, app: 'MultiMotorApp', parent: ttk.Frame,
                 cid: int, grid_row: int):
        self.app = app
        self.id = cid
        self.target = 0.0
        remembered = app.remembered_limits.get(cid)
        self.lo, self.hi = remembered if remembered else (-1.0, 1.0)
        self.dirty = False
        self._sync = False          # guard against slider/entry feedback loops

        self.enabled_var = tk.BooleanVar(value=False)
        self.min_var = tk.StringVar(value=f'{self.lo:g}')
        self.max_var = tk.StringVar(value=f'{self.hi:g}')
        self.tgt_var = tk.StringVar(value='0.000')

        def w(col, widget, **kw):
            widget.grid(row=grid_row, column=col, **kw)
            return widget

        r = self
        w(0, ttk.Checkbutton(
            parent, text='使能', variable=self.enabled_var,
            command=lambda: r.app._on_enable_toggled(r)), padx=(6, 2), pady=3)
        w(1, ttk.Label(parent, text=f'ID {cid}', font=('', 10, 'bold')),
          padx=(0, 6))
        self.slider = ttk.Scale(
            parent, from_=self.lo, to=self.hi, length=260,
            command=lambda v: r.app._on_slider_moved(r, v))
        w(2, self.slider, padx=4, sticky='ew')
        self.tgt_entry = w(3, ttk.Entry(
            parent, textvariable=self.tgt_var, width=8, justify='center'),
            padx=2)
        self.pos_lbl = w(4, ttk.Label(parent, text='---', width=19,
                                      anchor='w'), padx=2)
        self.mode_lbl = w(5, ttk.Label(parent, text='---', width=10,
                                       anchor='w'), padx=2)
        w(6, ttk.Label(parent, text='min'), padx=(8, 0))
        self.min_entry = w(7, ttk.Entry(
            parent, textvariable=self.min_var, width=7, justify='center'),
            padx=2)
        w(8, ttk.Label(parent, text='max'), padx=(8, 0))
        self.max_entry = w(9, ttk.Entry(
            parent, textvariable=self.max_var, width=7, justify='center'),
            padx=2)
        w(10, ttk.Button(parent, text='停止', width=5,
                         command=lambda: r.app._on_stop_one(r)), padx=(4, 6))

        self.tgt_entry.bind('<Return>', lambda _: r.app._on_tgt_entry(r))
        self.tgt_entry.bind('<FocusOut>', lambda _: r.app._on_tgt_entry(r))
        self.min_var.trace_add('write', lambda *_: r.app._on_limits_changed(r))
        self.max_var.trace_add('write', lambda *_: r.app._on_limits_changed(r))

    # -- limit handling -------------------------------------------------------

    def limits(self) -> Tuple[float, float]:
        return self.lo, self.hi

    def _parse_limits(self) -> Optional[Tuple[float, float]]:
        lo = _parse_float(self.min_var.get())
        hi = _parse_float(self.max_var.get())
        if lo is None or hi is None or not (lo < hi):
            return None
        return lo, hi

    # -- target helpers -------------------------------------------------------

    def set_target(self, value: float) -> None:
        """Programmatically set target + sync slider/entry (no command)."""
        self._sync = True
        try:
            self.target = value
            self.tgt_var.set(f'{value:.6f}')
            self.slider.set(value)
        finally:
            self._sync = False

    def set_limits(self, lo: float, hi: float) -> None:
        self._sync = True
        try:
            self.lo, self.hi = lo, hi
            self.slider.configure(from_=lo, to=hi)
        finally:
            self._sync = False
        self.set_target(min(max(self.target, lo), hi))

    def status_text(self, s: Optional[ControllerStatus]) -> None:
        if s is None:
            self.pos_lbl.config(text='---')
            self.mode_lbl.config(text='---', foreground='gray')
            return
        self.pos_lbl.config(
            text=f'{_fmt(s.position, 4)}  {_fmt(s.velocity)}r/s')
        mode = _mode_name(s.mode)
        if s.fault != 0:
            self.mode_lbl.config(text=f'FAULT({s.fault})', foreground='red')
        elif s.mode == int(Mode.STOPPED):
            self.mode_lbl.config(text=mode, foreground='gray')
        else:
            self.mode_lbl.config(text=mode, foreground='black')


# ── Main application ─────────────────────────────────────────────────────────

class MultiMotorApp:
    """多电机调试 GUI.

    Thread model (same as fastgui):
    - Main thread: all tkinter operations
    - asyncio background thread: CAN I/O via ControlManager (cycle_hz)
    - queue.Queue + root.after(50 ms): status / scan results → tkinter
    """

    _POLL_MS = 50

    def __init__(
        self,
        default_can_type: str = 'socketcan',
        default_can_chan: str = 'can0',
        default_ids: str = '1',
        default_vel_limit: float = 0.5,
        default_accel_limit: float = 2.0,
        cycle_hz: float = 50.0,
        can_disable_brs: bool = False,
    ):
        self.manager = ManagedControlManager(
            cycle_hz=cycle_hz, query_profile='lite')
        self._status_q: queue.Queue = queue.Queue(maxsize=200)
        self._scan_q: queue.Queue = queue.Queue()
        self._rows: Dict[int, MotorRow] = {}
        self.remembered_limits: Dict[int, Tuple[float, float]] = {}
        self._prev_state = ManagerState.DISCONNECTED
        self._can_disable_brs = can_disable_brs
        self._closing = False
        self._connected_at = 0.0

        self.root = tk.Tk()
        self.root.title('moteus 多电机调试 GUI')
        self.root.minsize(1050, 620)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=3)
        self.root.rowconfigure(3, weight=1)

        # ── StringVars ────────────────────────────────────────────────────
        self.var_can_type = tk.StringVar(value=default_can_type)
        self.var_chan = tk.StringVar(value=default_can_chan)
        self.var_ids = tk.StringVar(value=default_ids)
        self.var_scan_lo = tk.StringVar(value='1')
        self.var_scan_hi = tk.StringVar(value='20')
        self.var_vel_limit = tk.StringVar(value=f'{default_vel_limit:g}')
        self.var_accel_limit = tk.StringVar(value=f'{default_accel_limit:g}')
        self.var_torque = tk.StringVar(value='')   # empty = controller default
        self.var_status_lbl = tk.StringVar(value='未连接')

        self._build_ui()
        self._register_listener()
        for var in (self.var_vel_limit, self.var_accel_limit, self.var_torque):
            var.trace_add('write', lambda *_: self._on_global_changed())
        self.root.after(self._POLL_MS, self._poll_ui)

    def run(self) -> None:
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.mainloop()

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self) -> None:
        pad = {'padx': 6, 'pady': 4}

        top = ttk.Frame(self.root)
        top.grid(row=0, column=0, sticky='ew', **pad)
        self._build_connect_frame(top)
        self._build_global_frame(top)

        bar = ttk.Frame(self.root)
        bar.grid(row=1, column=0, sticky='ew', padx=6)
        self._dot = ttk.Label(bar, text='●', foreground='gray', font=('', 13))
        self._dot.pack(side='left', padx=4)
        ttk.Label(bar, textvariable=self.var_status_lbl).pack(side='left')
        self._stop_all_btn = tk.Button(
            bar, text='停止全部', command=self._on_stop_all,
            bg='#c0392b', fg='white', activebackground='#e74c3c',
            activeforeground='white', relief='raised', padx=10)
        self._stop_all_btn.pack(side='right', padx=6)

        self._build_motor_frame(self.root)
        self._build_log_frame(self.root)

    def _build_connect_frame(self, parent: ttk.Frame) -> None:
        frm = ttk.LabelFrame(parent, text='CAN 连接')
        frm.pack(side='left', padx=(0, 6), anchor='n')

        r = 0
        ttk.Label(frm, text='接口:').grid(row=r, column=0, sticky='e', padx=6, pady=3)
        cb = ttk.Combobox(frm, textvariable=self.var_can_type, width=11,
                          values=['socketcan', 'fdcanusb', 'candle'],
                          state='readonly')
        cb.grid(row=r, column=1, columnspan=2, sticky='w', padx=4, pady=3)
        cb.bind('<<ComboboxSelected>>', self._on_iface_changed)

        r += 1
        ttk.Label(frm, text='通道:').grid(row=r, column=0, sticky='e', padx=6, pady=3)
        ttk.Entry(frm, textvariable=self.var_chan, width=14).grid(
            row=r, column=1, columnspan=2, sticky='w', padx=4, pady=3)

        r += 1
        ttk.Label(frm, text='电机 ID:').grid(row=r, column=0, sticky='e', padx=6, pady=3)
        ttk.Entry(frm, textvariable=self.var_ids, width=14).grid(
            row=r, column=1, columnspan=2, sticky='w', padx=4, pady=3)

        r += 1
        ttk.Label(frm, text='探测范围:').grid(row=r, column=0, sticky='e', padx=6, pady=3)
        ttk.Entry(frm, textvariable=self.var_scan_lo, width=4,
                  justify='center').grid(row=r, column=1, sticky='e', padx=(20, 0))
        ttk.Label(frm, text='—').grid(row=r, column=1, sticky='e', padx=(52, 0))
        ttk.Entry(frm, textvariable=self.var_scan_hi, width=4,
                  justify='center').grid(row=r, column=2, sticky='w', padx=(2, 0))

        r += 1
        btn_row = ttk.Frame(frm)
        btn_row.grid(row=r, column=0, columnspan=3, pady=4)
        self.scan_btn = ttk.Button(btn_row, text='自动探测', command=self._on_scan)
        self.scan_btn.pack(side='left', padx=4)
        self.connect_btn = ttk.Button(btn_row, text='连接', command=self._on_connect)
        self.connect_btn.pack(side='left', padx=4)
        self.disconnect_btn = ttk.Button(
            btn_row, text='断开', command=self._on_disconnect, state='disabled')
        self.disconnect_btn.pack(side='left', padx=4)

    def _build_global_frame(self, parent: ttk.Frame) -> None:
        frm = ttk.LabelFrame(parent, text='全局运动参数')
        frm.pack(side='left', fill='x', expand=True, anchor='n')

        r = 0
        ttk.Label(frm, text='速度上限 (r/s):').grid(row=r, column=0, sticky='e',
                                                    padx=6, pady=3)
        self.vel_entry = ttk.Entry(frm, textvariable=self.var_vel_limit, width=8)
        self.vel_entry.grid(row=r, column=1, sticky='w', padx=4, pady=3)

        r += 1
        ttk.Label(frm, text='加速度上限 (r/s²):').grid(row=r, column=0, sticky='e',
                                                      padx=6, pady=3)
        self.accel_entry = ttk.Entry(frm, textvariable=self.var_accel_limit,
                                     width=8)
        self.accel_entry.grid(row=r, column=1, sticky='w', padx=4, pady=3)

        r += 1
        ttk.Label(frm, text='转矩限制 (Nm, 空=默认):').grid(row=r, column=0, sticky='e',
                                                        padx=6, pady=3)
        self.torque_entry = ttk.Entry(frm, textvariable=self.var_torque, width=8)
        self.torque_entry.grid(row=r, column=1, sticky='w', padx=4, pady=3)

        r += 1
        ttk.Label(frm, text='(修改后立即作用于已使能电机)',
                  foreground='gray', font=('', 8)).grid(
            row=r, column=0, columnspan=2, sticky='w', padx=6)

    def _build_motor_frame(self, parent: tk.Tk) -> None:
        frm = ttk.LabelFrame(parent, text='电机列表 (使能后拖动滑条; 也可在目标框输入精确值)')
        frm.grid(row=2, column=0, sticky='nsew', padx=6, pady=4)
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(1, weight=1)

        hdr = ttk.Frame(frm)
        hdr.grid(row=0, column=0, sticky='ew', padx=4)
        for col, text, anchor in [
                (0, '使能', 'w'), (2, '位置 (rev)', 'w'), (3, '目标', 'w'),
                (4, '实际 / 速度', 'w'), (5, '模式', 'w')]:
            ttk.Label(hdr, text=text, foreground='gray').grid(
                row=0, column=col, sticky=anchor, padx=2)

        canvas = tk.Canvas(frm, highlightthickness=0)
        vsb = ttk.Scrollbar(frm, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        canvas.grid(row=1, column=0, sticky='nsew')
        vsb.grid(row=1, column=1, sticky='ns')

        self._motor_inner = ttk.Frame(canvas)
        self._motor_inner.columnconfigure(2, weight=1)  # slider stretches
        self._motor_inner.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=self._motor_inner, anchor='nw',
                             tags='inner')
        canvas.bind(
            '<Configure>',
            lambda e: canvas.itemconfigure('inner', width=e.width))
        self._motor_canvas = canvas

    def _build_log_frame(self, parent: tk.Tk) -> None:
        frm = ttk.LabelFrame(parent, text='日志')
        frm.grid(row=3, column=0, sticky='nsew', padx=6, pady=(0, 6))
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(0, weight=1)
        self.log_text = scrolledtext.ScrolledText(
            frm, height=6, state='disabled', wrap='word', font=('Courier', 9))
        self.log_text.tag_config('error', foreground='red')
        self.log_text.tag_config('warn', foreground='#b86400')
        self.log_text.tag_config('info', foreground='black')
        self.log_text.grid(row=0, column=0, sticky='nsew', padx=2, pady=2)

    # ── Row management ────────────────────────────────────────────────────

    def _build_rows(self, ids: List[int]) -> None:
        for w in self._motor_inner.winfo_children():
            w.destroy()
        self._rows.clear()
        for i, cid in enumerate(ids):
            self._rows[cid] = MotorRow(self, self._motor_inner, cid, i)

    # ── Event handlers: connection ────────────────────────────────────────

    def _on_iface_changed(self, _event=None) -> None:
        defaults = {'socketcan': 'can0', 'fdcanusb': '/dev/ttyUSB0', 'candle': '0'}
        iface = self.var_can_type.get()
        if self.var_chan.get() in defaults.values():
            self.var_chan.set(defaults.get(iface, ''))

    def _parse_ids(self) -> Optional[List[int]]:
        try:
            ids = [int(x.strip()) for x in self.var_ids.get().split(',') if x.strip()]
        except ValueError:
            self._log('电机 ID 格式错误（应为逗号分隔整数）', 'error')
            return None
        if any(not 1 <= i <= 126 for i in ids):
            self._log('电机 ID 必须在 1–126 内（127 为广播地址）', 'error')
            return None
        ids = sorted(set(ids))
        if not ids:
            self._log('没有可用的电机 ID', 'error')
            return None
        return ids

    def _on_connect(self) -> None:
        if str(self.scan_btn.cget('state')) == 'disabled':
            self._log('请等待扫描完成', 'warn')
            return
        if self.manager.get_state() != ManagerState.DISCONNECTED:
            self._log('请先断开当前连接', 'warn')
            return
        ids = self._parse_ids()
        if ids is None:
            return
        self._build_rows(ids)
        self._connected_at = time.monotonic()
        self._log(f'连接中: {self.var_can_type.get()} {self.var_chan.get().strip()} '
                  f'ids={ids}')
        self.manager.connect(
            ids,
            can_type=self.var_can_type.get(),
            can_chan=self.var_chan.get().strip(),
            can_disable_brs=self._can_disable_brs,
        )

    def _on_disconnect(self) -> None:
        self._log('断开中…')
        self._uncheck_all()
        self.manager.disconnect()

    def _on_scan(self) -> None:
        if self.manager.get_state() != ManagerState.DISCONNECTED:
            self._log('已连接时不能扫描，请先断开', 'error')
            return
        try:
            lo, hi = int(self.var_scan_lo.get()), int(self.var_scan_hi.get())
        except ValueError:
            lo, hi = 0, 0
        if lo < 1 or hi > 126 or lo > hi:
            self._log('探测范围无效（1–126，127 为广播地址）', 'error')
            return
        self.scan_btn.config(state='disabled', text='扫描中…')
        self.connect_btn.config(state='disabled')
        self._log(f'扫描 ID {lo}–{hi} …')
        threading.Thread(
            target=self._scan_worker,
            args=(self.var_can_type.get(), self.var_chan.get().strip(),
                  lo, hi),
            daemon=True,
        ).start()

    def _scan_worker(self, can_type, can_chan, lo, hi) -> None:
        try:
            found = scan_ids(can_type, None, can_chan, lo, hi,
                             self._can_disable_brs)
            self._scan_q.put(found)
        except Exception as exc:
            self._scan_q.put(exc)

    # ── Event handlers: motors ────────────────────────────────────────────

    def _clamp(self, row: MotorRow, value: float) -> float:
        lo, hi = row.limits()
        return min(max(value, lo), hi)

    def _on_slider_moved(self, row: MotorRow, value_str: str) -> None:
        if row._sync:
            return
        value = _parse_float(value_str)
        if value is None:
            return
        row.set_target(self._clamp(row, value))
        if row.enabled_var.get():
            row.dirty = True

    def _on_tgt_entry(self, row: MotorRow) -> None:
        if row._sync:
            return
        value = _parse_float(row.tgt_var.get())
        if value is None:
            row.set_target(row.target)
            self._log(f'ID {row.id} 目标位置无效', 'error')
            return
        row.set_target(self._clamp(row, value))
        if row.enabled_var.get():
            row.dirty = True

    def _on_limits_changed(self, row: MotorRow) -> None:
        if row._sync:
            return
        if row.enabled_var.get():
            self._on_stop_one(row)
            self._log(f'ID {row.id} 限位已修改，请重新使能', 'warn')
        parsed = row._parse_limits()
        fg = 'red'
        if parsed is not None:
            fg = 'black'
            row.set_limits(*parsed)
            self.remembered_limits[row.id] = parsed
        row.min_entry.config(foreground=fg)
        row.max_entry.config(foreground=fg)

    def _on_global_changed(self) -> None:
        vel = _parse_float(self.var_vel_limit.get())
        accel = _parse_float(self.var_accel_limit.get())
        torque_text = self.var_torque.get().strip()
        torque = _parse_float(torque_text) if torque_text else None
        valid = vel is not None and vel > 0 and accel is not None and accel > 0
        valid = valid and (not torque_text or (torque is not None and torque > 0))
        self.vel_entry.config(foreground='black' if vel is not None and vel > 0 else 'red')
        self.accel_entry.config(foreground='black' if accel is not None and accel > 0 else 'red')
        self.torque_entry.config(foreground='black' if not torque_text or
                                 torque is not None and torque > 0 else 'red')
        if not valid and any(r.enabled_var.get() for r in self._rows.values()):
            self._on_stop_all()
            self._log('运动参数无效，已停止全部电机', 'error')
            return
        for row in self._rows.values():
            if row.enabled_var.get():
                row.dirty = True

    def _on_enable_toggled(self, row: MotorRow) -> None:
        if not row.enabled_var.get():
            self.manager.command_stop([row.id])
            self._log(f'ID {row.id} 停止')
            return
        if self.manager.get_state() != ManagerState.CONNECTED:
            row.enabled_var.set(False)
            self._log('尚未连接 CAN', 'error')
            return
        if row._parse_limits() is None or self._global_params() is None:
            row.enabled_var.set(False)
            self._log(f'无法使能 ID {row.id}: 限位或运动参数无效', 'error')
            return
        status = self.manager.get_status_one(row.id)
        pos = status.position if status else float('nan')
        if (status is None or status.last_update < self._connected_at or
                time.monotonic() - status.last_update > 0.5 or
                not math.isfinite(pos) or status.fault != 0 or
                status.mode in (int(Mode.FAULT), int(Mode.TIMEOUT))):
            row.enabled_var.set(False)
            self._log(f'无法使能 ID {row.id}: 无有效实时位置反馈或存在故障',
                      'error')
            return
        if not row.lo <= pos <= row.hi:
            row.enabled_var.set(False)
            self._log(f'无法使能 ID {row.id}: 当前位置 {pos:.4f} 超出限位',
                      'error')
            return
        initial = pos
        row.set_target(initial)
        row.dirty = True
        self._log(f'ID {row.id} 使能, 初始目标 {initial:+.4f}')

    def _on_stop_one(self, row: MotorRow) -> None:
        row.enabled_var.set(False)
        row.dirty = False
        self.manager.command_stop([row.id])
        self._log(f'ID {row.id} 停止')

    def _on_stop_all(self) -> None:
        self._uncheck_all()
        self.manager.command_stop()
        self._log('⚠ 停止全部')

    def _uncheck_all(self) -> None:
        for row in self._rows.values():
            row.enabled_var.set(False)
            row.dirty = False

    # ── Command issue (from poller, main thread) ──────────────────────────

    def _global_params(self) -> Optional[Tuple[float, float, Optional[float]]]:
        """Return motion limits, or None when any field is invalid."""
        vel = _parse_float(self.var_vel_limit.get())
        accel = _parse_float(self.var_accel_limit.get())
        if vel is None or vel <= 0 or accel is None or accel <= 0:
            return None
        torque_text = self.var_torque.get().strip()
        torque = _parse_float(torque_text) if torque_text else None
        if torque_text and (torque is None or torque <= 0):
            return None
        return vel, accel, torque

    def _issue_dirty_commands(self) -> None:
        if self.manager.get_state() != ManagerState.CONNECTED:
            return
        if not any(r.enabled_var.get() and r.dirty for r in self._rows.values()):
            return
        params = self._global_params()
        if params is None:
            self._on_stop_all()
            self._log('运动参数无效，已停止全部电机', 'error')
            return
        vel, accel, torque = params
        for row in self._rows.values():
            if not (row.enabled_var.get() and row.dirty):
                continue
            row.dirty = False
            target = self._clamp(row, row.target)
            self.manager.command_position(
                [row.id], position=target,
                velocity_limit=vel,
                accel_limit=accel,
                maximum_torque=torque,
                watchdog_timeout=0.5,
                persistent=True,
                key=f'pos_{row.id}',
            )

    # ── Thread communication / polling ────────────────────────────────────

    def _register_listener(self) -> None:
        def _push(status: Dict[int, ControllerStatus]) -> None:
            try:
                self._status_q.put_nowait(status)
            except queue.Full:
                pass
        self.manager.add_listener(_push)

    def _poll_ui(self) -> None:
        latest: Optional[Dict[int, ControllerStatus]] = None
        while True:
            try:
                latest = self._status_q.get_nowait()
            except queue.Empty:
                break
        if latest:
            self._refresh_rows(latest)

        self._drain_scan_results()
        self._check_feedback_age()
        self._issue_dirty_commands()
        self._sync_connection_ui()

        if not self._closing:
            self.root.after(self._POLL_MS, self._poll_ui)

    def _refresh_rows(self, status: Dict[int, ControllerStatus]) -> None:
        for cid, s in status.items():
            row = self._rows.get(cid)
            if row is None:
                continue
            row.status_text(s)
            if (s.fault != 0 or s.mode in (int(Mode.FAULT),
                                           int(Mode.TIMEOUT))) and row.enabled_var.get():
                row.enabled_var.set(False)
                row.dirty = False
                self.manager.command_stop([cid])
                self._log(f'ID {cid} 状态异常 (mode={_mode_name(s.mode)}, '
                          f'fault={s.fault})，已自动停止',
                          'error')

    def _check_feedback_age(self) -> None:
        if self.manager.get_state() != ManagerState.CONNECTED:
            if any(row.enabled_var.get() for row in self._rows.values()):
                self._on_stop_all()
            return
        now = time.monotonic()
        for row in self._rows.values():
            if not row.enabled_var.get():
                continue
            status = self.manager.get_status_one(row.id)
            if (status is None or status.last_update < self._connected_at or
                    now - status.last_update > 0.5 or
                    not math.isfinite(status.position)):
                self._on_stop_one(row)
                self._log(f'ID {row.id} 反馈超时，已停止', 'error')

    def _drain_scan_results(self) -> None:
        while True:
            try:
                result = self._scan_q.get_nowait()
            except queue.Empty:
                return
            self.scan_btn.config(state='normal', text='自动探测')
            self.connect_btn.config(state='normal')
            if isinstance(result, Exception):
                self._log(f'扫描失败: {result}', 'error')
                continue
            if result:
                self.var_ids.set(','.join(str(i) for i in result))
                self._log(f'扫描完成, 在线电机: {result}')
            else:
                self._log('扫描完成, 未发现电机', 'warn')

    def _sync_connection_ui(self) -> None:
        state = self.manager.get_state()
        if state == self._prev_state:
            return
        self._prev_state = state

        if state == ManagerState.CONNECTED:
            self.connect_btn.config(state='disabled')
            self.disconnect_btn.config(state='normal')
            self._dot.config(foreground='green')
            self.var_status_lbl.set('已连接')
            self._log('已连接')
        elif state == ManagerState.CONNECTING:
            self.connect_btn.config(state='disabled')
            self.disconnect_btn.config(state='disabled')
            self._dot.config(foreground='orange')
            self.var_status_lbl.set('连接中…')
        elif state == ManagerState.ERROR:
            self.connect_btn.config(state='disabled')
            self.disconnect_btn.config(state='normal')
            self._dot.config(foreground='red')
            self.var_status_lbl.set('错误')
            self._log(f'错误: {self.manager.get_last_error()}', 'error')
        else:
            self.connect_btn.config(state='normal')
            self.disconnect_btn.config(state='disabled')
            self._dot.config(foreground='gray')
            self.var_status_lbl.set('未连接')

    def _log(self, msg: str, level: str = 'info') -> None:
        import datetime
        ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.log_text.config(state='normal')
        self.log_text.insert('end', f'[{ts}] {msg}\n', level)
        self.log_text.see('end')
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 800:
            self.log_text.delete('1.0', f'{lines - 400}.0')
        self.log_text.config(state='disabled')

    def _on_close(self) -> None:
        self._closing = True
        try:
            if self.manager.is_connected():
                self.manager.command_stop()
            self.manager.disconnect()
        except Exception:
            pass
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        prog='python3 scripts/multi_motor_gui.py',
        description='moteus 多电机调试 GUI（滑条位置控制 + 限位保护）',
    )
    parser.add_argument('--can-type', default='socketcan',
                        choices=['socketcan', 'fdcanusb', 'candle'],
                        help='CAN 接口类型 (默认 socketcan)')
    parser.add_argument('--can-chan', default='can0',
                        help='通道: can0 / /dev/ttyUSB0 / 0')
    parser.add_argument('--ids', default='1',
                        help='逗号分隔的电机 ID (默认 1)')
    parser.add_argument('--vel-limit', type=float, default=0.5,
                        help='默认速度上限 rev/s (默认 0.5)')
    parser.add_argument('--accel-limit', type=float, default=2.0,
                        help='默认加速度上限 rev/s^2 (默认 2.0)')
    parser.add_argument('--cycle-hz', type=float, default=50.0,
                        help='控制周期 Hz (默认 50)')
    parser.add_argument('--can-disable-brs', action='store_true',
                        help='禁用 CAN-FD BRS')
    args = parser.parse_args()

    if (not math.isfinite(args.vel_limit) or args.vel_limit <= 0 or
            not math.isfinite(args.accel_limit) or args.accel_limit <= 0 or
            not math.isfinite(args.cycle_hz) or args.cycle_hz <= 0):
        parser.error('速度、加速度和控制频率必须是正数')

    app = MultiMotorApp(
        default_can_type=args.can_type,
        default_can_chan=args.can_chan,
        default_ids=args.ids,
        default_vel_limit=args.vel_limit,
        default_accel_limit=args.accel_limit,
        cycle_hz=args.cycle_hz,
        can_disable_brs=args.can_disable_brs,
    )
    app.run()


if __name__ == '__main__':
    main()
