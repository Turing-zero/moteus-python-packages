# Copyright 2025 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""moteus Controller GUI

Usage::

    python -m moteus_gui.fastgui.gui_app
    python -m moteus_gui.fastgui.gui_app --can-type socketcan --can-chan can0 --ids 1,2,3
    python -m moteus_gui.fastgui.gui_app --can-type fdcanusb --can-chan /dev/ttyUSB0 --ids 1
    python -m moteus_gui.fastgui.gui_app --can-type candle --can-chan 0 --ids 1

    # or via the console script installed with this package:
    moteus_fastgui --can-type socketcan --can-chan can0 --ids 1,2,3

CSV logging (programmatic)::

    from moteus.control_manager import ControlManager, CsvLogger

    manager = ControlManager(cycle_hz=500)
    logger = CsvLogger('/tmp/data.csv', controller_ids=[1, 2])
    manager.add_listener(logger.on_status_update)
    manager.connect([1, 2], can_type='socketcan', can_chan='can0')
    ...
    manager.disconnect()
    logger.close()
    # or as context manager:
    # with CsvLogger('/tmp/data.csv') as logger:
    #     manager.add_listener(logger.on_status_update)
    #     ...
"""

import argparse
import math
import os
import queue
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
from typing import Dict, List, Optional

from moteus.control_manager import ControlManager, ControllerStatus, CsvLogger, ManagerState
from moteus.protocol import Mode


# ── Helpers ───────────────────────────────────────────────────────────────────

def _mode_name(mode_int: int) -> str:
    try:
        return Mode(mode_int).name
    except (ValueError, KeyError):
        return str(mode_int)


def _fmt(v: float, decimals: int = 4) -> str:
    if math.isnan(v) or math.isinf(v):
        return 'N/A'
    return f'{v:.{decimals}f}'


def _opt_float(var: tk.StringVar) -> Optional[float]:
    s = var.get().strip()
    return float(s) if s else None


# ── Main application ──────────────────────────────────────────────────────────

class MoteusApp:
    """moteus controller GUI.

    Thread model:
    - Main thread: all tkinter operations
    - asyncio background thread: CAN I/O via ControlManager
    - queue.Queue + root.after(50 ms): status delivery from asyncio → tkinter
    """

    _STATUS_COLS = ('ID', 'MODE', 'POSITION', 'VELOCITY', 'TORQUE',
                    'VOLTAGE', 'TEMP', 'FAULT')
    _POLL_MS = 50
    _LOG_MAX_LINES = 800

    def __init__(
        self,
        default_can_type: str = 'socketcan',
        default_can_chan: str = 'can0',
        default_ids: str = '1',
        default_can_disable_brs: bool = False,
    ):
        self.manager = ControlManager(cycle_hz=1000.0, query_profile='lite')
        self._state_q: queue.Queue = queue.Queue(maxsize=200)
        self._tree_rows: Dict[int, str] = {}
        self._prev_state = ManagerState.DISCONNECTED
        self._csv_logger: Optional[CsvLogger] = None

        self.root = tk.Tk()
        self.root.title('moteus Controller GUI')
        self.root.minsize(900, 680)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)
        self.root.rowconfigure(3, weight=1)

        # ── StringVars ────────────────────────────────────────────────────────
        # Connection
        self.var_can_type = tk.StringVar(value=default_can_type)
        self.var_chan      = tk.StringVar(value=default_can_chan)
        self.var_ids       = tk.StringVar(value=default_ids)
        self.var_query_profile = tk.StringVar(value='lite')
        self.var_status_lbl = tk.StringVar(value='Disconnected')
        self._can_disable_brs = default_can_disable_brs

        # Position tab
        self.var_pos       = tk.StringVar(value='0.0')
        self.var_pos_vel   = tk.StringVar(value='')     # feed-forward velocity
        self.var_pos_kp    = tk.StringVar(value='')
        self.var_pos_kd    = tk.StringVar(value='')
        self.var_pos_trq   = tk.StringVar(value='')
        self.var_pos_persist = tk.BooleanVar(value=True)
        self.var_rezero    = tk.StringVar(value='0.0')

        # Velocity tab
        self.var_vel_tgt   = tk.StringVar(value='1.0')  # target velocity
        self.var_vel_kd    = tk.StringVar(value='')
        self.var_vel_trq   = tk.StringVar(value='')
        self.var_vel_persist = tk.BooleanVar(value=True)

        # CSV tab
        self.var_csv_path   = tk.StringVar(value='')
        self.var_csv_fields = tk.StringVar(
            value='timestamp,id,mode,position,velocity,torque,voltage,temperature,fault')
        self.var_csv_rows   = tk.StringVar(value='Rows: 0')

        self._build_ui()
        self._register_listener()
        self.root.after(self._POLL_MS, self._poll_ui)

    def run(self) -> None:
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.mainloop()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        pad = {'padx': 6, 'pady': 4}

        # Row 0: connection (left) + command notebook (right)
        top = ttk.Frame(self.root)
        top.grid(row=0, column=0, sticky='ew', **pad)
        top.columnconfigure(1, weight=1)
        self._build_connect_frame(top)
        self._build_cmd_notebook(top)

        # Row 1: quick status bar
        bar = ttk.Frame(self.root)
        bar.grid(row=1, column=0, sticky='ew', padx=6)
        self._status_bar = ttk.Label(bar, text='', font=('', 8), foreground='gray')
        self._status_bar.pack(side='left', padx=4)

        # Row 2: status table
        self._build_status_frame(self.root)

        # Row 3: log
        self._build_log_frame(self.root)

    def _build_connect_frame(self, parent: ttk.Frame) -> None:
        frm = ttk.LabelFrame(parent, text='CAN Connection')
        frm.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        frm.columnconfigure(1, weight=1)

        r = 0
        ttk.Label(frm, text='Interface:').grid(row=r, column=0, sticky='w', padx=6, pady=3)
        cb = ttk.Combobox(frm, textvariable=self.var_can_type, width=12,
                          values=['socketcan', 'fdcanusb', 'candle'], state='readonly')
        cb.grid(row=r, column=1, sticky='ew', padx=6, pady=3)
        cb.bind('<<ComboboxSelected>>', self._on_iface_changed)

        r += 1
        ttk.Label(frm, text='Channel:').grid(row=r, column=0, sticky='w', padx=6, pady=3)
        ttk.Entry(frm, textvariable=self.var_chan, width=18).grid(
            row=r, column=1, sticky='ew', padx=6, pady=3)

        r += 1
        ttk.Label(frm, text='IDs:').grid(row=r, column=0, sticky='w', padx=6, pady=3)
        ttk.Entry(frm, textvariable=self.var_ids, width=18).grid(
            row=r, column=1, sticky='ew', padx=6, pady=3)

        r += 1
        ttk.Label(frm, text='Query:').grid(row=r, column=0, sticky='w', padx=6, pady=3)
        ttk.Combobox(
            frm, textvariable=self.var_query_profile, width=12,
            values=['lite', 'full'], state='readonly',
        ).grid(row=r, column=1, sticky='ew', padx=6, pady=3)

        r += 1
        btn_row = ttk.Frame(frm)
        btn_row.grid(row=r, column=0, columnspan=2, pady=4)
        self.connect_btn = ttk.Button(btn_row, text='Connect', command=self._on_connect)
        self.connect_btn.pack(side='left', padx=4)
        self.disconnect_btn = ttk.Button(btn_row, text='Disconnect',
                                          command=self._on_disconnect, state='disabled')
        self.disconnect_btn.pack(side='left', padx=4)

        r += 1
        dot_row = ttk.Frame(frm)
        dot_row.grid(row=r, column=0, columnspan=2, pady=(0, 4))
        self._dot = ttk.Label(dot_row, text='●', foreground='gray', font=('', 13))
        self._dot.pack(side='left', padx=4)
        ttk.Label(dot_row, textvariable=self.var_status_lbl).pack(side='left')

    def _build_cmd_notebook(self, parent: ttk.Frame) -> None:
        nb = ttk.Notebook(parent)
        nb.grid(row=0, column=1, sticky='nsew')

        self._build_motion_tab(nb)
        self._build_csv_tab(nb)

    # ── Motion tab ────────────────────────────────────────────────────────────

    def _build_motion_tab(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text='  Motion  ')
        tab.columnconfigure(tuple(range(10)), weight=1)

        # Row 0: quick action buttons
        r = 0
        for col, (label, cmd) in enumerate([
            ('Stop All',  self._on_stop_all),
            ('Brake All', self._on_brake_all),
            ('Zero Vel',  self._on_zero_vel),
            ('Rezero →',  self._on_rezero),
        ]):
            ttk.Button(tab, text=label, command=cmd).grid(
                row=r, column=col, padx=4, pady=5, sticky='w')
        ttk.Entry(tab, textvariable=self.var_rezero, width=7).grid(
            row=r, column=4, sticky='w', padx=(0, 4))

        ttk.Separator(tab, orient='horizontal').grid(
            row=1, column=0, columnspan=10, sticky='ew', pady=4, padx=4)

        # Row 2-3: Position control
        r = 2
        ttk.Label(tab, text='── Position ──', foreground='gray').grid(
            row=r, column=0, columnspan=2, sticky='w', padx=6)

        r += 1
        pos_fields = [
            ('Target pos:', self.var_pos, 8),
            ('FF vel:',     self.var_pos_vel, 7),
            ('kp_scale:',   self.var_pos_kp, 6),
            ('kd_scale:',   self.var_pos_kd, 6),
        ]
        for i, (lbl, var, w) in enumerate(pos_fields):
            ttk.Label(tab, text=lbl).grid(row=r, column=i*2, sticky='e',
                                           padx=(6, 2), pady=2)
            ttk.Entry(tab, textvariable=var, width=w).grid(
                row=r, column=i*2+1, sticky='ew', padx=(0, 4), pady=2)

        r += 1
        ttk.Label(tab, text='Max torque:').grid(row=r, column=0, sticky='e',
                                                  padx=(6, 2), pady=2)
        ttk.Entry(tab, textvariable=self.var_pos_trq, width=8).grid(
            row=r, column=1, sticky='ew', padx=(0, 4), pady=2)
        ttk.Checkbutton(tab, text='Persist', variable=self.var_pos_persist).grid(
            row=r, column=2, columnspan=2, padx=4)
        ttk.Button(tab, text='Send Position', command=self._on_send_position).grid(
            row=r, column=4, columnspan=4, padx=6, pady=4, sticky='ew')

        ttk.Separator(tab, orient='horizontal').grid(
            row=5, column=0, columnspan=10, sticky='ew', pady=4, padx=4)

        # Row 6-7: Velocity control
        r = 6
        ttk.Label(tab, text='── Velocity ──', foreground='gray').grid(
            row=r, column=0, columnspan=2, sticky='w', padx=6)

        r += 1
        vel_fields = [
            ('Target vel:', self.var_vel_tgt, 8),
            ('kd_scale:',   self.var_vel_kd, 6),
            ('Max torque:', self.var_vel_trq, 8),
        ]
        for i, (lbl, var, w) in enumerate(vel_fields):
            ttk.Label(tab, text=lbl).grid(row=r, column=i*2, sticky='e',
                                           padx=(6, 2), pady=2)
            ttk.Entry(tab, textvariable=var, width=w).grid(
                row=r, column=i*2+1, sticky='ew', padx=(0, 4), pady=2)
        ttk.Checkbutton(tab, text='Persist', variable=self.var_vel_persist).grid(
            row=r, column=6, padx=4)
        ttk.Button(tab, text='Send Velocity', command=self._on_send_velocity).grid(
            row=r, column=7, columnspan=3, padx=6, pady=4, sticky='ew')

    # ── CSV Log tab ───────────────────────────────────────────────────────────

    def _build_csv_tab(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text='  CSV Log  ')
        tab.columnconfigure(1, weight=1)

        r = 0
        ttk.Label(tab, text='Output path:').grid(row=r, column=0, sticky='e',
                                                    padx=(8, 4), pady=6)
        path_row = ttk.Frame(tab)
        path_row.grid(row=r, column=1, columnspan=3, sticky='ew', padx=(0, 8))
        path_row.columnconfigure(0, weight=1)
        self._csv_path_entry = ttk.Entry(path_row, textvariable=self.var_csv_path)
        self._csv_path_entry.grid(row=0, column=0, sticky='ew')
        ttk.Button(path_row, text='Browse…', command=self._on_csv_browse).grid(
            row=0, column=1, padx=(4, 0))

        r += 1
        ttk.Label(tab, text='Fields:').grid(row=r, column=0, sticky='e',
                                              padx=(8, 4), pady=4)
        ttk.Entry(tab, textvariable=self.var_csv_fields, width=50).grid(
            row=r, column=1, columnspan=3, sticky='ew', padx=(0, 8), pady=4)

        r += 1
        ttk.Label(tab, text='Available:',
                  foreground='gray').grid(row=r, column=0, sticky='ne',
                                          padx=(8, 4), pady=(0, 4))
        avail = (
            'timestamp, id, mode, position, velocity, torque, voltage, temperature, fault, '
            'trajectory_complete, abs_position, '
            'encoder_0_position, encoder_0_velocity, '
            'encoder_1_position, encoder_1_velocity, '
            'encoder_2_position, encoder_2_velocity, '
            'encoder_validity'
        )
        ttk.Label(tab, text=avail, wraplength=400,
                  foreground='gray', font=('', 8)).grid(
            row=r, column=1, columnspan=3, sticky='w', padx=(0, 8), pady=(0, 4))

        r += 1
        btn_row = ttk.Frame(tab)
        btn_row.grid(row=r, column=0, columnspan=4, pady=8)
        self.csv_start_btn = ttk.Button(btn_row, text='Start Logging',
                                         command=self._on_csv_start)
        self.csv_start_btn.pack(side='left', padx=6)
        self.csv_stop_btn = ttk.Button(btn_row, text='Stop Logging',
                                        command=self._on_csv_stop, state='disabled')
        self.csv_stop_btn.pack(side='left', padx=6)
        ttk.Label(btn_row, textvariable=self.var_csv_rows,
                  foreground='gray').pack(side='left', padx=12)

        r += 1
        ttk.Label(tab, text=(
            'Tip: logging starts immediately; rows accumulate while connected.\n'
            'Stop before disconnecting to flush and close the file.\n'
            'Encoder/abs fields need Query=full (set before Connect). '
            'lite is faster for default fields.'
        ), foreground='gray', font=('', 8), justify='left').grid(
            row=r, column=0, columnspan=4, sticky='w', padx=8, pady=(4, 8))

    # ── Status table ──────────────────────────────────────────────────────────

    def _build_status_frame(self, parent: tk.Tk) -> None:
        frm = ttk.LabelFrame(parent, text='Controller Status')
        frm.grid(row=2, column=0, sticky='nsew', padx=6, pady=4)
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(0, weight=1)

        col_widths = {
            'ID': 50, 'MODE': 115, 'POSITION': 100, 'VELOCITY': 85,
            'TORQUE': 75, 'VOLTAGE': 70, 'TEMP': 60, 'FAULT': 55,
        }
        self.tree = ttk.Treeview(frm, columns=self._STATUS_COLS,
                                  show='headings', height=6)
        for col in self._STATUS_COLS:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=col_widths.get(col, 80),
                              anchor='center', stretch=True)
        self.tree.tag_configure('fault', foreground='red')
        self.tree.tag_configure('ok',    foreground='black')

        vsb = ttk.Scrollbar(frm, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.grid(row=0, column=0, sticky='nsew')
        vsb.grid(row=0, column=1, sticky='ns')

    # ── Log panel ─────────────────────────────────────────────────────────────

    def _build_log_frame(self, parent: tk.Tk) -> None:
        frm = ttk.LabelFrame(parent, text='Log')
        frm.grid(row=3, column=0, sticky='nsew', padx=6, pady=(0, 6))
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(
            frm, height=5, state='disabled', wrap='word', font=('Courier', 9))
        self.log_text.tag_config('error', foreground='red')
        self.log_text.tag_config('warn',  foreground='#b86400')
        self.log_text.tag_config('info',  foreground='black')
        self.log_text.grid(row=0, column=0, sticky='nsew', padx=2, pady=2)

    # ── Event handlers ────────────────────────────────────────────────────────

    def _on_iface_changed(self, _event=None) -> None:
        defaults = {'socketcan': 'can0', 'fdcanusb': '/dev/ttyUSB0', 'candle': '0'}
        iface = self.var_can_type.get()
        if self.var_chan.get() in defaults.values():
            self.var_chan.set(defaults.get(iface, ''))

    def _parse_ids(self) -> Optional[List[int]]:
        try:
            ids = [int(x.strip()) for x in self.var_ids.get().split(',') if x.strip()]
        except ValueError:
            self._log('Invalid controller IDs (comma-separated integers)', 'error')
            return None
        if not ids:
            self._log('No controller IDs specified', 'error')
            return None
        return ids

    def _on_connect(self) -> None:
        can_type = self.var_can_type.get()
        can_chan  = self.var_chan.get().strip()
        ids = self._parse_ids()
        if ids is None:
            return
        profile = self.var_query_profile.get().strip() or 'lite'
        try:
            self.manager.set_query_profile(profile)
        except (ValueError, RuntimeError) as e:
            self._log(f'Query profile: {e}', 'error')
            return
        self._init_tree_rows(ids)
        self._log(
            f'Connecting: type={can_type} chan={can_chan} ids={ids} '
            f'query={profile}'
        )
        self.manager.connect(
            ids,
            can_type=can_type,
            can_chan=can_chan,
            can_disable_brs=self._can_disable_brs,
        )

    def _on_disconnect(self) -> None:
        if self._csv_logger:
            self._on_csv_stop()
        self._log('Disconnecting…')
        self.manager.disconnect()

    def _on_stop_all(self) -> None:
        self.manager.command_stop()
        self._log('STOP ALL')

    def _on_brake_all(self) -> None:
        self.manager.command_brake()
        self._log('BRAKE ALL')

    def _on_zero_vel(self) -> None:
        self.manager.command_zero_velocity()
        self._log('ZERO VELOCITY')

    def _on_rezero(self) -> None:
        try:
            val = float(self.var_rezero.get())
        except ValueError:
            self._log('Invalid rezero value', 'error')
            return
        self.manager.command_rezero(rezero=val)
        self._log(f'REZERO → {val:.4f}')

    def _on_send_position(self) -> None:
        ids = self._parse_ids()
        if ids is None:
            return
        try:
            pos = float(self.var_pos.get())
        except ValueError:
            self._log('Invalid position value', 'error')
            return
        persistent = self.var_pos_persist.get()
        self.manager.command_position(
            ids, position=pos,
            velocity=_opt_float(self.var_pos_vel),
            kp_scale=_opt_float(self.var_pos_kp),
            kd_scale=_opt_float(self.var_pos_kd),
            maximum_torque=_opt_float(self.var_pos_trq),
            persistent=persistent,
        )
        self._log(
            f'POSITION ({"persist" if persistent else "once"}) '
            f'IDs={ids} pos={pos:.4f}'
            + (f' ff_vel={self.var_pos_vel.get().strip()}' if self.var_pos_vel.get().strip() else '')
        )

    def _on_send_velocity(self) -> None:
        ids = self._parse_ids()
        if ids is None:
            return
        try:
            vel = float(self.var_vel_tgt.get())
        except ValueError:
            self._log('Invalid velocity value', 'error')
            return
        persistent = self.var_vel_persist.get()
        self.manager.command_velocity(
            ids, velocity=vel,
            kd_scale=_opt_float(self.var_vel_kd),
            maximum_torque=_opt_float(self.var_vel_trq),
            persistent=persistent,
        )
        self._log(
            f'VELOCITY ({"persist" if persistent else "once"}) '
            f'IDs={ids} vel={vel:.3f} rev/s'
        )

    # ── CSV handlers ──────────────────────────────────────────────────────────

    def _on_csv_browse(self) -> None:
        path = filedialog.asksaveasfilename(
            title='Save CSV log',
            defaultextension='.csv',
            filetypes=[('CSV files', '*.csv'), ('All files', '*.*')],
            initialdir=os.path.expanduser('~'),
        )
        if path:
            self.var_csv_path.set(path)

    def _on_csv_start(self) -> None:
        path = self.var_csv_path.get().strip()
        if not path:
            self._log('No CSV path specified — use Browse or type a path', 'error')
            return

        fields_raw = self.var_csv_fields.get().strip()
        fields = [f.strip() for f in fields_raw.split(',') if f.strip()] or None

        heavy = {
            'trajectory_complete', 'abs_position', 'encoder_validity',
            'encoder_0_position', 'encoder_0_velocity',
            'encoder_1_position', 'encoder_1_velocity',
            'encoder_2_position', 'encoder_2_velocity',
        }
        if fields and any(f in heavy for f in fields):
            if self.manager.get_query_profile() != 'full':
                self._log(
                    'CSV fields include encoder/abs data but Query=lite — '
                    'those columns will be NaN. Reconnect with Query=full.',
                    'warn',
                )

        try:
            self._csv_logger = CsvLogger(path, fields=fields)
        except OSError as e:
            self._log(f'Cannot open CSV file: {e}', 'error')
            return

        self.manager.add_listener(self._csv_logger.on_status_update)
        self.csv_start_btn.config(state='disabled')
        self.csv_stop_btn.config(state='normal')
        self._log(f'CSV logging → {path}')

    def _on_csv_stop(self) -> None:
        if self._csv_logger is None:
            return
        self.manager.remove_listener(self._csv_logger.on_status_update)
        rows = self._csv_logger.row_count
        self._csv_logger.close()
        self._csv_logger = None
        self.csv_start_btn.config(state='normal')
        self.csv_stop_btn.config(state='disabled')
        self.var_csv_rows.set('Rows: 0')
        self._log(f'CSV logging stopped — {rows} rows written')

    # ── Thread communication ─────────────────────────────────────────────────

    def _register_listener(self) -> None:
        def _push(status: Dict[int, ControllerStatus]) -> None:
            try:
                self._state_q.put_nowait(status)
            except queue.Full:
                pass
        self.manager.add_listener(_push)

    def _poll_ui(self) -> None:
        latest: Optional[Dict[int, ControllerStatus]] = None
        while True:
            try:
                latest = self._state_q.get_nowait()
            except queue.Empty:
                break

        if latest:
            self._refresh_tree(latest)

        self._sync_connection_ui()

        # Update CSV row count
        if self._csv_logger is not None:
            self.var_csv_rows.set(f'Rows: {self._csv_logger.row_count}')

        self.root.after(self._POLL_MS, self._poll_ui)

    def _refresh_tree(self, status: Dict[int, ControllerStatus]) -> None:
        for cid, s in status.items():
            values = (
                cid,
                _mode_name(s.mode),
                _fmt(s.position, 4),
                _fmt(s.velocity, 3),
                _fmt(s.torque, 3),
                _fmt(s.voltage, 1),
                _fmt(s.temperature, 1),
                str(s.fault),
            )
            tag = 'fault' if s.fault != 0 else 'ok'
            if cid in self._tree_rows:
                self.tree.item(self._tree_rows[cid], values=values, tags=(tag,))
            else:
                iid = self.tree.insert('', 'end', values=values, tags=(tag,))
                self._tree_rows[cid] = iid

    def _sync_connection_ui(self) -> None:
        state = self.manager.get_state()
        if state == self._prev_state:
            return
        self._prev_state = state

        connected = state == ManagerState.CONNECTED
        if state == ManagerState.CONNECTED:
            self.connect_btn.config(state='disabled', text='Connect')
            self.disconnect_btn.config(state='normal')
            self._dot.config(foreground='green')
            self.var_status_lbl.set('Connected')
            self._log('Connected')
        elif state == ManagerState.CONNECTING:
            self.connect_btn.config(state='disabled', text='Connecting…')
            self.disconnect_btn.config(state='disabled')
            self._dot.config(foreground='orange')
            self.var_status_lbl.set('Connecting…')
        elif state == ManagerState.ERROR:
            self.connect_btn.config(state='normal', text='Connect')
            self.disconnect_btn.config(state='disabled')
            self._dot.config(foreground='red')
            self.var_status_lbl.set('Error')
            self._log(f'Error: {self.manager.get_last_error()}', 'error')
        else:
            self.connect_btn.config(state='normal', text='Connect')
            self.disconnect_btn.config(state='disabled')
            self._dot.config(foreground='gray')
            self.var_status_lbl.set('Disconnected')

    def _log(self, msg: str, level: str = 'info') -> None:
        import datetime
        ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.log_text.config(state='normal')
        self.log_text.insert('end', f'[{ts}] {msg}\n', level)
        self.log_text.see('end')
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > self._LOG_MAX_LINES:
            self.log_text.delete('1.0', f'{lines - self._LOG_MAX_LINES // 2}.0')
        self.log_text.config(state='disabled')

    def _init_tree_rows(self, ids: List[int]) -> None:
        for iid in self.tree.get_children():
            self.tree.delete(iid)
        self._tree_rows.clear()
        placeholder = ('---',) * (len(self._STATUS_COLS) - 1)
        for cid in ids:
            iid = self.tree.insert('', 'end', values=(cid,) + placeholder)
            self._tree_rows[cid] = iid

    def _on_close(self) -> None:
        if self._csv_logger:
            self._on_csv_stop()
        self.manager.disconnect()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        prog='python -m moteus_gui.fastgui.gui_app',
        description='moteus controller GUI',
    )
    parser.add_argument('--can-type', default='socketcan',
                        choices=['socketcan', 'fdcanusb', 'candle'], metavar='TYPE',
                        help='CAN interface type (default: socketcan)')
    parser.add_argument('--can-chan', default='can0', metavar='CHAN',
                        help='channel: can0 / /dev/ttyUSB0 / 0')
    parser.add_argument('--ids', default='1', metavar='IDS',
                        help='comma-separated controller IDs (default: 1)')
    parser.add_argument('--can-disable-brs', action='store_true',
                        help='disable CAN-FD BRS flag for transport devices')
    args = parser.parse_args()

    app = MoteusApp(
        default_can_type=args.can_type,
        default_can_chan=args.can_chan,
        default_ids=args.ids,
        default_can_disable_brs=args.can_disable_brs,
    )
    app.run()


if __name__ == '__main__':
    main()
