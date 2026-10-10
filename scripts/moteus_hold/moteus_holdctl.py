#!/usr/bin/env python3
# Copyright 2025 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""moteus 位置保持守护进程的命令行调试工具

不带参数进入交互式命令行；带参数时执行单条命令后退出。

Usage::

    moteus_holdctl.py                    # 交互模式
    moteus_holdctl.py status             # 打印一次状态
    moteus_holdctl.py watch 5            # 5 Hz 持续刷新，Ctrl-C 退出
    moteus_holdctl.py step 1 0.01        # 电机 1 目标 +0.01 rev
    moteus_holdctl.py set 2 0.5          # 电机 2 目标设为 0.5 rev
    moteus_holdctl.py jog                # 键盘点动界面

交互模式命令（输入 help 查看）::

    1+ / 1- / 2+0.05 / all-   按步长（或给定值）step
    ss 0.02                   设置步长
    s / w                     状态 / 持续刷新
"""

import argparse
import json
import math
import os
import re
import shlex
import socket
import sys
import time
from typing import Any, Dict, List, Optional

DEFAULT_SOCKETS = ['/run/moteus-hold/ctl.sock', '/tmp/moteus-hold.sock']

HELP = """\
命令（ID 可为数字或 all；位置单位 rev，电流 A）:
  s | status                 打印状态
  w | watch [hz]             持续刷新（默认 5 Hz），Ctrl-C 返回
  <id>+ | <id>-              按当前步长移动目标，如 1+  2-  all+
  <id>+<v> | <id>-<v>        按给定增量移动，如 1+0.05  3-0.1
  step <id> <delta>          同上
  set <id> <pos>             设置绝对目标位置
  ss | stepsize [v]          查看/设置步长
  hold [id]                  在当前位置保持（默认 all）
  stop [id]                  释放电机（moteus stop，默认 all）
  clear <id>                 发 stop 清除故障/超时后在当前位置保持
  limit <id> <lo> <hi>       软件限位，none 表示不限
  param [name value]         查看/修改运行参数：velocity_limit accel_limit
                             maximum_torque kp_scale kd_scale watchdog_timeout
                             max_step max_move
  jog                        键盘点动界面
  help | q
"""


# ── socket 客户端 ─────────────────────────────────────────────────────────────

def resolve_socket(path: Optional[str]) -> str:
    if path:
        return path
    env = os.environ.get('MOTEUS_HOLD_SOCK')
    if env:
        return env
    for p in DEFAULT_SOCKETS:
        if os.path.exists(p):
            return p
    raise SystemExit(
        f'找不到守护进程的 socket（{", ".join(DEFAULT_SOCKETS)}），'
        f'守护进程是否在运行？可用 --socket 指定')


class Client:
    def __init__(self, path: str):
        self.path = path
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.sock.connect(path)
        except OSError as e:
            raise SystemExit(f'连接 {path} 失败: {e}')
        self.sock.settimeout(3.0)
        self.rfile = self.sock.makefile('rb')

    def request(self, **req) -> Dict[str, Any]:
        self.sock.sendall((json.dumps(req) + '\n').encode('utf-8'))
        line = self.rfile.readline()
        if not line:
            raise SystemExit('守护进程关闭了连接')
        return json.loads(line)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


# ── 格式化 ────────────────────────────────────────────────────────────────────

def _f(v: Any, fmt: str) -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        width = re.match(r'\d*', fmt).group(0)
        return '---'.rjust(int(width)) if width else '---'
    return format(v, fmt)


def format_status(resp: Dict[str, Any]) -> List[str]:
    lines = []
    d = resp.get('daemon')
    if d:
        p = d.get('params', {})
        lines.append(
            f"bus {d['channel']}{'' if d['bus_open'] else '(closed)'}  "
            f"{d['loop_hz']:.0f}/{d['rate']:.0f} Hz  cycle {d['cycle_ms']:.2f} ms  "
            f"timeout {d['timeouts']}  reset {d['resets']}  up {d['uptime_s']:.0f}s")
        lines.append(
            f"vel_lim {_f(p.get('velocity_limit'), 'g')}  "
            f"acc_lim {_f(p.get('accel_limit'), 'g')}  "
            f"max_tq {_f(p.get('maximum_torque'), 'g')}  "
            f"max_step {_f(p.get('max_step'), 'g')}  "
            f"max_move {_f(p.get('max_move'), 'g')}")
    lines.append(
        f"{'ID':>3} {'STATE':<5} {'MODE':<13} {'TARGET':>9} {'POS':>9} "
        f"{'VEL':>8} {'TORQUE':>7} {'Iq(A)':>7} {'Id(A)':>7} {'VBUS':>5} "
        f"{'TEMP':>4} {'FLT':>3} {'AGE':>5}")
    for m in resp.get('motors', []):
        if not m['online']:
            state = 'off'
        elif m['holding']:
            state = 'hold'
        else:
            state = 'free'
        age = m.get('age_ms')
        lines.append(
            f"{m['id']:>3} {state:<5} {m['mode_name'][:13]:<13} "
            f"{_f(m['target'], '9.4f')} {_f(m['position'], '9.4f')} "
            f"{_f(m['velocity'], '8.4f')} {_f(m['torque'], '7.3f')} "
            f"{_f(m['q_current'], '7.3f')} {_f(m['d_current'], '7.3f')} "
            f"{_f(m['voltage'], '5.1f')} {_f(m['temperature'], '4.0f')} "
            f"{m['fault']:>3} {_f(age if age is not None and age < 99999 else None, '5.0f')}")
    return lines


def format_messages(resp: Dict[str, Any]) -> List[str]:
    return ([f'  {x}' for x in resp.get('msgs', [])] +
            [f'  错误: {x}' for x in resp.get('errors', [])])


# ── 命令解析 ─────────────────────────────────────────────────────────────────

_JOG_RE = re.compile(r'^(all|\d+)([+-])([0-9.]*(?:e-?\d+)?)$', re.IGNORECASE)


def _ids(tok: Optional[str]):
    if tok is None or tok.lower() == 'all':
        return 'all'
    return [int(x) for x in tok.split(',')]


def _num_or_none(tok: str) -> Optional[float]:
    return None if tok.lower() in ('none', '-', 'inf') else float(tok)


class Shell:
    def __init__(self, client: Client, stepsize: float):
        self.client = client
        self.stepsize = stepsize

    def show(self, resp: Dict[str, Any], table: bool = True) -> None:
        for line in format_messages(resp):
            print(line)
        if table and 'motors' in resp:
            for line in format_status(resp):
                print(line)

    def run_line(self, line: str) -> bool:
        """执行一条命令；返回 False 表示退出。"""
        line = line.strip()
        if not line or line.startswith('#'):
            return True
        toks = shlex.split(line)
        cmd = toks[0].lower()
        args = toks[1:]
        c = self.client

        m = _JOG_RE.match(line.replace(' ', ''))
        if m:
            target, sign, val = m.groups()
            delta = float(val) if val else self.stepsize
            delta = delta if sign == '+' else -delta
            self.show(c.request(cmd='step', ids=_ids(target), delta=delta))
            return True

        if cmd in ('q', 'quit', 'exit'):
            return False
        if cmd in ('h', 'help', '?'):
            print(HELP)
        elif cmd in ('s', 'status'):
            self.show(c.request(cmd='status'))
        elif cmd in ('w', 'watch'):
            self.watch(float(args[0]) if args else 5.0)
        elif cmd == 'step':
            self.show(c.request(cmd='step', ids=_ids(args[0]),
                                delta=float(args[1])))
        elif cmd == 'set':
            self.show(c.request(cmd='set', ids=_ids(args[0]),
                                position=float(args[1])))
        elif cmd in ('ss', 'stepsize'):
            if args:
                v = float(args[0])
                if not (v > 0 and math.isfinite(v)):
                    raise ValueError('步长必须为正数')
                self.stepsize = v
            print(f'  步长 = {self.stepsize:g} rev')
        elif cmd in ('hold', 'stop', 'clear'):
            if cmd == 'clear' and not args:
                raise ValueError('clear 需要指定 ID（或 all）')
            self.show(c.request(cmd=cmd, ids=_ids(args[0] if args else None)))
        elif cmd == 'limit':
            if len(args) != 3:
                raise ValueError('用法: limit <id> <lo|none> <hi|none>')
            self.show(c.request(cmd='limit', id=int(args[0]),
                                lo=_num_or_none(args[1]),
                                hi=_num_or_none(args[2])))
        elif cmd == 'param':
            if not args:
                resp = c.request(cmd='param')
                for k, v in resp['daemon']['params'].items():
                    print(f'  {k:<17} {v}')
            elif len(args) == 2:
                self.show(c.request(cmd='param', name=args[0], value=args[1]),
                          table=False)
            else:
                raise ValueError('用法: param [name value]')
        elif cmd == 'jog':
            jog(c, self.stepsize)
        else:
            print(f'  未知命令 {cmd!r}，输入 help 查看')
        return True

    def watch(self, hz: float) -> None:
        period = 1.0 / max(0.5, min(hz, 50.0))
        tty = sys.stdout.isatty()
        try:
            while True:
                lines = format_status(self.client.request(cmd='status'))
                if tty:
                    sys.stdout.write('\x1b[H\x1b[2J')
                sys.stdout.write('\n'.join(lines) + '\n')
                if tty:
                    sys.stdout.write('(Ctrl-C 返回)\n')
                sys.stdout.flush()
                time.sleep(period)
        except KeyboardInterrupt:
            print()

    def repl(self) -> None:
        try:
            import readline  # noqa: F401  (启用方向键/历史)
            hist = os.path.expanduser('~/.moteus_holdctl_history')
            try:
                readline.read_history_file(hist)
            except OSError:
                pass
        except ImportError:
            readline = None
            hist = None

        print(f'已连接 {self.client.path}，步长 {self.stepsize:g} rev，输入 help 查看命令')
        self.show(self.client.request(cmd='status'))
        try:
            while True:
                try:
                    line = input('holdctl> ')
                except EOFError:
                    print()
                    break
                except KeyboardInterrupt:
                    print()
                    continue
                try:
                    if not self.run_line(line):
                        break
                except IndexError:
                    print('  参数不足（输入 help 查看用法）')
                except ValueError as e:
                    print(f'  参数错误: {e}（输入 help 查看用法）')
        finally:
            if readline is not None and hist:
                try:
                    readline.write_history_file(hist)
                except OSError:
                    pass


# ── 键盘点动（curses） ───────────────────────────────────────────────────────

def jog(client: Client, stepsize: float) -> None:
    import curses
    import locale
    locale.setlocale(locale.LC_ALL, '')

    keys_help = ('数字键选电机  a 全部  ↑/+ 正向  ↓/- 反向  [ ] 步长减半/加倍  '
                 'h 保持  x 释放  c 清故障  q 退出')

    def main(scr):
        curses.curs_set(0)
        scr.timeout(100)
        ids = [m['id'] for m in client.request(cmd='status')['motors']]
        sel: Any = ids[0] if ids else 'all'
        step = stepsize
        msg = ''
        while True:
            resp = client.request(cmd='status')
            scr.erase()
            rows = [
                f'jog  选中: {sel}  步长: {step:g} rev',
                keys_help,
                '',
            ] + format_status(resp) + ['', msg]
            h, w = scr.getmaxyx()
            for i, row in enumerate(rows[:h - 1]):
                attr = 0
                if i >= 5 and row.split() and row.split()[0] == str(sel):
                    attr = curses.A_REVERSE
                try:
                    scr.addstr(i, 0, row[:w - 1], attr)
                except curses.error:
                    pass
            scr.refresh()

            ch = scr.getch()
            if ch == -1:
                continue
            target = 'all' if sel == 'all' else [sel]
            r = None
            if ch in (ord('q'), 27):
                return
            elif ch in (curses.KEY_UP, ord('+'), ord('=')):
                r = client.request(cmd='step', ids=target, delta=step)
            elif ch in (curses.KEY_DOWN, ord('-'), ord('_')):
                r = client.request(cmd='step', ids=target, delta=-step)
            elif ch == ord(']'):
                step *= 2
            elif ch == ord('['):
                step /= 2
            elif ch == ord('a'):
                sel = 'all'
            elif ord('0') <= ch <= ord('9'):
                n = ch - ord('0')
                if n in ids:
                    sel = n
            elif ch == ord('h'):
                r = client.request(cmd='hold', ids=target)
            elif ch == ord('x'):
                r = client.request(cmd='stop', ids=target)
            elif ch == ord('c'):
                r = client.request(cmd='clear', ids=target)
            if r is not None:
                msg = '  '.join(r.get('msgs', []) +
                                [f'错误: {e}' for e in r.get('errors', [])])

    curses.wrapper(main)


def main() -> None:
    p = argparse.ArgumentParser(
        description='moteus 位置保持守护进程调试工具',
        epilog=HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--socket', default=None,
                   help='守护进程 socket（默认按 $MOTEUS_HOLD_SOCK、'
                        + '、'.join(DEFAULT_SOCKETS) + ' 依次查找）')
    p.add_argument('--step', type=float, default=0.01,
                   help='初始步长 rev（默认 0.01）')
    p.add_argument('command', nargs=argparse.REMAINDER,
                   help='单条命令，省略则进入交互模式')
    args = p.parse_args()

    client = Client(resolve_socket(args.socket))
    shell = Shell(client, args.step)
    try:
        if args.command:
            try:
                shell.run_line(shlex.join(args.command))
            except IndexError:
                raise SystemExit('参数不足（--help 查看用法）')
            except ValueError as e:
                raise SystemExit(f'参数错误: {e}')
        else:
            shell.repl()
    finally:
        client.close()


if __name__ == '__main__':
    main()
