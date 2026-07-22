"""用模拟器验证 ControlManager 的实际控制/记录频率。

目的
----
排除真实 CAN 硬件往返耗时的干扰，单独测量 ``ControlManager`` 控制循环
（也就是 CSV 记录触发点 ``_fire_listeners``）能达到的真实频率。

因为 ``SimulatedTransport.cycle()`` 几乎瞬间返回，循环唯一的限速点就是
``_control_loop`` 末尾的 ``await asyncio.sleep(cycle_period - elapsed)``。
在未调整计时器精度的 Windows 上，该 sleep 被量化到 ~15.6ms，导致实际频率
被压到 ~64Hz（间隔 ~15ms），与 ``cycle_hz=200`` 完全对不上。

用法
----
    python scripts/test_cycle_rate_sim.py
    python scripts/test_cycle_rate_sim.py --cycle-hz 200 --seconds 3
"""

import argparse
import asyncio
import statistics
import sys
import time

from moteus.control_manager import ControlManager, ManagerState
from moteus.simulator import SimulatedTransport


def _install_sim_transport(manager: ControlManager, ids):
    """把 ControlManager 的 _build_transport 换成返回 SimulatedTransport。

    以实例属性覆盖 bound method；调用处是 ``self._build_transport(...)``，
    实例属性会绕过描述符绑定，不会自动传入 self，因此签名对齐 4 个参数即可。
    """
    async def _fake_build(can_type, can_iface, can_chan, can_disable_brs):
        return SimulatedTransport(list(ids))

    manager._build_transport = _fake_build


def _measure(cycle_hz: float, seconds: float, ids):
    manager = ControlManager(cycle_hz=cycle_hz)
    _install_sim_transport(manager, ids)

    stamps = []

    def _listener(_status):
        stamps.append(time.perf_counter())

    manager.add_listener(_listener)
    # can_type 随便填，反正 _build_transport 已被替换成 sim
    manager.connect(list(ids), can_type='sim', can_chan=None)

    # 等待进入 CONNECTED
    t0 = time.perf_counter()
    while manager.get_state() != ManagerState.CONNECTED:
        if manager.get_state() == ManagerState.ERROR:
            raise RuntimeError(f'连接失败: {manager.get_last_error()}')
        if time.perf_counter() - t0 > 3.0:
            raise RuntimeError('等待 CONNECTED 超时')
        time.sleep(0.005)

    # 丢弃启动瞬间的样本，稳定后再计时
    time.sleep(0.2)
    stamps.clear()
    time.sleep(seconds)
    manager.disconnect()

    return stamps


def _report(label: str, stamps, cycle_hz: float, seconds: float):
    n = len(stamps)
    print(f'\n=== {label} ===')
    if n < 2:
        print(f'  样本不足 (n={n})，无法统计')
        return
    span = stamps[-1] - stamps[0]
    freq = (n - 1) / span if span > 0 else float('nan')
    intervals_ms = [
        (stamps[i + 1] - stamps[i]) * 1000.0 for i in range(len(stamps) - 1)
    ]
    intervals_ms.sort()
    median = statistics.median(intervals_ms)
    p95 = intervals_ms[int(len(intervals_ms) * 0.95)]
    print(f'  目标频率      : {cycle_hz:.1f} Hz  (周期 {1000.0 / cycle_hz:.2f} ms)')
    print(f'  样本数        : {n}  (统计窗口 ~{seconds:.1f}s)')
    print(f'  实际频率      : {freq:.1f} Hz')
    print(f'  间隔中位数    : {median:.2f} ms')
    print(f'  间隔最小/最大 : {intervals_ms[0]:.2f} / {intervals_ms[-1]:.2f} ms')
    print(f'  间隔 p95      : {p95:.2f} ms')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cycle-hz', type=float, default=200.0)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--ids', type=int, nargs='+', default=[1])
    args = parser.parse_args()

    print(f'Python {sys.version.split()[0]}  platform={sys.platform}')
    print(f'目标 cycle_hz={args.cycle_hz}  控制器={args.ids}  每阶段测量 {args.seconds}s')

    # 阶段 1：默认计时器精度
    stamps1 = _measure(args.cycle_hz, args.seconds, args.ids)
    _report('阶段 1: 默认 Windows 计时器精度', stamps1, args.cycle_hz, args.seconds)

    # 阶段 2：timeBeginPeriod(1) 提高计时器精度
    if sys.platform == 'win32':
        import ctypes
        winmm = ctypes.WinDLL('winmm')
        winmm.timeBeginPeriod(1)
        try:
            stamps2 = _measure(args.cycle_hz, args.seconds, args.ids)
            _report('阶段 2: timeBeginPeriod(1) 后 (1ms 精度)',
                    stamps2, args.cycle_hz, args.seconds)
        finally:
            winmm.timeEndPeriod(1)
    else:
        print('\n(非 Windows 平台，跳过 timeBeginPeriod 对比)')


if __name__ == '__main__':
    main()
