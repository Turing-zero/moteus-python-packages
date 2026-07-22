"""验证 Windows 跨线程唤醒延迟受系统计时器 quantum 影响。

背景
----
真实 fdcanusb 路径通过 ``moteus/aiostream.py`` 的独立读/写线程 + 队列完成串口
I/O。每次 CAN 往返都要经过若干次"事件循环线程 → 工作线程 →
run_coroutine_threadsafe 回到事件循环线程"的跨线程切换。

在 Windows 上，线程调度 / 跨线程唤醒的时间粒度由系统计时器 quantum 决定，
默认约 15.6ms。因此每一跳可能引入最多 ~15.6ms 的等待，累积成实测中
CAN 往返 ~15ms 的上限。这一点在纯单线程的 sim 中不会出现（无跨线程跳转）。

本脚本复刻 aiostream 的跨线程往返模式，测量单跳延迟，并对比调用
``timeBeginPeriod(1)`` 前后的差异。
"""

import argparse
import asyncio
import queue
import statistics
import sys
import threading
import time


def _run_queue(q, stop):
    # 复刻 aiostream._run_queue：阻塞取任务并执行。
    while not stop.is_set():
        try:
            item = q.get(block=True, timeout=0.05)
            item()
        except queue.Empty:
            pass


async def _measure_round_trips(n: int):
    """执行 n 次"提交任务到工作线程并等待其回调解析 future"的往返。"""
    loop = asyncio.get_event_loop()
    work_q: queue.Queue = queue.Queue()
    stop = threading.Event()
    worker = threading.Thread(target=_run_queue, args=(work_q, stop), daemon=True)
    worker.start()

    intervals_ms = []
    try:
        # 预热
        for _ in range(20):
            await _one_hop(loop, work_q)

        for _ in range(n):
            t0 = time.perf_counter()
            await _one_hop(loop, work_q)
            intervals_ms.append((time.perf_counter() - t0) * 1000.0)
    finally:
        stop.set()
        worker.join(timeout=1.0)

    return intervals_ms


async def _one_hop(loop, work_q):
    # 复刻 aiostream.read/drain：提交任务到工作线程，工作线程用
    # run_coroutine_threadsafe 把结果送回事件循环并解析 future。
    f = loop.create_future()

    def job():
        async def _set():
            if not f.done():
                f.set_result(True)
        asyncio.run_coroutine_threadsafe(_set(), loop)

    work_q.put_nowait(job)
    await f


def _report(label: str, intervals_ms):
    intervals_ms = sorted(intervals_ms)
    n = len(intervals_ms)
    median = statistics.median(intervals_ms)
    mean = statistics.mean(intervals_ms)
    p95 = intervals_ms[int(n * 0.95)]
    print(f'\n=== {label} ===')
    print(f'  样本数        : {n}')
    print(f'  单跳均值      : {mean:.3f} ms')
    print(f'  单跳中位数    : {median:.3f} ms')
    print(f'  单跳最小/最大 : {intervals_ms[0]:.3f} / {intervals_ms[-1]:.3f} ms')
    print(f'  单跳 p95      : {p95:.3f} ms')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-n', type=int, default=300, help='每阶段往返次数')
    args = parser.parse_args()

    print(f'Python {sys.version.split()[0]}  platform={sys.platform}')
    print(f'每阶段测量 {args.n} 次跨线程往返')

    intervals1 = asyncio.run(_measure_round_trips(args.n))
    _report('阶段 1: 默认系统计时器', intervals1)

    if sys.platform == 'win32':
        import ctypes
        winmm = ctypes.WinDLL('winmm')
        winmm.timeBeginPeriod(1)
        try:
            intervals2 = asyncio.run(_measure_round_trips(args.n))
            _report('阶段 2: timeBeginPeriod(1) 后 (1ms 精度)', intervals2)
        finally:
            winmm.timeEndPeriod(1)
    else:
        print('\n(非 Windows 平台，跳过 timeBeginPeriod 对比)')


if __name__ == '__main__':
    main()
