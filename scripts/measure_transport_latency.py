"""在真实硬件上测量单次 transport.cycle() 往返耗时。

sim 实验已证明 ControlManager 的控制循环本身能跑到 >700Hz，15ms 的上限
来自真实 CAN transport 的 I/O 往返。本脚本用你实际的适配器直接测量
``transport.cycle([query])`` 的单次耗时，从而定位延迟到底有多大、是否稳定。

用法示例
--------
fdcanusb (串口)::
    python scripts/measure_transport_latency.py --can-type fdcanusb --can-chan COM3 --ids 1

candle::
    python scripts/measure_transport_latency.py --can-type candle --can-chan 0 --ids 1

python-can (pcan / gs_usb 等)::
    python scripts/measure_transport_latency.py --can-type pcan --can-chan PCAN_USBBUS1 --ids 1

可加 --debug-log serial.log（仅 fdcanusb）把每帧收发时间戳写盘，进一步核对
是"命令发出→响应到达"之间慢，还是别处慢。
"""

import argparse
import asyncio
import statistics
import time
import typing

import moteus
from moteus.transport import Transport
from moteus.fdcanusb_device import FdcanusbDevice
from moteus.pythoncan_device import PythonCanDevice
from moteus.candle_device import CandleDevice


def _build_transport(can_type, can_iface, can_chan, can_disable_brs, debug_log):
    # 与 ControlManager._build_transport 保持一致，额外支持 fdcanusb debug_log。
    log_fp = open(debug_log, 'wb') if debug_log else None
    if can_type == 'fdcanusb':
        device = FdcanusbDevice(can_chan, disable_brs=can_disable_brs,
                                debug_log=log_fp)
    elif can_type == 'candle':
        device = CandleDevice(channel_index=int(can_chan or '0'),
                              disable_brs=can_disable_brs)
    else:
        kwargs: typing.Dict[str, typing.Any] = {}
        if can_chan:
            kwargs['channel'] = can_chan
        if can_iface:
            kwargs['interface'] = can_iface
        if can_disable_brs:
            kwargs['disable_brs'] = True
        device = PythonCanDevice(**kwargs)
    return Transport([device]), log_fp


async def _run(args):
    transport, log_fp = _build_transport(
        args.can_type, args.can_iface, args.can_chan,
        args.can_disable_brs, args.debug_log)
    controllers = {cid: moteus.Controller(id=cid, transport=transport)
                   for cid in args.ids}

    print(f'transport={args.can_type} chan={args.can_chan} ids={args.ids}')
    print(f'预热 20 次，测量 {args.n} 次 transport.cycle()...\n')

    async def one_cycle():
        return await transport.cycle(
            [c.make_query() for c in controllers.values()])

    # 预热（首个 cycle 可能触发路由/发现）
    for _ in range(20):
        await one_cycle()

    intervals_ms = []
    for _ in range(args.n):
        t0 = time.perf_counter()
        await one_cycle()
        intervals_ms.append((time.perf_counter() - t0) * 1000.0)

    transport.close()
    if log_fp:
        log_fp.close()

    intervals_ms.sort()
    n = len(intervals_ms)
    span_hz = 1000.0 / statistics.median(intervals_ms)
    print('=== transport.cycle() 单次往返耗时 ===')
    print(f'  样本数        : {n}')
    print(f'  均值          : {statistics.mean(intervals_ms):.3f} ms')
    print(f'  中位数        : {statistics.median(intervals_ms):.3f} ms  '
          f'(≈ {span_hz:.1f} Hz 上限)')
    print(f'  最小/最大     : {intervals_ms[0]:.3f} / {intervals_ms[-1]:.3f} ms')
    print(f'  p95           : {intervals_ms[int(n * 0.95)]:.3f} ms')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--can-type', default='fdcanusb',
                        choices=['fdcanusb', 'candle', 'pcan', 'socketcan',
                                 'kvaser', 'vector', 'gs_usb'])
    parser.add_argument('--can-chan', default=None)
    parser.add_argument('--can-iface', default=None,
                        help='python-can interface（当 can-type 为 pcan/gs_usb 等时可省略）')
    parser.add_argument('--ids', type=int, nargs='+', default=[1])
    parser.add_argument('--n', type=int, default=500)
    parser.add_argument('--can-disable-brs', action='store_true')
    parser.add_argument('--debug-log', default=None,
                        help='fdcanusb 专用：把每帧收发写入该文件')
    args = parser.parse_args()

    # 对 python-can 类型，can-type 实际是 interface 名
    if args.can_type in ('pcan', 'socketcan', 'kvaser', 'vector', 'gs_usb'):
        args.can_iface = args.can_type
        args.can_type = 'pythoncan'

    asyncio.run(_run(args))


if __name__ == '__main__':
    main()
