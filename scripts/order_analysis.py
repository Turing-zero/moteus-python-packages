#!/usr/bin/env python3
# Copyright 2026 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""moteus servo_stats 阶次域分析工具 (order-domain torque spectrum).

原理
====
用 position 列(编码器积分转数)把 torque 从"时间均匀采样"重采样为
"转角均匀采样", 再做 FFT。横轴 = 机械阶次 (order, 周/转), 与时间戳
无关, 因此:
  * 转速微小波动不糊峰;
  * 时间戳标定误差(median/mean Δt 偏差)在阶次域不存在。
代价: 超过阶次奈奎斯特 (≈每转点数/2) 的分量仍混叠; 1~26 阶通常安全。

阶次归因 (电机 24S-26P, pp=13 为例)
==================================
  1 阶      -> 编码器/联轴/负载偏心 (随速增长, 高速主导)
  2 阶      -> 码盘倾斜 / 联轴不对中
  pp 阶     -> 1×elec: 电流采样偏置 或 编码器电周期一次谐波
  2*pp 阶   -> 2×elec: 相增益失配 (+ 磁极侧齿槽)
  Ns 阶     -> 定子槽数 (公差齿槽, 定子侧不对称)
理想齿槽在 LCM(Ns, 2*pp) 阶 (24S-26P => 312 阶), 一般远超奈奎斯特, 看不到。

用法
====
  # 默认: 自动分段, 分析 torque, 出两张图
  python order_analysis.py lite_1.csv --pole-pairs 13

  # 交互框选: 在时间序列上拖拽选区, 对选区做阶次分析 (可重复)
  python order_analysis.py lite_1.csv --pole-pairs 13 --interactive

  # 自定义要标注的阶次族 + 分析列
  python order_analysis.py lite_1.csv --pole-pairs 13 \\
      --orders 1,2,13,24,26 --col torque --save out/run1

  # 不弹窗只存图
  python order_analysis.py lite_1.csv --pole-pairs 13 --save out/run1 --no-show
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
from scipy import signal


# ── 中文字体 ────────────────────────────────────────────────────────────────
def setup_cjk_font():
    """尽力选用可用的中文字体, 否则图中中文显示为方块 (不影响计算)。"""
    import matplotlib
    from matplotlib import font_manager

    # 主动注册常见 Noto CJK 路径 (容器/Linux 常见位置)
    for pat in ("/usr/share/fonts/opentype/noto/NotoSansCJK*.ttc",
                "/usr/share/fonts/**/NotoSansCJK*.ttc",
                "/usr/share/fonts/**/*CJK*.ttf"):
        for f in glob.glob(pat, recursive=True):
            try:
                font_manager.fontManager.addfont(f)
            except Exception:
                pass
    avail = {f.name for f in font_manager.fontManager.ttflist}
    for c in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
              "Noto Sans CJK JP", "Source Han Sans SC", "PingFang SC",
              "WenQuanYi Zen Hei"):
        if c in avail:
            matplotlib.rcParams["font.sans-serif"] = [c, "DejaVu Sans"]
            break
    else:
        print("[提示] 未找到中文字体, 图中中文可能显示为方块。")
    matplotlib.rcParams["axes.unicode_minus"] = False


# ── 数据 ────────────────────────────────────────────────────────────────────
def load(path):
    if not os.path.isfile(path):
        sys.exit(f"[错误] 找不到文件: {path}")
    df = pd.read_csv(path)
    for c in ("timestamp", "position"):
        if c not in df.columns:
            sys.exit(f"[错误] CSV 缺少 '{c}' 列。")
    t = df["timestamp"].to_numpy(dtype=float).copy()
    t -= t[0]
    return df, t


# ── 转速平台自动分段 ────────────────────────────────────────────────────────
def detect_plateaus(t, vel, min_revs, rel_tol):
    """把记录贪心切成恒速平台, 返回 [(i0, i1, mean_speed_rev_s), ...]。

    半开区间 [i0, i1)。相邻均值接近的段合并; 过短段丢弃。
    """
    va = np.abs(vel)
    n = len(va)
    raw = []
    i = 0
    while i < n:
        acc, cnt, j = va[i], 1, i + 1
        while j < n:
            mean = acc / cnt
            tolv = max(rel_tol * max(mean, 1e-6), 0.3)
            if abs(va[j] - mean) > tolv:
                break
            acc += va[j]
            cnt += 1
            j += 1
        raw.append([i, j, acc / cnt])
        i = j
    merged = []
    for seg in raw:
        if merged and abs(seg[2] - merged[-1][2]) <= max(
                rel_tol * merged[-1][2], 0.3):
            merged[-1][1] = seg[1]
            merged[-1][2] = float(np.mean(va[merged[-1][0]:seg[1]]))
        else:
            merged.append(seg)
    out = []
    for i0, i1, mean in merged:
        dur = t[i1 - 1] - t[i0]
        revs = mean * dur
        if revs >= min_revs and (i1 - i0) >= 32 and mean > 0.2:
            out.append((i0, i1, float(mean)))
    if not out:  # 兜底: 整段
        out = [(0, n, float(np.mean(va)))]
    return out


# ── 阶次谱 ──────────────────────────────────────────────────────────────────
def order_spectrum(pos_seg, y_seg):
    """按转角等间隔重采样后做 Hann 加窗 FFT。

    返回 (orders, amp)。amp 为幅度标定 (纯正弦幅值 A -> 峰≈A)。
    转角用 |Δposition| 累加, 对正反转都单调。
    """
    n = len(pos_seg)
    rev = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(pos_seg)))])
    total = rev[-1]
    if total <= 0 or n < 8:
        return np.array([]), np.array([])
    rev_uni = np.linspace(0.0, total, n)
    y_uni = np.interp(rev_uni, rev, y_seg)
    y_uni = signal.detrend(y_uni)          # 去线性趋势 + 直流
    win = np.hanning(n)
    spec = np.fft.rfft(y_uni * win)
    amp = 2.0 * np.abs(spec) / np.sum(win)
    if len(amp):
        amp[0] /= 2.0
    orders = np.fft.rfftfreq(n, d=total / n)   # 周/转 = 阶次
    return orders, amp


def band_peak(orders, amp, target, half=0.15):
    """在 target±half 阶范围内取峰值幅度; 无则 nan。"""
    if len(orders) == 0:
        return np.nan
    m = (orders >= target - half) & (orders <= target + half)
    return float(amp[m].max()) if np.any(m) else np.nan


# ── 归因标签 ────────────────────────────────────────────────────────────────
def order_label(o, pp, n_slots):
    if o == 1:
        return "1阶 (编码器/联轴偏心)"
    if o == 2:
        return "2阶 (码盘倾斜/不对中)"
    if o == pp:
        return f"{o}阶 (1×elec: 电流偏置/编码器谐波)"
    if o == 2 * pp:
        return f"{o}阶 (2×elec: 相增益失配)"
    if n_slots and o == n_slots:
        return f"{o}阶 (槽数: 公差齿槽)"
    return f"{o}阶"


ORDER_COLORS = ["#2ca02c", "#1f77b4", "#ff7f0e", "#d62728", "#9467bd",
                "#8c564b", "#e377c2", "#7f7f7f"]
ORDER_MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*"]


# ── 频谱对比图渲染 (供交互模式与非交互复用) ────────────────────────────────
def render_order_overlay(fig, specs, fam, track, col, pp, ns, order_max,
                         title_suffix=""):
    """把已累积的多段阶次谱叠加渲染到给定 figure。

    specs = [(speed, orders, amp), ...] 按加入顺序;
    fam   = {order: [该阶次在各段的幅值]} 与 specs 顺序对应。
    上图: 各段阶次谱叠加; 下图: 各阶次幅值 vs 转速。
    """
    import matplotlib.pyplot as plt

    fig.clf()
    axes = fig.subplots(2, 1, gridspec_kw={"height_ratios": [1.4, 1]})
    speeds = [sp for sp, _, _ in specs]
    cmap = plt.cm.viridis(np.linspace(0, 0.9, max(len(specs), 1)))

    ax = axes[0]
    for (sp, o, a), c in zip(specs, cmap):
        ax.semilogy(o, np.maximum(a, 1e-6), lw=0.8, color=c,
                    label=f"{sp:.2f} rev/s")
    for k, target in enumerate(track):
        ax.axvline(target, color=ORDER_COLORS[k % len(ORDER_COLORS)],
                   ls="--", lw=1.0, alpha=0.7)
    ax.set_xlim(0, order_max)
    ax.set_xlabel("机械阶次 order (周/转)")
    ax.set_ylabel(f"{col} 幅值")
    ax.set_title(f"{col} 阶次谱叠加 ({len(specs)} 段){title_suffix}\n"
                 f"竖线标注: "
                 f"{', '.join(order_label(o, pp, ns) for o in track)}")
    if specs:
        ax.legend(fontsize=8, ncol=4)
    ax.grid(alpha=0.3, which="both")

    ax = axes[1]
    order_speed = np.argsort(speeds) if speeds else []
    sp_sorted = [speeds[i] for i in order_speed]
    for k, target in enumerate(track):
        vals = [fam[target][i] for i in order_speed]
        ax.plot(sp_sorted, vals,
                marker=ORDER_MARKERS[k % len(ORDER_MARKERS)], lw=1.2,
                color=ORDER_COLORS[k % len(ORDER_COLORS)],
                label=order_label(target, pp, ns))
    ax.set_xlabel("转速 (rev/s)")
    ax.set_ylabel(f"该阶次 {col} 幅值")
    ax.set_title("各阶次分量幅值 vs 转速")
    if track:
        ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()


# ── 交互式框选分析 ──────────────────────────────────────────────────────────
def interactive_select(t, pos, vel, y, col, track, pp, ns, args):
    """在时间序列上拖拽框选多个区间, 累积对比其阶次谱。

    每选一段就把该段阶次谱叠加进对比窗口, 并更新"各阶次幅值 vs 转速"
    曲线——这正是阶次分析的核心用途: 跨转速对比同一阶次分量。
    """
    import matplotlib.pyplot as plt
    from matplotlib.widgets import SpanSelector

    show_data = [("velocity", vel)]
    if col != "velocity":
        show_data.append((col, y))

    fig, axes = plt.subplots(
        len(show_data), 1, sharex=True,
        figsize=(11, 2.2 * len(show_data)), squeeze=False)
    axes = axes[:, 0]
    for ax, (label, values) in zip(axes, show_data):
        ax.plot(t, values, lw=0.8)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("时间 t (s) — 在最上方子图拖拽选择分析区间(可多次)")
    fig.suptitle("交互框选阶次分析：多次框选不同转速段, 自动叠加对比 "
                 "(关闭窗口结束)")
    fig.tight_layout()

    # 累积所有选区的结果
    specs = []                       # [(speed, orders, amp), ...]
    fam = {o: [] for o in track}     # order -> [各段该阶次幅值]
    state = {"specfig": None, "selection": 0, "spans": []}

    def on_select(xmin, xmax):
        i0 = max(0, int(np.searchsorted(t, xmin, side="left")))
        i1 = min(len(t), int(np.searchsorted(t, xmax, side="right")))
        if i1 - i0 < 32:
            print(f"[选择过短] 仅 {i1 - i0} 点，至少需要 32 点。")
            return

        orders, amp = order_spectrum(pos[i0:i1], y[i0:i1])
        if len(orders) == 0:
            print("[无法分析] 选区内没有足够的转角变化。")
            return

        speed = float(np.mean(np.abs(vel[i0:i1])))
        revs = float(np.sum(np.abs(np.diff(pos[i0:i1]))))
        idx = len(specs)
        specs.append((speed, orders, amp))
        print("=" * 70)
        print(f"选区 #{idx}: t=[{t[i0]:.3f}, {t[i1 - 1]:.3f}]s  "
              f"{i1 - i0}点  ~{revs:.2f}转  转速≈{speed:.3f} rev/s")
        row = []
        for target in track:
            value = band_peak(orders, amp, target)
            fam[target].append(value)
            row.append(f"{target}:{value:.4f}"
                       if np.isfinite(value) else f"{target}:  -  ")
        print("  " + "  ".join(row))

        # 标注时间序列上已选中的区间
        for ax in axes:
            ax.axvspan(t[i0], t[i1 - 1], color="tab:orange", alpha=0.12)
            ax.annotate(f"#{idx}\n{speed:.1f}", xy=(t[i0], 0),
                        xycoords=("data", "axes fraction"),
                        xytext=(2, 2), textcoords="offset points",
                        fontsize=7, color="tab:orange")
        fig.canvas.draw_idle()

        if state["specfig"] is None:
            state["specfig"] = plt.figure(figsize=(11, 8.5))
        render_order_overlay(state["specfig"], specs, fam, track, col,
                             pp, ns, args.order_max)

        if args.save:
            out = f"{args.save}_order_interactive.png"
            os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
            state["specfig"].savefig(out, dpi=130)
            print(f"[已保存] {out} (含全部 {len(specs)} 段)")

        state["specfig"].canvas.draw_idle()
        try:
            state["specfig"].show()
        except Exception:
            pass

    span = SpanSelector(
        axes[0], on_select, "horizontal", useblit=True,
        props=dict(alpha=0.2, facecolor="tab:orange"),
        interactive=True, drag_from_anywhere=True)
    fig._span_selector = span
    print("[交互] 已打开选择窗口：在最上方子图反复框选不同转速段, "
          "对比窗口会自动叠加。")
    plt.show()


# ── 主流程 ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="moteus servo_stats 阶次域 torque 分析",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("csv", help="servo_stats CSV 路径")
    ap.add_argument("--pole-pairs", type=int, default=13, help="极对数, 默认 13")
    ap.add_argument("--slots", type=int, default=24,
                    help="定子槽数 (用于阶次标注), 默认 24; 0=不标")
    ap.add_argument("--col", default="torque", help="分析列, 默认 torque")
    ap.add_argument("--orders", default=None,
                    help="要追踪/标注的阶次, 逗号分隔; "
                         "默认 1,2,pp,2*pp,slots")
    ap.add_argument("--order-max", type=float, default=32.0,
                    help="阶次谱横轴上限, 默认 32")
    ap.add_argument("--min-revs", type=float, default=5.0,
                    help="平台最少转数, 默认 5")
    ap.add_argument("--seg-tol", type=float, default=0.08,
                    help="平台转速相对容差, 默认 0.08")
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="交互模式: 在时间序列上拖拽框选区间做阶次分析")
    ap.add_argument("--save", default=None, help="图片保存前缀, 如 out/run1")
    ap.add_argument("--no-show", action="store_true", help="不弹窗")
    args = ap.parse_args()

    if args.interactive and args.no_show:
        ap.error("--interactive 需要图形窗口，不能与 --no-show 同时使用")

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

    df, t = load(args.csv)
    if args.col not in df.columns:
        sys.exit(f"[错误] 找不到列 '{args.col}'; 可用: {list(df.columns)}")
    if "velocity" not in df.columns:
        sys.exit("[错误] 需要 velocity 列做转速分段。")

    pos = df["position"].to_numpy(dtype=float).copy()
    vel = df["velocity"].to_numpy(dtype=float).copy()
    y = df[args.col].to_numpy(dtype=float).copy()

    pp, ns = args.pole_pairs, args.slots
    if args.orders:
        track = [int(x) for x in args.orders.split(",")]
    else:
        track = sorted(set([1, 2, pp, 2 * pp] + ([ns] if ns else [])))

    setup_cjk_font()
    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if args.interactive:
        print("=" * 70)
        print(f"交互阶次域分析: {args.csv}  列={args.col}  pp={pp}  槽数={ns}")
        print(f"追踪阶次: {track}")
        print("=" * 70)
        interactive_select(t, pos, vel, y, args.col, track, pp, ns, args)
        return

    segs = detect_plateaus(t, vel, args.min_revs, args.seg_tol)

    print("=" * 70)
    print(f"阶次域分析: {args.csv}  列={args.col}  pp={pp}  槽数={ns}")
    print(f"检出转速平台: {len(segs)} 个  追踪阶次: {track}")
    print("=" * 70)

    speeds, specs = [], []
    fam = {o: [] for o in track}
    for i0, i1, sp in segs:
        o, a = order_spectrum(pos[i0:i1], y[i0:i1])
        if len(o) == 0:
            continue
        speeds.append(sp)
        specs.append((sp, o, a))
        row = []
        for tgt in track:
            v = band_peak(o, a, tgt)
            fam[tgt].append(v)
            row.append(f"{tgt}:{v:.4f}" if v == v else f"{tgt}:  -  ")
        print(f"  {sp:5.1f} rev/s ({i1 - i0:5d}点)  " + "  ".join(row))

    # ── 图 ──
    cmap = plt.cm.viridis(np.linspace(0, 0.9, max(len(specs), 1)))
    fig, axes = plt.subplots(2, 1, figsize=(11, 8.5),
                             gridspec_kw={"height_ratios": [1.4, 1]})

    ax = axes[0]
    for (sp, o, a), c in zip(specs, cmap):
        ax.semilogy(o, np.maximum(a, 1e-6), lw=0.7, color=c,
                    label=f"{sp:.0f} rev/s")
    for k, o in enumerate(track):
        ax.axvline(o, color=ORDER_COLORS[k % len(ORDER_COLORS)],
                   ls="--", lw=1.0, alpha=0.7)
    ax.set_xlim(0, args.order_max)
    ax.set_xlabel("机械阶次 order (周/转)")
    ax.set_ylabel(f"{args.col} 幅值")
    ax.set_title(f"{args.col} 阶次谱 (按转子角度重采样, 与时间戳无关)\n"
                 f"竖线标注: {', '.join(order_label(o, pp, ns) for o in track)}")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(alpha=0.3, which="both")

    ax = axes[1]
    for k, o in enumerate(track):
        ax.plot(speeds, fam[o],
                marker=ORDER_MARKERS[k % len(ORDER_MARKERS)], lw=1.2,
                color=ORDER_COLORS[k % len(ORDER_COLORS)],
                label=order_label(o, pp, ns))
    ax.set_xlabel("转速 (rev/s)")
    ax.set_ylabel(f"该阶次 {args.col} 幅值")
    ax.set_title("各阶次分量幅值 vs 转速")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    if args.save:
        out = f"{args.save}_order.png"
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        fig.savefig(out, dpi=130)
        print(f"[已保存] {out}")
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
