#!/usr/bin/env python3
# Copyright 2026 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""离线一阶 IIR 低通滤波 — 对齐 VESC ~500 Hz 截止.

对高速率电流 CSV (``d_A``/``q_A`` 或 ``cur1_A``/``cur2_A``/``cur3_A``)
按实测采样率折算等效 α, 使截止频率对齐 VESC ``α=0.1 @ 30 kHz``
换算出的 ~500 Hz。

    alpha_equiv = 1 - exp(-2*pi*fc / fs)

注意: 若日志 fs 只有数百 Hz, 默认 500 Hz 已超过 Nyquist, 曲线几乎不变。
脚本会自动把 fc 钳到 ``0.2·fs``; 需要原值时加 ``--force-fc``。

用法示例
========
    # 默认处理 d_A,q_A, 打印 stddev, 弹窗对比
    python scripts/iir_lowpass_currents.py logs/2.csv

    # 相电流列
    python scripts/iir_lowpass_currents.py logs/xxx.csv --cols cur1_A,cur2_A,cur3_A

    # 自定义截止频率 / 写出滤波后 CSV
    python scripts/iir_lowpass_currents.py logs/2.csv --fc 50 \\
        --out logs/2_iir.csv --no-show

    # 只统计不绘图
    python scripts/iir_lowpass_currents.py logs/2.csv --no-plot
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

# VESC α=0.1 @ 30 kHz → fc ≈ -ln(1-α)·fs/(2π) ≈ 507 Hz
DEFAULT_FC_HZ = 500.0
DEFAULT_COLS = ("d_A", "q_A")
FALLBACK_COLS = ("cur1_A", "cur2_A", "cur3_A")


def load_log(path: str) -> Tuple[pd.DataFrame, np.ndarray]:
    if not os.path.isfile(path):
        sys.exit(f"[错误] 找不到文件: {path}")
    df = pd.read_csv(path)
    if "timestamp" not in df.columns:
        sys.exit("[错误] CSV 中缺少 'timestamp' 列, 无法估计采样率。")
    t = df["timestamp"].to_numpy(dtype=float)
    t = t - t[0]
    return df, t


def sampling_rate(t: np.ndarray) -> Tuple[float, float]:
    """返回 (fs_Hz, dt_median_s)。"""
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    if dt.size == 0:
        sys.exit("[错误] 时间戳无效, 无法估计采样率。")
    dt_med = float(np.median(dt))
    return 1.0 / dt_med, dt_med


def alpha_for_fc(fs: float, fc: float) -> float:
    """一阶 IIR 低通: y[n] = α x[n] + (1-α) y[n-1], 对应截止 fc。"""
    if fs <= 0:
        sys.exit(f"[错误] 无效采样率 fs={fs}")
    if fc <= 0:
        sys.exit(f"[错误] 截止频率必须 > 0, 得到 fc={fc}")
    alpha = 1.0 - float(np.exp(-2.0 * np.pi * fc / fs))
    return float(np.clip(alpha, 0.0, 1.0))


def resolve_fc(fs: float, requested_fc: float, force: bool) -> float:
    """把目标截止钳到可滤波范围。

    VESC 等效 ~500 Hz 假定 fs 远高于此 (如 30 kHz)。若日志采样率只在
    数百 Hz, 500 Hz 已超过 Nyquist, α→1, 曲线几乎不变。默认钳到
    ``0.2 * fs`` (约 Nyquist 的 40%), 可用 ``--force-fc`` 强制原值。
    """
    nyq = fs / 2.0
    safe = 0.2 * fs
    if requested_fc < nyq:
        return requested_fc
    if force:
        print(
            f"[警告] fc={requested_fc:.1f} Hz ≥ Nyquist={nyq:.1f} Hz "
            f"(fs≈{fs:.1f}), α≈1, 几乎不滤波 (--force-fc)。",
            file=sys.stderr,
        )
        return requested_fc
    print(
        f"[提示] 请求 fc={requested_fc:.1f} Hz ≥ Nyquist={nyq:.1f} Hz "
        f"(本日志 fs≈{fs:.1f} Hz)。\n"
        f"       VESC 的 ~500 Hz 截止需要数 kHz 以上采样; "
        f"当前自动改用 fc={safe:.1f} Hz (=0.2·fs)。\n"
        f"       若坚持原值: 加 --force-fc; 或提高 ControlManager 采样率后再记。",
        file=sys.stderr,
    )
    return safe


def iir_filter(x: np.ndarray, alpha: float) -> np.ndarray:
    """因果一阶 IIR 低通 (与在线实现一致)。"""
    x = np.asarray(x, dtype=float)
    y = np.empty_like(x)
    if x.size == 0:
        return y
    # NaN 处保持 NaN, 有效段内递推
    y[0] = x[0]
    a = float(alpha)
    b = 1.0 - a
    for i in range(1, len(x)):
        xi = x[i]
        if not np.isfinite(xi):
            y[i] = xi
            continue
        prev = y[i - 1]
        if not np.isfinite(prev):
            y[i] = xi
        else:
            y[i] = a * xi + b * prev
    return y


def pick_columns(df: pd.DataFrame, requested: Optional[Sequence[str]]) -> List[str]:
    if requested:
        cols = [c.strip() for c in requested if c.strip()]
    else:
        if all(c in df.columns for c in DEFAULT_COLS):
            cols = list(DEFAULT_COLS)
        elif all(c in df.columns for c in FALLBACK_COLS):
            cols = list(FALLBACK_COLS)
        else:
            candidates = [
                c for c in list(DEFAULT_COLS) + list(FALLBACK_COLS)
                if c in df.columns
            ]
            if not candidates:
                sys.exit(
                    "[错误] CSV 中无 d_A/q_A 或 cur1_A/cur2_A/cur3_A, "
                    "请用 --cols 指定。"
                )
            cols = candidates

    missing = [c for c in cols if c not in df.columns]
    if missing:
        sys.exit(f"[错误] 缺少列: {', '.join(missing)}")
    return cols


def finite_std(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.std(x)) if x.size else float("nan")


def plot_compare(
    t: np.ndarray,
    raw: dict,
    filt: dict,
    fs: float,
    fc: float,
    alpha: float,
    save: Optional[str],
    show: bool,
) -> None:
    import matplotlib.pyplot as plt

    n = len(raw)
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, 2.4 * n),
                             constrained_layout=True)
    if n == 1:
        axes = [axes]

    for ax, name in zip(axes, raw):
        ax.plot(t, raw[name], lw=0.6, alpha=0.55, label=f"{name} raw", color="#888")
        ax.plot(t, filt[name], lw=0.9, label=f"{name} IIR", color="#1f77b4")
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("t (s)")
    # English title avoids missing-CJK-glyph warnings on stock DejaVu Sans.
    fig.suptitle(
        f"1st-order IIR LPF  fc={fc:.1f} Hz  α={alpha:.4f}  fs≈{fs:.1f} Hz",
        fontsize=11,
    )
    if save:
        os.makedirs(os.path.dirname(save) or ".", exist_ok=True)
        fig.savefig(save, dpi=150)
        print(f"[保存] {save}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="离线一阶 IIR 低通 (对齐 VESC ~500 Hz 截止)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("csv", help="输入 CSV (需含 timestamp + 电流列)")
    ap.add_argument(
        "--cols", default=None,
        help="要滤波的列, 逗号分隔 (默认: d_A,q_A 或 cur1_A,cur2_A,cur3_A)",
    )
    ap.add_argument(
        "--fc", type=float, default=DEFAULT_FC_HZ, metavar="HZ",
        help=f"目标截止频率 Hz (默认 {DEFAULT_FC_HZ:g}, VESC α=0.1@30kHz 等效)",
    )
    ap.add_argument(
        "--force-fc", action="store_true",
        help="允许 fc≥Nyquist (几乎不滤波); 默认自动钳到 0.2·fs",
    )
    ap.add_argument(
        "--alpha", type=float, default=None,
        help="直接指定 α (0~1); 若给出则忽略 --fc 换算",
    )
    ap.add_argument(
        "--out", default=None, metavar="PATH",
        help="写出带 *_iir 列的新 CSV (默认不写)",
    )
    ap.add_argument("--plot", dest="do_plot", action="store_true", default=True,
                    help="绘制滤波前后对比 (默认开)")
    ap.add_argument("--no-plot", dest="do_plot", action="store_false",
                    help="不绘图")
    ap.add_argument("--save", default=None, metavar="PNG",
                    help="保存对比图路径")
    ap.add_argument("--show", dest="show", action="store_true", default=True)
    ap.add_argument("--no-show", dest="show", action="store_false",
                    help="不弹窗 (配合 --save)")
    args = ap.parse_args()

    df, t = load_log(args.csv)
    cols = pick_columns(
        df,
        args.cols.split(",") if args.cols else None,
    )
    fs, dt_med = sampling_rate(t)

    if args.alpha is not None:
        alpha = float(np.clip(args.alpha, 0.0, 1.0))
        fc_eff = -np.log(max(1.0 - alpha, 1e-15)) * fs / (2.0 * np.pi)
        fc = float(fc_eff)
    else:
        fc = resolve_fc(fs, args.fc, force=args.force_fc)
        alpha = alpha_for_fc(fs, fc)

    print(f"文件: {args.csv}")
    print(f"样本: {len(t)}  时长: {t[-1]:.3f} s")
    print(f"采样率 fs ≈ {fs:.2f} Hz  (median Δt = {dt_med*1e3:.3f} ms)")
    print(f"Nyquist = {fs/2:.2f} Hz")
    print(f"使用 fc = {fc:.2f} Hz  →  α = {alpha:.6f}")
    print(f"列: {', '.join(cols)}")
    print("-" * 56)

    raw = {}
    filt = {}
    for name in cols:
        x = df[name].to_numpy(dtype=float)
        y = iir_filter(x, alpha)
        raw[name] = x
        filt[name] = y
        out_name = f"{name}_iir"
        df[out_name] = y
        print(
            f"{name:12s}  stddev 前={finite_std(x):.4f}  "
            f"后={finite_std(y):.4f}  → {out_name}"
        )

    if args.out:
        out_dir = os.path.dirname(args.out)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        df.to_csv(args.out, index=False)
        print(f"[写出] {args.out}")

    if args.do_plot or args.save:
        plot_compare(
            t, raw, filt, fs, fc, alpha,
            save=args.save,
            show=args.show and args.do_plot,
        )


if __name__ == "__main__":
    main()
