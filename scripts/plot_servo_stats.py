#!/usr/bin/env python3
# Copyright 2026 SN
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""moteus servo_stats 日志绘图 + FFT 电机异常分析工具.

功能
====
1. 通用绘图: 从 servo_stats CSV 中任意选择列, 画时间序列曲线。
2. 频谱分析: 针对轮询采集的 servo_stats 信号 (默认 d_A, q_A, velocity,
   position) 做 FFT / Lomb-Scargle, 并把谱峰对齐到三个参考频率族:

       1×mech = rev/s               (机械一次)
       1×elec = pole_pairs × rev/s  (电一次)
       2×elec = 2 × pole_pairs × rev/s (电二次)

   谱峰落在哪一族, 直接指认原因:

       1×mech  -> 编码器偏心 (encoder eccentricity)
       1×elec  -> 电流偏置 / 编码器一次谐波 (current offset / encoder 1st harmonic)
       2×elec  -> 相增益失配 (phase gain mismatch)

关于混叠 (重要)
==============
servo_stats 是软件轮询, 采样率通常只有几十 Hz (本例 ~66 Hz)。而
电频率 = 极对数 × 机械频率, 例如 7 极对 × 5 rev/s = 35 Hz, 已经
超过奈奎斯特频率 (fs/2 ≈ 33 Hz)。因此 1×elec / 2×elec 会以"混叠
(alias)"频率出现在低频端。本脚本对每个参考频率同时给出真实频率与
折叠后的混叠频率, 并在混叠频率处检索谱峰, 从而正确对齐。若要避免
混叠, 请使用控制器高频记录 (register poll / diagnostic) 而非低频轮询。

用法示例
========
    # 列出所有可用列
    python scripts/plot_servo_stats.py logs/xxx.csv --list

    # 画若干列的时间序列
    python scripts/plot_servo_stats.py logs/xxx.csv --plot d_A,q_A,velocity

    # 交互框选: 弹出时间序列, 鼠标拖拽选区即对该区间做 FFT 诊断 (可重复)
    python scripts/plot_servo_stats.py logs/xxx.csv --interactive --pole-pairs 7

    # 默认信号做异常诊断 FFT (需给定极对数)
    python scripts/plot_servo_stats.py logs/xxx.csv --pole-pairs 7

    # 指定分析列 + Lomb-Scargle (可探测超奈奎斯特频率, 缓解混叠)
    python scripts/plot_servo_stats.py logs/xxx.csv --fft q_A,d_A \
        --pole-pairs 7 --method ls

    # 保存图片而不弹窗
    python scripts/plot_servo_stats.py logs/xxx.csv --pole-pairs 7 \
        --save out/analysis --no-show
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
from scipy import signal


# ── 诊断映射 ────────────────────────────────────────────────────────────────
# family_key -> (中文标签, 归因说明)
FAMILY_LABEL = {
    "1xmech": "1×mech (机械一次)",
    "1xelec": "1×elec (电一次)",
    "2xelec": "2×elec (电二次)",
}
FAMILY_CAUSE = {
    "1xmech": "编码器偏心 (encoder eccentricity)",
    "1xelec": "电流偏置 / 编码器一次谐波 (current offset or encoder 1st harmonic)",
    "2xelec": "相增益失配 (phase gain mismatch)",
}

# 默认做频谱诊断的信号列
DEFAULT_FFT_COLS = ["d_A", "q_A", "velocity", "position"]


# ── 数据加载 ────────────────────────────────────────────────────────────────
def load_log(path):
    """读取 CSV, 返回 (DataFrame, t) ; t 为从 0 起算的相对时间(秒)。"""
    if not os.path.isfile(path):
        sys.exit(f"[错误] 找不到文件: {path}")
    df = pd.read_csv(path)
    if "timestamp" not in df.columns:
        sys.exit("[错误] CSV 中缺少 'timestamp' 列, 无法建立时间轴。")
    t = df["timestamp"].to_numpy(dtype=float)
    t = t - t[0]
    return df, t


def sampling_info(t):
    """根据时间戳返回 (fs_eff, dt_median, dt_min, dt_max)。"""
    dt = np.diff(t)
    dt = dt[dt > 0]
    dt_med = float(np.median(dt))
    return 1.0 / dt_med, dt_med, float(dt.min()), float(dt.max())


# ── 频谱工具 ────────────────────────────────────────────────────────────────
def fold_alias(f, fs):
    """把频率 f 折叠到 [0, fs/2] 区间 (采样混叠的镜像折叠)。"""
    nyq = fs / 2.0
    f_mod = np.mod(f, fs)
    return fs - f_mod if f_mod > nyq else f_mod


def detrend_signal(y, mode):
    if mode == "none":
        return y - np.mean(y)  # 至少去直流, 否则 0Hz 泄漏
    if mode == "constant":
        return signal.detrend(y, type="constant")
    if mode == "linear":
        return signal.detrend(y, type="linear")
    raise ValueError(mode)


def spectrum_fft(t, y, fs, detrend="linear"):
    """非均匀采样 -> 线性插值到均匀网格 -> 加窗 FFT 幅度谱.

    返回 (freqs, amp), amp 已标定为幅度 (纯正弦幅值 A -> 峰值≈A)。
    """
    n = len(t)
    t_uni = np.linspace(t[0], t[-1], n)
    y_uni = np.interp(t_uni, t, y)
    y_uni = detrend_signal(y_uni, detrend)

    win = np.hanning(n)
    yw = y_uni * win
    spec = np.fft.rfft(yw)
    # 修复: 重采样网格的真实间距由区间长度决定, 而非外部传入的 fs
    # (fs=1/median(dt) 会因 dt 抖动/间隙与 mean(dt) 偏差, 造成频率轴整体缩放误差)
    dt_grid = (t_uni[-1] - t_uni[0]) / (n - 1) if n > 1 else 1.0 / fs
    freqs = np.fft.rfftfreq(n, d=dt_grid)
    # 幅度标定: 相干增益 = sum(win); 单边谱乘 2
    amp = 2.0 * np.abs(spec) / np.sum(win)
    if len(amp) > 0:
        amp[0] /= 2.0  # 直流分量不乘 2
    return freqs, amp


def spectrum_ls(t, y, fmax, detrend="linear", n_freq=4000):
    """Lomb-Scargle 周期图, 直接处理非均匀采样, 可探测到 fmax (可超奈奎斯特).

    返回 (freqs, amp), amp 为等效幅度 (近似标定, 用于相对比较)。
    """
    y = detrend_signal(y, detrend)
    freqs = np.linspace(fmax / n_freq, fmax, n_freq)
    ang = 2.0 * np.pi * freqs
    power = signal.lombscargle(t, y, ang, normalize=False)
    # 幅度近似: A ≈ 2*sqrt(P/N)
    amp = 2.0 * np.sqrt(np.maximum(power, 0.0) / len(t))
    return freqs, amp


def peak_near(freqs, amp, f_target, half_bw):
    """在 [f_target-half_bw, f_target+half_bw] 内取幅度最大的峰。

    返回 (f_peak, amp_peak); 若目标频率超出谱范围返回 (nan, nan)。
    """
    if f_target < freqs[0] - half_bw or f_target > freqs[-1] + half_bw:
        return np.nan, np.nan
    mask = (freqs >= f_target - half_bw) & (freqs <= f_target + half_bw)
    if not np.any(mask):
        idx = int(np.argmin(np.abs(freqs - f_target)))
        return float(freqs[idx]), float(amp[idx])
    band_f = freqs[mask]
    band_a = amp[mask]
    j = int(np.argmax(band_a))
    return float(band_f[j]), float(band_a[j])


def noise_floor(amp):
    """用中位数作为噪声基底估计 (对稀疏尖峰稳健)。"""
    med = float(np.median(amp))
    return med if med > 0 else float(np.mean(amp) + 1e-12)


def find_top_peaks(freqs, amp, top, min_freq):
    """返回按幅度排序的前 top 个局部极大峰 [(f, a), ...]。"""
    if len(amp) < 3:
        return []
    # 局部极大
    idx = np.where(
        (amp[1:-1] >= amp[:-2]) & (amp[1:-1] >= amp[2:])
    )[0] + 1
    idx = idx[freqs[idx] >= min_freq]
    if len(idx) == 0:
        return []
    order = idx[np.argsort(amp[idx])[::-1]][:top]
    return [(float(freqs[i]), float(amp[i])) for i in order]


# ── 转速分段 (处理一张表含多个转速) ─────────────────────────────────────────
def build_angle(df, t):
    """构造严格非减的累计机械转数(rev), 供阶次(order)重采样使用。

    优先用 position(编码器积分, 单位 rev); 否则由 velocity 数值积分。
    取 |Δposition| 的累加, 使角度对正反转都单调, 阶次分析才成立。
    """
    if "position" in df.columns:
        pos = df["position"].to_numpy(dtype=float)
        return np.concatenate([[0.0], np.cumsum(np.abs(np.diff(pos)))])
    v = df["velocity"].to_numpy(dtype=float)
    dt = np.diff(t)
    step = 0.5 * (np.abs(v[1:]) + np.abs(v[:-1])) * dt
    return np.concatenate([[0.0], np.cumsum(step)])


def detect_segments(t, v, rev, min_revs, rel_tol):
    """把记录贪心切分成"恒速段"。

    返回 (kept, dropped):
      kept    = [(i0, i1, mean_speed), ...] 通过 min_revs 的段;
      dropped = [(i0, i1, mean_speed, revs), ...] 因转数不足被丢弃的候选段
                (用于提示用户是否需要下调 --min-revs)。
    相邻均值相近的段会被合并; 半开区间 [i0,i1)。
    """
    va = np.abs(v)
    n = len(va)
    raw = []
    i = 0
    while i < n:
        acc, cnt, j = va[i], 1, i + 1
        while j < n:
            mean = acc / cnt
            tolv = max(rel_tol * max(mean, 1e-6), 0.3)  # 绝对下限 0.3 rev/s
            if abs(va[j] - mean) > tolv:
                break
            acc += va[j]
            cnt += 1
            j += 1
        raw.append([i, j, acc / cnt])
        i = j
    # 合并相邻且均值接近的段
    merged = []
    for seg in raw:
        if merged and abs(seg[2] - merged[-1][2]) <= max(
                rel_tol * merged[-1][2], 0.3):
            merged[-1][1] = seg[1]
            i0 = merged[-1][0]
            merged[-1][2] = float(np.mean(va[i0:seg[1]]))
        else:
            merged.append(seg)
    # 过滤过短段 (按转数)。转速≈0 的静止段无法诊断, 单独排除且不提示。
    out, dropped = [], []
    for i0, i1, mean in merged:
        revs = rev[i1 - 1] - rev[i0]
        if revs >= min_revs and (i1 - i0) >= 16:
            out.append((i0, i1, float(mean)))
        elif mean > 0.2 and revs > 0.5:  # 明显在转、只是太短
            dropped.append((i0, i1, float(mean), float(revs)))
    if not out:  # 兜底: 整段
        out = [(0, n, float(np.mean(va)))]
    return out, dropped


# ── 分析单信号 (时间域 Hz 或 阶次域 order 通用) ──────────────────────────────
def analyze_signal(label, x, y, fs_domain, fundamental, pole_pairs, spec,
                   detrend, tol, max_bin, top, unit):
    """对单个信号做频谱分析, 返回 (freqs, amp, references, report_lines)。

    x/fs_domain/fundamental 的量纲随域而变:
      时间域: x=时间(s), fs_domain=采样率(Hz), fundamental=f_mech(Hz)=rev/s
      阶次域: x=转数(rev), fs_domain=每转采样点数, fundamental=1(阶)
    """
    nyq = fs_domain / 2.0
    f_1mech = 1.0 * fundamental
    f_1elec = pole_pairs * fundamental
    f_2elec = 2.0 * pole_pairs * fundamental

    if spec == "fft":
        freqs, amp = spectrum_fft(x, y, fs_domain, detrend=detrend)
    else:
        freqs, amp = spectrum_ls(x, y, max_bin, detrend=detrend)

    if len(freqs) < 2:
        return freqs, amp, [], [f"  [跳过] {label}: 有效样本不足。"]

    df_bin = float(freqs[1] - freqs[0])
    floor = noise_floor(amp)
    # 匹配容差以基频为尺度 (各族均为 fundamental 的整数倍)。
    half_bw = max(tol * fundamental, 2.0 * df_bin)

    references = []
    for key, f_true in (("1xmech", f_1mech), ("1xelec", f_1elec),
                        ("2xelec", f_2elec)):
        if spec == "fft" and f_true > nyq:
            f_search = fold_alias(f_true, fs_domain)
            aliased = True
        else:
            f_search = f_true
            aliased = False
        f_peak, a_peak = peak_near(freqs, amp, f_search, half_bw)
        snr = a_peak / floor if a_peak == a_peak else np.nan
        references.append(
            dict(key=key, f_true=f_true, f_search=f_search, aliased=aliased,
                 f_peak=f_peak, amp=a_peak, snr=snr))

    u = unit
    lines = []
    lines.append(f"  信号 {label}:  谱估计={spec}  去趋势={detrend}  "
                 f"分辨率Δ={df_bin:.3f}{u}  噪声基底={floor:.4g}")
    lines.append(f"    族        真实({u})  检索({u})   峰({u})   幅度      "
                 f"SNR    混叠  归因")
    valid = [r for r in references if r["snr"] == r["snr"]]
    dominant = max(valid, key=lambda r: r["snr"]) if valid else None
    for r in references:
        alias_tag = "是" if r["aliased"] else "否"
        if r["snr"] != r["snr"]:
            lines.append(f"    {FAMILY_LABEL[r['key']]:<9} {r['f_true']:>8.3f}  "
                         f"   超范围, 未检索")
            continue
        star = " *" if (dominant is not None and r is dominant) else "  "
        lines.append(
            f"    {FAMILY_LABEL[r['key']]:<9} {r['f_true']:>8.3f}  "
            f"{r['f_search']:>8.3f}  {r['f_peak']:>8.3f}  {r['amp']:>8.4g}  "
            f"{r['snr']:>6.2f}   {alias_tag:<3}{star}{FAMILY_CAUSE[r['key']]}")

    if dominant is not None:
        lines.append(
            f"    => 主导族: {FAMILY_LABEL[dominant['key']]}  "
            f"(SNR={dominant['snr']:.2f})  ==>  {FAMILY_CAUSE[dominant['key']]}")

    for i in range(len(references)):
        for j in range(i + 1, len(references)):
            ri, rj = references[i], references[j]
            if (ri["f_search"] == ri["f_search"]
                    and rj["f_search"] == rj["f_search"]
                    and abs(ri["f_search"] - rj["f_search"]) <= half_bw):
                lines.append(
                    f"    [歧义] {FAMILY_LABEL[ri['key']]} 与 "
                    f"{FAMILY_LABEL[rj['key']]} 混叠后重合 "
                    f"(~{ri['f_search']:.2f}{u}), 二者无法区分; "
                    f"建议 --method ls / --domain order / 提高采样率。")

    peaks = find_top_peaks(freqs, amp, top, min_freq=2.0 * df_bin)
    if peaks:
        lines.append(f"    幅度最高的 {len(peaks)} 个谱峰及归类:")
        for f_pk, a_pk in peaks:
            key = classify_peak(f_pk, references, half_bw)
            cause = FAMILY_CAUSE[key] if key else "(未落入任一参考族)"
            fam = FAMILY_LABEL[key] if key else "-"
            lines.append(f"      {f_pk:>8.3f}{u}  幅度={a_pk:>8.4g}  "
                         f"SNR={a_pk / floor:>5.2f}  -> {fam:<16}{cause}")

    return freqs, amp, references, lines


def classify_peak(f_peak, references, half_bw):
    """把一个谱峰归到最近的参考族 (按检索频率比较), 超出容差返回 None。"""
    best_key, best_d = None, None
    for r in references:
        if r["f_search"] != r["f_search"]:
            continue
        d = abs(f_peak - r["f_search"])
        if d <= half_bw and (best_d is None or d < best_d):
            best_d, best_key = d, r["key"]
    return best_key


# ── 绘图 ────────────────────────────────────────────────────────────────────
_FONT_READY = False


def _setup_cjk_font():
    """尽力选用系统里可用的中文字体, 否则图中中文会显示为方块。"""
    global _FONT_READY
    if _FONT_READY:
        return
    _FONT_READY = True
    import matplotlib
    from matplotlib import font_manager

    candidates = ["Microsoft YaHei", "SimHei", "SimSun", "PingFang SC",
                  "Noto Sans CJK SC", "Source Han Sans SC", "Arial Unicode MS",
                  "WenQuanYi Zen Hei"]
    avail = {f.name for f in font_manager.fontManager.ttflist}
    for c in candidates:
        if c in avail:
            matplotlib.rcParams["font.sans-serif"] = (
                [c] + list(matplotlib.rcParams.get("font.sans-serif", [])))
            matplotlib.rcParams["axes.unicode_minus"] = False
            return
    print("[提示] 未找到中文字体, 图中中文可能显示为方块 (不影响文字报告)。")


def plot_timeseries(df, t, cols, save, show):
    import matplotlib.pyplot as plt
    _setup_cjk_font()

    cols = [c for c in cols if c in df.columns]
    missing = [c for c in cols if c not in df.columns]
    if not cols:
        print("[警告] 没有可绘制的有效列。")
        return
    fig, axes = plt.subplots(len(cols), 1, sharex=True,
                             figsize=(11, 2.2 * len(cols)), squeeze=False)
    for ax, c in zip(axes[:, 0], cols):
        ax.plot(t, df[c].to_numpy(dtype=float), lw=0.8)
        ax.set_ylabel(c)
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("时间 t (s)")
    fig.suptitle("servo_stats 时间序列")
    fig.tight_layout()
    _finish(fig, save, "timeseries", show)


def _render_spectra(results, xlabel, title, fig=None):
    """把频谱结果画到(或清空重画)给定 figure; 返回该 figure。不做 show/save。"""
    import matplotlib.pyplot as plt

    n = len(results)
    if n == 0:
        return fig
    if fig is None:
        fig = plt.figure(figsize=(11, 2.6 * n))
    else:
        fig.clf()
        fig.set_size_inches(11, 2.6 * n)
    axes = fig.subplots(n, 1, squeeze=False)[:, 0]
    colors = {"1xmech": "#2ca02c", "1xelec": "#ff7f0e", "2xelec": "#d62728"}
    for ax, (col, freqs, amp, references) in zip(axes, results):
        ax.semilogy(freqs, np.maximum(amp, 1e-12), lw=0.8, color="#1f77b4")
        for r in references:
            fs_line = r["f_search"]
            if fs_line != fs_line:
                continue
            label = FAMILY_LABEL[r["key"]]
            if r["aliased"]:
                label += f"(混叠<-{r['f_true']:.1f})"
            ax.axvline(fs_line, color=colors[r["key"]], ls="--", lw=1.2,
                       label=label)
        ax.set_ylabel(f"{col}\n幅度")
        ax.grid(True, which="both", alpha=0.25)
        ax.legend(fontsize=7, loc="upper right")
    axes[-1].set_xlabel(xlabel)
    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_spectra(results, xlabel, title, save, tag, show):
    _setup_cjk_font()
    fig = _render_spectra(results, xlabel, title)
    if fig is None:
        return
    _finish(fig, save, tag, show)


def _finish(fig, save, tag, show):
    import matplotlib.pyplot as plt

    if save:
        out = f"{save}_{tag}.png"
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        fig.savefig(out, dpi=130)
        print(f"[已保存] {out}")
    if show:
        plt.show()
    else:
        plt.close(fig)


# ── 单区间诊断 (供分段循环与交互选区共用) ───────────────────────────────────
def diagnose_window(df, t, rev, i0, i1, fft_cols, args, fs, mean_v, label):
    """对索引区间 [i0,i1) 做频谱诊断; 打印报告并返回绘谱所需数据。

    返回 dict(results, xlabel, title, f_mech) 或 None(样本不足/无法分析)。
    """
    if i1 - i0 < 16:
        print(f"  [跳过] {label} 样本过少 ({i1 - i0} 点)。")
        return None
    t_seg = t[i0:i1]
    rev_seg = (rev[i0:i1] - rev[i0]) if rev is not None else None

    if args.rev_per_s is not None:
        f_mech = abs(args.rev_per_s)
    elif mean_v is not None:
        f_mech = abs(mean_v)
    elif "velocity" in df.columns:
        f_mech = float(np.mean(np.abs(
            df["velocity"].to_numpy(dtype=float)[i0:i1])))
    else:
        print(f"  [跳过] {label} 无 velocity 且未提供 --rev-per-s。")
        return None

    dur = t_seg[-1] - t_seg[0]
    revs = rev_seg[-1] if rev_seg is not None else f_mech * dur
    print(f"### {label} t=[{t_seg[0]:.2f},{t_seg[-1]:.2f}]s  {i1 - i0}点  "
          f"~{revs:.1f}转  转速≈{f_mech:.3f} rev/s")

    if args.domain == "order":
        if rev_seg is None or rev_seg[-1] <= 0:
            print("  [跳过] 无角度信息, 无法做阶次分析。")
            return None
        x = rev_seg
        spr = (i1 - i0) / rev_seg[-1]
        fs_domain, fundamental, unit = spr, 1.0, "阶"
        max_bin = 3.0 * 2.0 * args.pole_pairs
        xlabel = "阶次 order (每转周数)"
        nyq_dom = spr / 2.0
        print(f"  阶次域: 每转≈{spr:.1f}点  阶次奈奎斯特={nyq_dom:.2f}  "
              f"参考阶次: 1×mech=1, 1×elec={args.pole_pairs}, "
              f"2×elec={2 * args.pole_pairs}")
        if 2 * args.pole_pairs > nyq_dom:
            print("  警告: 电阶次超过阶次奈奎斯特, 将折叠检索; 可 --method ls。")
    else:
        x = t_seg
        fs_domain, fundamental, unit = fs, f_mech, "Hz"
        max_bin = args.max_freq if args.max_freq else \
            3.0 * 2.0 * args.pole_pairs * f_mech
        xlabel = "频率 f (Hz)"
        f_1e, f_2e = args.pole_pairs * f_mech, 2 * args.pole_pairs * f_mech
        print(f"  时间域: fs={fs:.2f}Hz 奈奎斯特={fs / 2:.2f}Hz  "
              f"f_mech={f_mech:.3f}Hz  1×elec={f_1e:.2f}Hz  2×elec={f_2e:.2f}Hz")
        if f_1e > fs / 2 or f_2e > fs / 2:
            print(f"  警告: 电频率超奈奎斯特, 混叠检索 (1×elec->"
                  f"{fold_alias(f_1e, fs):.2f}Hz, 2×elec->"
                  f"{fold_alias(f_2e, fs):.2f}Hz); 可 --method ls / "
                  f"--domain order。")

    results = []
    for col in fft_cols:
        y = df[col].to_numpy(dtype=float)[i0:i1]
        if not np.all(np.isfinite(y)):
            fill = float(np.nanmean(y)) if np.any(np.isfinite(y)) else 0.0
            y = np.nan_to_num(y, nan=fill)
        freqs, amp, references, lines = analyze_signal(
            col, x, y, fs_domain, fundamental, args.pole_pairs, args.method,
            args.detrend, args.tol, max_bin, args.top, unit)
        for ln in lines:
            print(ln)
        print("-" * 78)
        results.append((col, freqs, amp, references))

    dom = "阶次" if args.domain == "order" else "频率"
    title = f"servo_stats 幅度谱 ({dom}) - {label} ~{f_mech:.2f}rev/s"
    return dict(results=results, xlabel=xlabel, title=title, f_mech=f_mech)


# ── 交互式框选诊断 ──────────────────────────────────────────────────────────
def interactive_select(df, t, rev, fft_cols, args, fs):
    """在时间序列图上鼠标拖拽框选区间, 对选区做 FFT 诊断并弹出频谱。"""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import SpanSelector
    _setup_cjk_font()

    show_cols = (["velocity"] if "velocity" in df.columns else [])
    for c in fft_cols:
        if c in df.columns and c not in show_cols:
            show_cols.append(c)
    if not show_cols:
        print("[错误] 没有可显示的列。")
        return

    fig, axes = plt.subplots(len(show_cols), 1, sharex=True,
                             figsize=(11, 1.9 * len(show_cols)), squeeze=False)
    axes = axes[:, 0]
    for ax, c in zip(axes, show_cols):
        ax.plot(t, df[c].to_numpy(dtype=float), lw=0.8)
        ax.set_ylabel(c)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("时间 t (s)  —  在图上拖拽选择要分析的区间")
    fig.suptitle("交互框选: 拖拽选一段(尽量恒速)区间做FFT诊断  "
                 "(可重复选择; 关闭窗口结束)")
    fig.tight_layout()

    state = {"specfig": None, "k": 0}

    def on_select(xmin, xmax):
        i0 = int(np.searchsorted(t, xmin, "left"))
        i1 = int(np.searchsorted(t, xmax, "right"))
        i0, i1 = max(0, i0), min(len(t), i1)
        if i1 - i0 < 16:
            print(f"[选择过短] 仅 {i1 - i0} 点, 请拖长一些。")
            return
        print("=" * 78)
        out = diagnose_window(df, t, rev, i0, i1, fft_cols, args, fs, None,
                              "手动选区")
        if out is None:
            return
        sfig = _render_spectra(out["results"], out["xlabel"], out["title"],
                               state["specfig"])
        state["specfig"] = sfig
        if args.save:
            path = f"{args.save}_sel{state['k']}.png"
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            sfig.savefig(path, dpi=130)
            print(f"[已保存] {path}")
            state["k"] += 1
        sfig.canvas.draw_idle()
        try:
            sfig.show()  # 非交互后端(如 Agg)会告警, 忽略之
        except Exception:
            pass

    span = SpanSelector(
        axes[0], on_select, "horizontal", useblit=True,
        props=dict(alpha=0.2, facecolor="tab:orange"),
        interactive=True, drag_from_anywhere=True)
    fig._span_selector = span  # 保持引用, 防止被 GC
    print("[交互] 已打开选择窗口: 在最上方(或任意)子图拖拽框选区间即可诊断。")
    plt.show()


# ── 主流程 ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="moteus servo_stats 绘图 + FFT 电机异常诊断",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("csv", nargs="?",
                    default="logs/log_5_100hz_1_servo_stats.csv",
                    help="servo_stats CSV 路径")
    ap.add_argument("--list", action="store_true", help="列出所有列并退出")
    ap.add_argument("--plot", default=None,
                    help="要画时间序列的列, 逗号分隔, 如 d_A,q_A,velocity")
    ap.add_argument("--fft", default=None,
                    help=f"要做频谱诊断的列, 逗号分隔 (默认: "
                         f"{','.join(DEFAULT_FFT_COLS)})")
    ap.add_argument("--pole-pairs", type=int, default=7,
                    help="电机极对数 pp (电频率 = pp × rev/s), 默认 7")
    ap.add_argument("--rev-per-s", type=float, default=None,
                    help="机械频率 rev/s; 默认取 |velocity| 均值")
    ap.add_argument("--method", choices=["fft", "ls"], default="fft",
                    help="谱估计: fft=均匀重采样FFT; ls=Lomb-Scargle(可超奈奎斯特)")
    ap.add_argument("--domain", choices=["time", "order"], default="time",
                    help="time=时间域(Hz); order=阶次域(按角度重采样, "
                         "天然处理变速), 默认 time")
    ap.add_argument("--segment", choices=["none", "auto"], default="auto",
                    help="auto=按转速自动分段, 逐段诊断(处理一张表多转速); "
                         "none=整段, 默认 auto")
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="交互模式: 弹出时间序列, 鼠标拖拽框选区间做FFT诊断")
    ap.add_argument("--min-revs", type=float, default=8.0,
                    help="自动分段时每段至少的机械转数, 默认 8")
    ap.add_argument("--seg-tol", type=float, default=0.08,
                    help="分段判定的转速相对容差, 默认 0.08")
    ap.add_argument("--fs", type=float, default=None,
                    help="时间域 FFT 重采样率 Hz; 默认 1/中位采样间隔")
    ap.add_argument("--detrend", choices=["none", "constant", "linear"],
                    default="linear", help="去趋势方式, 默认 linear")
    ap.add_argument("--tol", type=float, default=0.06,
                    help="谱峰匹配相对容差 (默认 0.06 = 6%%)")
    ap.add_argument("--max-freq", type=float, default=None,
                    help="ls 方法的最高探测频率 Hz; 默认 3×(2×elec)")
    ap.add_argument("--top", type=int, default=6, help="列出前 N 个谱峰")
    ap.add_argument("--save", default=None, help="图片保存前缀, 如 out/run1")
    ap.add_argument("--no-show", action="store_true", help="不弹出窗口")
    args = ap.parse_args()

    try:  # 让中文报告在 Windows 终端/重定向时也能正常输出
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

    df, t = load_log(args.csv)
    fs_eff, dt_med, dt_min, dt_max = sampling_info(t)

    if args.list:
        print(f"文件: {args.csv}")
        print(f"样本数: {len(df)}  时长: {t[-1]:.2f}s  "
              f"有效采样率≈{fs_eff:.2f}Hz (中位Δt={dt_med * 1e3:.2f}ms)")
        print("可用列:")
        for c in df.columns:
            print(f"  {c}")
        return

    fs = args.fs if args.fs else fs_eff

    # 机械转数(角度), 供分段与阶次域使用
    if "position" in df.columns or "velocity" in df.columns:
        rev = build_angle(df, t)
    else:
        rev = None

    # 时间序列绘图
    show = not args.no_show
    if args.plot:
        plot_timeseries(df, t, [c.strip() for c in args.plot.split(",")],
                        args.save, show)

    # 频谱诊断
    do_fft = (args.fft is not None) or (args.plot is None)
    if not do_fft:
        return

    fft_cols = ([c.strip() for c in args.fft.split(",")]
                if args.fft else DEFAULT_FFT_COLS)
    fft_cols = [c for c in fft_cols if c in df.columns]

    # 交互框选模式: 不做自动分段, 由用户拖拽选区
    if args.interactive:
        print("=" * 78)
        print("moteus servo_stats 交互框选诊断")
        print("=" * 78)
        print(f"文件: {args.csv}   {len(df)}点/{t[-1]:.2f}s   "
              f"采样率≈{fs_eff:.2f}Hz   极对数pp={args.pole_pairs}   "
              f"分析域={'order' if args.domain == 'order' else 'time'}")
        print("归因规则: 1×mech->编码器偏心 | 1×elec->电流偏置/编码器一次谐波 | "
              "2×elec->相增益失配")
        interactive_select(df, t, rev, fft_cols, args, fs)
        return

    # 确定分段
    if "velocity" in df.columns:
        v_all = df["velocity"].to_numpy(dtype=float)
    else:
        v_all = None
    dropped = []
    if args.segment == "auto" and v_all is not None and rev is not None \
            and args.rev_per_s is None:
        segments, dropped = detect_segments(
            t, v_all, rev, args.min_revs, args.seg_tol)
    else:
        segments = [(0, len(df), None)]

    # 报告头
    print("=" * 78)
    print("moteus servo_stats 电机异常 FFT 诊断报告")
    print("=" * 78)
    print(f"文件         : {args.csv}")
    print(f"样本数/时长  : {len(df)} 点 / {t[-1]:.2f} s")
    print(f"有效采样率   : {fs_eff:.2f} Hz  (中位Δt={dt_med * 1e3:.2f}ms, "
          f"min={dt_min * 1e3:.2f}ms, max={dt_max * 1e3:.2f}ms)")
    print(f"极对数 pp    : {args.pole_pairs}")
    print(f"分析域       : {'阶次域(order)' if args.domain == 'order' else '时间域(Hz)'}"
          f"   谱估计: {args.method}   分段: {args.segment}")
    print(f"检出转速段   : {len(segments)} 段")
    if dropped:
        print(f"已丢弃候选段 : {len(dropped)} 段 (转数不足 --min-revs="
              f"{args.min_revs:g}); 若需分析请下调 --min-revs:")
        for i0, i1, mean, revs in dropped:
            print(f"               t=[{t[i0]:.2f},{t[i1 - 1]:.2f}]s  "
                  f"转速≈{mean:.3f} rev/s  仅~{revs:.1f}转")
    print("-" * 78)
    print("归因规则: 1×mech->编码器偏心 | 1×elec->电流偏置/编码器一次谐波 | "
          "2×elec->相增益失配")
    print("-" * 78)

    multi = len(segments) > 1
    for s_idx, (i0, i1, mean_v) in enumerate(segments):
        label = f"转速段 {s_idx}:" if multi else "整段:"
        out = diagnose_window(df, t, rev, i0, i1, fft_cols, args, fs,
                              mean_v, label)
        if out is None:
            continue
        tag = f"spectrum_seg{s_idx}" if multi else "spectrum"
        plot_spectra(out["results"], out["xlabel"], out["title"],
                     args.save, tag, show)


if __name__ == "__main__":
    main()
