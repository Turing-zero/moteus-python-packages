# scripts —— servo_stats 日志分析工具

针对 moteus `servo_stats` CSV 日志的两个电机异常分析脚本。两者互补：

| 脚本 | 分析域 | 核心用途 |
| --- | --- | --- |
| `plot_servo_stats.py` | 频率域 (Hz) / 阶次域 | 通用绘图 + FFT/Lomb-Scargle 频谱诊断，把谱峰对齐到 `1×mech / 1×elec / 2×elec` 参考族 |
| `order_analysis.py` | 阶次域 (order, 周/转) | 按转子角度重采样做 FFT，跨转速对比同一阶次分量，转速波动/时间戳误差不糊峰 |

依赖：`numpy`、`pandas`、`scipy`、`matplotlib`（绘图/交互时）。

CSV 需含 `timestamp` 列；转速分段与阶次分析需 `velocity` / `position` 列。

---

## 归因速查

两脚本共享同一套物理归因（以极对数 `pp`、定子槽数 `Ns` 为参数）：

| 特征 | 阶次 | 归因 |
| --- | --- | --- |
| `1×mech` | 1 阶 | 编码器/联轴/负载偏心（随速增长，高速主导）|
| 2 阶 | 2 阶 | 码盘倾斜 / 联轴不对中 |
| `1×elec` | `pp` 阶 | 电流采样偏置 或 编码器电周期一次谐波 |
| `2×elec` | `2·pp` 阶 | 相增益失配（+ 磁极侧齿槽）|
| 槽数 | `Ns` 阶 | 定子公差齿槽、定子侧不对称 |

> 采样混叠：`servo_stats` 是软件轮询，采样率常只有几十 Hz。电频率 = `pp × rev/s`，很容易超过奈奎斯特而混叠到低频。`plot_servo_stats.py` 会给出真实/折叠后频率并在混叠频率处检索；若要彻底避开，用高频 register poll 记录，或改用阶次域 (`order_analysis.py` / `--domain order`)。

---

## `plot_servo_stats.py`

从 CSV 任选列绘制时间序列，并对信号做频谱诊断。

### 常用示例

```bash
# 列出所有可用列
python scripts/plot_servo_stats.py logs/xxx.csv --list

# 画若干列的时间序列
python scripts/plot_servo_stats.py logs/xxx.csv --plot d_A,q_A,velocity

# 交互框选: 弹出时间序列, 鼠标拖拽选区即对该区间做 FFT 诊断 (可重复)
python scripts/plot_servo_stats.py logs/xxx.csv --interactive --pole-pairs 7

# 默认信号 (d_A,q_A,velocity,position) 做异常诊断 FFT
python scripts/plot_servo_stats.py logs/xxx.csv --pole-pairs 7

# 指定分析列 + Lomb-Scargle (可探测超奈奎斯特频率, 缓解混叠)
python scripts/plot_servo_stats.py logs/xxx.csv --fft q_A,d_A --pole-pairs 7 --method ls

# 阶次域诊断 (按角度重采样, 天然处理变速)
python scripts/plot_servo_stats.py logs/xxx.csv --pole-pairs 7 --domain order

# 保存图片而不弹窗
python scripts/plot_servo_stats.py logs/xxx.csv --pole-pairs 7 --save out/analysis --no-show
```

### 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `csv` | `logs/log_5_100hz_1_servo_stats.csv` | servo_stats CSV 路径 |
| `--list` | — | 列出所有列并退出 |
| `--plot` | — | 要画时间序列的列，逗号分隔 |
| `--fft` | `d_A,q_A,velocity,position` | 要做频谱诊断的列，逗号分隔 |
| `--pole-pairs` | `7` | 电机极对数 pp（电频率 = pp × rev/s）|
| `--rev-per-s` | 取 `\|velocity\|` 均值 | 机械频率 rev/s |
| `--method` | `fft` | `fft`=均匀重采样 FFT；`ls`=Lomb-Scargle（可超奈奎斯特）|
| `--domain` | `time` | `time`=频率域(Hz)；`order`=阶次域（按角度重采样）|
| `--segment` | `auto` | `auto`=按转速自动分段逐段诊断；`none`=整段 |
| `--interactive` / `-i` | — | 交互模式：拖拽框选区间做 FFT 诊断 |
| `--min-revs` | `8` | 自动分段时每段至少的机械转数 |
| `--seg-tol` | `0.08` | 分段判定的转速相对容差 |
| `--fs` | 1/中位采样间隔 | 时间域 FFT 重采样率 Hz |
| `--detrend` | `linear` | 去趋势方式：`none`/`constant`/`linear` |
| `--tol` | `0.06` | 谱峰匹配相对容差 |
| `--max-freq` | 3×(2×elec) | `ls` 方法最高探测频率 Hz |
| `--top` | `6` | 列出前 N 个谱峰 |
| `--save` | — | 图片保存前缀，如 `out/run1` |
| `--no-show` | — | 不弹出窗口 |

---

## `order_analysis.py`

用 `position`（编码器积分转数）把 `torque` 从"时间均匀采样"重采样为"转角均匀采样"再做 FFT。横轴为机械阶次（order，周/转），与时间戳无关，因此转速微小波动不糊峰、时间戳标定误差在阶次域不存在。

### 常用示例

```bash
# 默认: 自动分段, 分析 torque, 出两张图 (阶次谱叠加 + 各阶次幅值 vs 转速)
python scripts/order_analysis.py logs/lite_1.csv --pole-pairs 13

# 交互框选: 反复在时间序列上拖拽选取不同转速段, 自动叠加对比其阶次谱
python scripts/order_analysis.py logs/lite_1.csv --pole-pairs 13 --interactive

# 自定义要标注的阶次族 + 分析列
python scripts/order_analysis.py logs/lite_1.csv --pole-pairs 13 --orders 1,2,13,24,26 --col torque --save out/run1

# 不弹窗只存图
python scripts/order_analysis.py logs/lite_1.csv --pole-pairs 13 --save out/run1 --no-show
```

### 交互模式（核心用途）

阶次分析的价值在于**跨转速对比同一阶次分量**。交互模式下：

- 在最上方子图反复拖拽框选不同转速的恒速段；
- 每选一段就把该段阶次谱**叠加**进对比窗口（不同颜色区分转速），并更新"各阶次幅值 vs 转速"曲线；
- 时间序列图上用橙色阴影 + `#编号/转速` 标出已选区间；
- 终端逐段打印各追踪阶次的幅值；
- 配合 `--save` 时保存为单张随选区累积更新的 `*_order_interactive.png`。

### 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `csv` | 必填 | servo_stats CSV 路径 |
| `--pole-pairs` | `13` | 极对数 |
| `--slots` | `24` | 定子槽数（用于阶次标注）；`0`=不标 |
| `--col` | `torque` | 分析列 |
| `--orders` | `1,2,pp,2*pp,slots` | 要追踪/标注的阶次，逗号分隔 |
| `--order-max` | `32` | 阶次谱横轴上限 |
| `--min-revs` | `5` | 平台最少转数 |
| `--seg-tol` | `0.08` | 平台转速相对容差 |
| `--interactive` / `-i` | — | 交互模式：拖拽框选多段做阶次对比（不能与 `--no-show` 同用）|
| `--save` | — | 图片保存前缀，如 `out/run1` |
| `--no-show` | — | 不弹窗 |

> 阶次奈奎斯特：超过每转采样点数 / 2 的阶次分量仍会混叠，1~26 阶通常安全。理想齿槽在 `LCM(Ns, 2·pp)` 阶（如 24S-26P → 312 阶），一般远超奈奎斯特，看不到。
