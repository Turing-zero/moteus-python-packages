# moteus 通信协议文档(基于本仓库实现)

> 本文档由 `moteus/moteus/protocol.py`、`multiplex.py`、`moteus.py`、`transport.py`、`control_manager.py` 的源码整理而成,字节示例均由本仓库代码实际生成。
> 寄存器的完整官方定义见上游 <https://github.com/mjbots/moteus/blob/main/docs/reference.md>。本文只覆盖本仓库实现了的部分;固件侧的语义(如各 fault 码含义、`ENCODER_VALIDITY` 的位定义)本仓库代码中没有,文中已标注"未在仓库中定义"。

## 目录

1. 分层概览
2. CAN-FD 帧与仲裁 ID
3. Multiplex 子帧格式
4. 数据类型、NaN 与缩放
5. 寄存器表
6. 控制模式 `Mode`
7. 查询(Query)的编码
8. 指令(Command)的编码与字节示例
9. Diagnostic 流通道
10. UUID 寻址
11. 高频控制接口 `ControlManager` 与 enc2 读取
12. 源码索引

---

## 1. 分层概览

```
Controller   (moteus.py)          make_*() 生成 Command(data + 仲裁字段)
Transport    (transport.py)       Command → CAN-FD Frame,多设备路由
TransportDevice                   fdcanusb / python-can / candle
```

所有应用层数据都放在 CAN-FD 帧的 data 段里,使用 mjlib **multiplex** 协议编码(第 3 节)。

## 2. CAN-FD 帧与仲裁 ID

帧属性:`is_fd=True`,`bitrate_switch=True`(BRS);仲裁 ID 大于 `0x7ff` 时使用扩展 ID(`transport.py:_command_to_frame`)。

**发送(主机 → 控制器)**

```
arbitration_id = dest_id
               | (0x8000 if reply_required else 0)
               | (source_id << 8)
               | (can_prefix << 16)
```

| 字段 | 位 | 说明 |
|---|---|---|
| `dest_id` | 6:0 | 目标控制器 CAN ID,`0x7f` 为广播 |
| `source_id` | 14:8 | 源 ID,`Controller(source_can_id=0)` 默认 0 |
| `reply_required` | 15 | 为 1 时控制器必须回复(即带 query) |
| `can_prefix` | 28:16 | `Controller(can_prefix=0x0000)` 默认 0 |

**接收(控制器 → 主机)**

- 回复帧中源/目的互换:`(arb >> 8) & 0x7f` 为回复方的控制器 ID(`parse_message` 里 `result.id`),`arb & 0x7f` 为原请求的 `source_id`。
- `Transport` 按 `prefix = (arb >> 16) & 0x1fff`、目的 ID、源 ID 过滤回复(`transport.py` 约 343-350 行)。

**帧长限制**:CAN-FD 数据段最大 64 字节;diagnostic 写入限 61 字节(`make_diagnostic_write` 的 assert)。

## 3. Multiplex 子帧格式

一个 CAN 帧的 data 由若干**子帧**顺序拼接。每个子帧以 1 字节命令码开头(`multiplex.py:parse_frame`)。

### 3.1 寄存器子帧(写 / 读 / 回复)

命令字节位布局:

```
bit 7..4   类型:  0x0 = WRITE(写)   0x1 = READ(读请求)   0x2 = REPLY(回复)
bit 3..2   数据类型: 0=int8  1=int16  2=int32  3=f32
bit 1..0   连续寄存器个数 N:1..3 时直接内嵌;为 0 时后面跟一个 varuint 表示个数
```

随后是 **起始寄存器号**(varuint),之后:

- WRITE / REPLY:紧跟 N 个同类型的值(小端),对应寄存器 `start .. start+N-1`。
- READ:没有值,只表示"请求这 N 个寄存器,以该类型回复"。

所以:

| 写 | 读 | 回复 | 类型 |
|---|---|---|---|
| `0x00` | `0x10` | `0x20` | int8 |
| `0x04` | `0x14` | `0x24` | int16 |
| `0x08` | `0x18` | `0x28` | int32 |
| `0x0c` | `0x1c` | `0x2c` | f32 |

加上个数 1..3 即得到短格式,例如 `0x1f` = 读 f32 × 3,`0x0d` = 写 f32 × 1。个数 ≥ 4 用长格式:`<cmd> <count> <start_reg>`,如 `1c 06 50` = 读 f32 × 6,从 0x50 开始。

### 3.2 其它子帧

| 命令字节 | 含义 | 后续内容 |
|---|---|---|
| `0x30` | WRITE_ERROR | `register`(varuint)、`error_code`(varuint) |
| `0x31` | READ_ERROR | 同上 |
| `0x40` | STREAM_CLIENT_DATA(主机→控制器) | `channel`、`size`、`size` 字节数据 |
| `0x41` | STREAM_SERVER_DATA(控制器→主机) | `channel`、`size`(varuint)、数据 |
| `0x42` | STREAM_CLIENT_POLL | `channel`、`max_length` |
| `0x50` | NOP | 无 |

遇到未知命令码,解析器直接停止解析该帧(避免错位)。`error_code` 的具体含义未在本仓库中定义。

### 3.3 varuint

每字节低 7 位为数据,最高位为 1 表示后面还有字节;小端,最多 5 字节。寄存器号 ≥ 128(如 `0x100`)需要 2 字节。

## 4. 数据类型、NaN 与缩放

### 4.1 Resolution 常量

| 常量 | 值 | 线上大小 | 备注 |
|---|---|---|---|
| `mp.INT8` | 0 | 1 B | |
| `mp.INT16` | 1 | 2 B | |
| `mp.INT32` | 2 | 4 B | |
| `mp.F32` | 3 | 4 B | 浮点,直接是物理单位 |
| `mp.IGNORE` | 4 | — | 仅本地使用,表示"不写/不查",不上线 |

### 4.2 NaN 表示

整数类型用**最小负值**表示 NaN:int8 `-128`、int16 `-32768`、int32 `-2147483648`;f32 直接用 IEEE NaN。解码时转为 `math.nan`,编码时非有限值写成这些值。

### 4.3 缩放

整数类型的物理值 = 原始整数 × scale,f32 的 scale 恒为 1。编码时按 scale 反算并**饱和**到该类型的最大值(`saturate`)。

| 物理量 | int8 | int16 | int32 | 单位 |
|---|---|---|---|---|
| position | 0.01 | 0.0001 | 0.00001 | rev |
| velocity | 0.1 | 0.00025 | 0.00001 | rev/s |
| accel | 0.05 | 0.001 | 0.00001 | rev/s² |
| torque | 0.5 | 0.01 | 0.001 | N·m |
| voltage | 0.5 | 0.1 | 0.001 | V |
| temperature | 1.0 | 0.1 | 0.001 | °C |
| current | 1.0 | 0.1 | 0.001 | A |
| power | 10.0 | 0.05 | 0.0001 | W |
| time | 0.01 | 0.001 | 0.000001 | s |
| pwm / scale / 比例(0~1) | 1/127 | 1/32767 | 1/2147483647 | 无量纲 |

整数类寄存器(`MODE`、`FAULT`、`TRAJECTORY_COMPLETE`、`ENCODER_VALIDITY`、GPIO 位域、`MILLISECOND_COUNTER` 等)直接取 `int(value)`,不缩放。

## 5. 寄存器表

来源:`protocol.py:Register`。"类型"列的缩放类别见 4.3;"—"表示取整数、不缩放。

### 5.1 状态(只读)

| 寄存器 | 地址 | 缩放类别 |
|---|---|---|
| `MODE` | 0x000 | — |
| `POSITION` | 0x001 | position |
| `VELOCITY` | 0x002 | velocity |
| `TORQUE` | 0x003 | torque |
| `Q_CURRENT` | 0x004 | current |
| `D_CURRENT` | 0x005 | current |
| `ABS_POSITION` | 0x006 | position |
| `POWER` | 0x007 | power |
| `MOTOR_TEMPERATURE` | 0x00a | temperature |
| `TRAJECTORY_COMPLETE` | 0x00b | — |
| `REZERO_STATE` / `HOME_STATE` | 0x00c | —(同一地址的两个别名) |
| `VOLTAGE` | 0x00d | voltage |
| `TEMPERATURE` | 0x00e | temperature |
| `FAULT` | 0x00f | — |

### 5.2 底层输出 / 调试

| 寄存器 | 地址 |
|---|---|
| `PWM_PHASE_A/B/C` | 0x010-0x012 |
| `VOLTAGE_PHASE_A/B/C` | 0x014-0x016 |
| `VFOC_THETA` / `VFOC_VOLTAGE` | 0x018 / 0x019 |
| `VOLTAGEDQ_D` / `VOLTAGEDQ_Q` | 0x01a / 0x01b |
| `COMMAND_Q_CURRENT` / `COMMAND_D_CURRENT` | 0x01c / 0x01d |
| `VFOC_THETA_RATE` | 0x01e |

(这一段在 `scale_register` 中没有专门缩放分支,未知寄存器按"仅 NaN 处理、不缩放"返回。)

### 5.3 位置模式指令寄存器 0x020-0x02e

| 寄存器 | 地址 | 缩放类别 |
|---|---|---|
| `COMMAND_POSITION` | 0x020 | position |
| `COMMAND_VELOCITY` | 0x021 | velocity |
| `COMMAND_FEEDFORWARD_TORQUE` | 0x022 | torque |
| `COMMAND_KP_SCALE` | 0x023 | pwm |
| `COMMAND_KD_SCALE` | 0x024 | pwm |
| `COMMAND_POSITION_MAX_TORQUE` | 0x025 | torque |
| `COMMAND_STOP_POSITION` | 0x026 | position |
| `COMMAND_TIMEOUT` | 0x027 | time |
| `COMMAND_VELOCITY_LIMIT` | 0x028 | velocity |
| `COMMAND_ACCEL_LIMIT` | 0x029 | accel |
| `COMMAND_FIXED_VOLTAGE_OVERRIDE` | 0x02a | voltage |
| `COMMAND_ILIMIT_SCALE` | 0x02b | pwm |
| `COMMAND_FIXED_CURRENT_OVERRIDE` | 0x02c | current(写入时;解码分支未单独处理) |
| `COMMAND_IGNORE_POSITION_BOUNDS` | 0x02d | 整数 |
| `COMMAND_KIV_SCALE` | 0x02e | pwm |

### 5.4 控制器内部量 0x030-0x03d

`POSITION_KP/KI/KD/FEEDFORWARD/COMMAND`(0x030-0x034,torque 类缩放);`CONTROL_POSITION`/`CONTROL_VELOCITY`/`CONTROL_TORQUE`(0x038-0x03a);`POSITION_ERROR`/`VELOCITY_ERROR`/`TORQUE_ERROR`(0x03b-0x03d)。

### 5.5 Stay-within 指令寄存器 0x040-0x049

`COMMAND_WITHIN_LOWER_BOUND`(0x040,position)、`UPPER_BOUND`(0x041,position)、`FEEDFORWARD_TORQUE`(0x042)、`KP_SCALE`(0x043)、`KD_SCALE`(0x044)、`MAX_TORQUE`(0x045)、`TIMEOUT`(0x046,time)、`ILIMIT_SCALE`(0x047)、`IGNORE_POSITION_BOUNDS`(0x048)、`KIV_SCALE`(0x049)。

### 5.6 编码器 0x050-0x058

| 寄存器 | 地址 | 缩放类别 |
|---|---|---|
| `ENCODER_0_POSITION` | 0x050 | position |
| `ENCODER_0_VELOCITY` | 0x051 | velocity |
| `ENCODER_1_POSITION` | 0x052 | position |
| `ENCODER_1_VELOCITY` | 0x053 | velocity |
| **`ENCODER_2_POSITION`** | **0x054** | position |
| **`ENCODER_2_VELOCITY`** | **0x055** | velocity |
| `ENCODER_VALIDITY` | 0x058 | —(位定义未在仓库中定义) |

### 5.7 AUX 外设 0x05c-0x07f

- GPIO:`AUX1_GPIO_COMMAND` 0x05c、`AUX2_GPIO_COMMAND` 0x05d、`AUX1_GPIO_STATUS` 0x05e、`AUX2_GPIO_STATUS` 0x05f(整数位域)。
- 模拟输入(0~1,pwm 类缩放):`AUX1_ANALOG_IN1..5` 0x060-0x064,`AUX2_ANALOG_IN1..5` 0x068-0x06c。
- `MILLISECOND_COUNTER` 0x070、`CLOCK_TRIM` 0x071(整数)。
- PWM 输入:`AUX1_PWM_INPUT_PERIOD` 0x072(整数 µs)、`AUX1_PWM_INPUT_DUTY_CYCLE` 0x073、`AUX2_…PERIOD` 0x074、`AUX2_…DUTY_CYCLE` 0x075。
- PWM 输出(写,pwm 类缩放,默认 int16):`AUX1_PWM1..5` 0x076-0x07a,`AUX2_PWM1..5` 0x07b-0x07f。

> 备注:`protocol.py:139-148` 中 `AUX1_PWM1 = 0x076,` 等行末尾带多余的逗号。因为是 `IntEnum`,元组会被解包给 `int()`,实测 `Register.AUX1_PWM1 == 118`,功能正常,只是写法上不整洁。

### 5.8 设备信息与操作

| 寄存器 | 地址 | 说明 |
|---|---|---|
| `MODEL_NUMBER` | 0x100 | |
| `FIRMWARE_VERSION` | 0x101 | |
| `REGISTER_MAP_VERSION` | 0x102 | |
| `MULTIPLEX_ID` | 0x110 | |
| `SERIAL_NUMBER1/2/3` | 0x120-0x122 | |
| `REZERO` / `SET_OUTPUT_NEAREST` | 0x130 | f32 写入,同一地址的别名 |
| `SET_OUTPUT_EXACT` | 0x131 | |
| `REQUIRE_REINDEX` | 0x132 | |
| `RECAPTURE_POSITION_VELOCITY` | 0x133 | |
| `DRIVER_FAULT1/2` | 0x140 / 0x141 | |
| `UUID1..4` | 0x150-0x153 | |
| `UUID_MASK1..4` | 0x154-0x157 | |
| `UUID_MASK_CAPABLE` | 0x158 | |

## 6. 控制模式 `Mode`(写 `MODE` 寄存器 0x000,int8)

| 值 | 名称 | 值 | 名称 |
|---|---|---|---|
| 0 | STOPPED | 10 | POSITION |
| 1 | FAULT | 11 | TIMEOUT |
| 5 | PWM | 12 | ZERO_VELOCITY |
| 6 | VOLTAGE | 13 | STAY_WITHIN |
| 7 | VOLTAGE_FOC | 14 | MEASURE_IND |
| 8 | VOLTAGE_DQ | 15 | BRAKE |
| 9 | CURRENT | | |

## 7. 查询(Query)的编码

`Controller._make_query_data` 把 `QueryResolution` 中每个字段的 resolution 排成若干**寄存器块**,由 `WriteCombiner`(基础命令 `0x10` 即读)把相邻且 resolution 相同的字段合并为一个 READ 子帧,`IGNORE` 的字段跳过。

分块如下(每块内部只合并块内相邻字段):

| 块 | 起始寄存器 | 包含字段(按顺序) |
|---|---|---|
| c1 | 0x000 | mode, position, velocity, torque, q_current, d_current, abs_position, power |
| c2 | 0x00a | motor_temperature, trajectory_complete, rezero/home_state, voltage, temperature, fault |
| c3 | 0x05e | aux1_gpio, aux2_gpio |
| c4 | 0x072 | aux1/2 PWM 输入周期与占空比 |
| c5 | `_extra` 的最小键 | `QueryResolution._extra` 中指定的任意寄存器(其间空缺用 IGNORE 填充) |

`QueryResolution` 默认值:mode=INT8,position/velocity/torque=F32,voltage/temperature/fault=INT8,其余 IGNORE。

**合并规则**:resolution 相同的连续字段合并为一个子帧;个数 ≤ 3 用短格式(1 字节命令 + varuint 起始寄存器),否则用长格式(命令 + 个数 + 起始寄存器)。

### 实测字节

| 查询 | data (hex) | 预期回复字节数 |
|---|---|---|
| 默认 | `11 00 1f 01 13 0d` | 22 |
| `ControlManager` 的 `full` 档 | `11 00 1f 01 1d 06 11 0b 13 0d 1c 06 50 11 58` | 61 |

逐段解读 `full`:

```
11 00        读 int8×1 @0x000            mode
1f 01        读 f32×3  @0x001            position, velocity, torque
1d 06        读 f32×1  @0x006            abs_position
11 0b        读 int8×1 @0x00b            trajectory_complete
13 0d        读 int8×3 @0x00d            voltage, temperature, fault
1c 06 50     读 f32×6  @0x050 (长格式)   enc0 pos/vel, enc1 pos/vel, enc2 pos/vel
11 58        读 int8×1 @0x058            encoder_validity
```

### 回复帧

回复使用相同结构,命令字节换成 `0x2x`,后面带值。以默认查询为例,回复形如(由解析器格式推得,不是抓包):

```
21 00 <mode:i8>
2f 01 <position:f32> <velocity:f32> <torque:f32>
23 0d <voltage:i8> <temperature:i8> <fault:i8>
```

主机端收到后通过 `parse_registers()` 得到 `{寄存器: 缩放后的值}`。回复过滤函数 `expect_reply` 认为 data 首字节高 4 位为 `0x2` 或等于 `0x31` 的帧是有效回复。

## 8. 指令(Command)的编码与字节示例

所有 `make_*` 返回的 `Command` 包含:`destination`、`source`、`reply_required`(= 是否带 query)、`can_prefix`、`data`、`expected_reply_size`。

**通用结构**:`[UUID 前缀(可选)] + [指令子帧...] + [query 子帧(可选)]`。

| 方法 | 写入内容 |
|---|---|
| `make_stop` | `MODE ← 0` |
| `make_brake` | `MODE ← 15` |
| `make_zero_velocity` | `MODE ← 12`,可选 `COMMAND_KD_SCALE` |
| `make_position` | `MODE ← 10`,再从 0x020 起写 15 个可选字段(见 5.3) |
| `make_stay_within` | `MODE ← 13`,从 0x040 起写 |
| `make_vfoc` | `MODE ← 7`,写 theta / voltage / theta_rate |
| `make_current` | `MODE ← 9`,写 d / q 电流 |
| `make_rezero` / `make_set_output*` | 向 0x130 / 0x131 写 f32 |
| `make_write_gpio` | 写 `AUX1/AUX2_GPIO_COMMAND`(int8) |
| `make_set_trim` | 向 `CLOCK_TRIM` 写 int32 |
| `make_diagnostic_write/read` | stream 子帧(第 9 节) |

`make_position` 的各字段 `None` 表示不写(IGNORE);非 `None` 的字段按 `PositionResolution` 的类型编码(默认全 F32),并由 `WriteCombiner` 合并相邻字段。

### 实测示例(controller id=1)

| 调用 | data (hex) | 解读 |
|---|---|---|
| `make_stop()` | `01 00 00` | 写 int8×1 @0x000 = 0(STOPPED) |
| `make_stop(query=True)` | `01 00 00 11 00 1f 01 13 0d` | 停止 + 默认查询;仲裁 ID 带 `0x8000` |
| `make_query()` | `11 00 1f 01 13 0d` | 只查询 |
| `make_position(position=0.5, velocity=0.0, query=True)` | `01 00 0a  0e 20 0000003f 00000000  11 00 1f 01 13 0d` | MODE=10;写 f32×2 @0x020 = (0.5, 0.0);再附默认查询 |
| `make_position(position=nan, velocity=1.0, maximum_torque=0.3, kp_scale=0.5)` | `01 00 0a  0e 20 0000c07f 0000803f  0d 23 0000003f  0d 25 9a99993e` | position 为 NaN(f32 NaN);kd_scale 未指定,所以 0x023 与 0x025 各自单独成一个子帧(中间的 0x024 被跳过) |

## 9. Diagnostic 流通道

用于通过 CAN 帧隧道传输控制器的文本诊断口(与 `tview`/`moteus_tool` 的 `diagnostic` 命令同一机制)。

- 写:`40 <channel> <len> <data...>`,data ≤ 61 字节(`make_diagnostic_write`,默认 channel=1)。
- 轮询:`42 <channel> <max_length>`(`make_diagnostic_read`,默认 max_length=48),要求回复。
- 回复:`41 <channel> <len:varuint> <data...>`;`parse_diagnostic_message` 校验首字节为 `0x41`、channel 匹配、长度不越界,否则返回 `None`。

## 10. UUID 寻址

用 `DeviceAddress` 寻址时(CAN ID 为空),`make_uuid_prefix` 会在 data 最前面加一段写 `UUID_MASK1..` 的 int32 子帧(UUID 前缀按小端 4 字节一组,长度必须是 4 的倍数且 ≤ 16 字节),使得只有 UUID 匹配的控制器响应。

广播 ID 为 `0x7f`;`Transport.discover()` 向 `0x7f` 广播 UUID 查询并建立路由表。

## 11. 高频控制接口 `ControlManager` 与 enc2 读取

### 11.1 工作方式

- 后台线程运行 asyncio 事件循环,默认 `cycle_hz=200`,`query_profile='full'`。
- 每周期:取出命令队列 → 为每个控制器选一个命令(一次性命令 > 持续命令 > 纯 query)→ 用 `transport.cycle()` 一次发出 → 解析回复 → 更新状态并通知 listener。
- 所有 `command_*()` 都附带 `query=True`,所以每个控制周期都会收到回复,状态随之更新。

### 11.2 查询档位

| 档位 | 读取内容 | 说明 |
|---|---|---|
| `lite` | 默认 `QueryResolution`(mode/position/velocity/torque/voltage/temperature/fault) | 回复小,USB 适配器上通常更快 |
| `full`(默认) | lite + `trajectory_complete` + `abs_position` + encoder 0/1/2 的位置、速度 + `encoder_validity` | 线上字节见第 7 节 |

可用 `ControlManager(query_profile=...)` 或 `set_query_profile()` 切换。

### 11.3 enc2 的读取接口

- 线上寄存器:`ENCODER_2_POSITION`(0x054,rev)、`ENCODER_2_VELOCITY`(0x055,rev/s),均以 F32 查询。
- Python 接口:`ControlManager.get_status()` / `get_status_one(id)` 返回 `ControllerStatus`,其中:
  - `encoder_2_position: float`
  - `encoder_2_velocity: float`
  - 未收到该寄存器时为 `nan`(例如 `lite` 档)。
- 数据记录:`CsvLogger` 的列 `encoder_2_position`、`encoder_2_velocity`。
- 直接用 `Controller`:`result.values[Register.ENCODER_2_POSITION]`;使用时需要把对应 `_extra` 加进 `QueryResolution`。

```python
from moteus.control_manager import ControlManager

cm = ControlManager(cycle_hz=200, query_profile='full')
cm.connect(...)                         # 参数见 control_manager.py:222
st = cm.get_status_one(1)
print(st.encoder_2_position, st.encoder_2_velocity, st.encoder_validity)
```

### 11.4 `ControllerStatus` 字段

`controller_id, mode, position, velocity, torque, voltage, temperature, fault, trajectory_complete, abs_position, encoder_0_position, encoder_0_velocity, encoder_1_position, encoder_1_velocity, encoder_2_position, encoder_2_velocity, encoder_validity, last_update`

`last_update` 为 `time.monotonic()` 时间戳,可用于判断数据是否过期。

## 12. 源码索引

| 内容 | 位置 |
|---|---|
| 寄存器、Mode、缩放、`parse_registers` | `moteus/moteus/protocol.py` |
| 子帧编解码、`WriteCombiner`、varuint | `moteus/moteus/multiplex.py` |
| `Controller.make_*`、`QueryResolution`、`PositionResolution` | `moteus/moteus/moteus.py` |
| 仲裁 ID 构造、路由、`discover` | `moteus/moteus/transport.py` |
| `ControlManager`、`ControllerStatus`、`CsvLogger` | `moteus/moteus/control_manager.py` |
