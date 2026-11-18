# 三自由度并联机构控制与可视化

`parallel_delta_control` 是同轴驱动 3-RRR 球面并联机构的三电机控制工程。它把 `parallel_delta` 的机构模型与 `moteus_hold` 的共享 CAN Transport、周期位置命令、watchdog 和故障处理结合起来，提供：

- **正解模式**：输入三个相对直立零位的电机轴角（°），路径延续 FK 求 RPY，然后协调运动；
- **逆解模式**：输入标准 roll/pitch/yaw（°），解析 IK 求三个电机角，FK 回验后协调运动；
- 直立软件零位标定、当前位置保持、整组 stop、故障清除；
- 机构、电机 ID/方向、传动比、限位和数值求解参数的动态配置；
- 本地网页三维几何显示、目标/实际姿态对比和实时电机反馈；
- moteus 内置三电机模拟器，无硬件也可测试完整命令与反馈链路。

> 三维页面使用原生 Canvas 投影，不需要 Node.js、CDN 或外部前端依赖。模拟是命令/运动学模拟，不包含机构动力学、碰撞、背隙或传动弹性。

## 1. 坐标、零位和单位

- 长度：mm；网页角度：degree；moteus 位置/速度/加速度：rev、rev/s、rev/s²。
- 平台主动旋转固定为 `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`。
- 默认范围：roll ±15°、pitch ±15°、yaw ±60°。
- **坐标与支链顺序**：电机直立轴角为 **30°、270°、150°**。出轴侧 `roots/Bw` 方位为 **210°、90°、330°**，因此 M1→M2→M3 按顺时针 120° 分布；每个出轴点到对应平台铰点 `Bv` 仍为 **逆时针 +90°**，平台铰点方位为 **300°、180°、60°**。
- **实物层叠顺序**：底层 M1、中层 M2、顶层 M3；几何显示中的电机高度按此排列。
- **直立零位**：`R=I`、RPY=(0,0,0)，三个网页电机相对角均为 0°。
- 电机角正方向通过 `directions` 配置；每个方向同时作用于反馈和命令。
- 默认总减速比：`motor_reduction × belt_reduction = 8 × 2.22 = 17.76`。

转换关系（`z_i` 为标定时原始 rev，`s_i` 为方向）：

```text
motor_deg_i = s_i × (raw_rev_i - z_i) × 360
q_i         = q_home_i + motor_deg_i / total_reduction
raw_target_i = z_i + s_i × motor_target_deg_i / 360
```

连续位置不取模。解析 IK 的 `[-180°,180°)` 输出会相对当前连续套轴角展开，避免跨圈跳变。

## 2. 默认机构参数

默认机构参数按 `parallel_delta` 工具截图中的实际运行值设置（不是源码出厂默认值）：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| H | 85 mm | 共同球心 C 相对基座的高度 |
| D | 80 mm | 平台中心任务球面直径 |
| e | 0 mm | 球心/中心柱沿 +X 偏移 |
| alpha1 / alpha2 | 45° / 60° | 近端、远端曲杆圆心角 |
| beta | 45° | 动平台锥角 |
| h_mid / d_stack | 15 / 7 mm | 套轴中层高度与层间距 |
| column_radius | 14 mm | 中心柱外观半径 |
| gear_radius / pinion_radius | 28 / 8 mm | 外观和底座包络参数 |
| rod_diameter / motor_radius | 8 / 14 mm | 杆件/电机外观参数 |

当前前端默认传动与安全参数：ID `[1,2,3]`、方向 `[-1,1,1]`、总减速比 `17.76`；速度/加速度上限均为 `100`，状态发布 `100 Hz`，反馈姿态裕量 `10°`，电机软限位 `±1066°`，`max_move=5000°`，`kp_scale=kd_scale=1`，`maximum_torque=1 N·m`，FK 容差 `1e-9`、最大迭代 `300`，姿态分支阈值 `300°`，反馈跳变阈值 `1000°`。这些值来自当前实物配置，不能替代机械限位和独立急停。

几何修改只改变模型；它不会自动证明实物无干涉或可达。

## 3. 安装

要求 Python ≥3.10。在整个 `moteus-python-packages` 仓库中运行时，程序优先加载仓库内的 `moteus/`，无需另外安装 moteus：

```powershell
python -m pip install numpy fastapi "uvicorn[standard]" httpx
cd moteus-python-packages\scripts\parallel_delta_control
```

独立部署本目录时，安装完整依赖：

```bash
python3 -m pip install -r requirements.txt
```

## 4. 无硬件模拟

```bash
python main.py --sim --host 127.0.0.1 --port 8080
```

打开 <http://127.0.0.1:8080>。模拟器上电位置已知，因此会自动完成直立标定并进入保持状态。页面可执行：

1. 逆解输入 RPY，例如 `[0, 0, 10]`，下发后三个电机目标应各为 160°；
2. 正解输入三个电机角，例如 `[16, 32, -16]`，查看返回 RPY；
3. 观察目标半透明模型与实际/模拟反馈模型；
4. 测试 hold、stop、clear 和动态配置。

也可以使用纯计算 CLI，不启动网页或 CAN：

```bash
python main.py fk 16 32 -16
python main.py ik 0 0 60
python main.py ik 15 -15 -60
```

输出包含连续套轴角、电机角、RPY、条件指标、残差和三维铰点几何。

## 5. 实物运行与每次上电标定

### Windows 11 + candleLightFD 只读链路检查

Windows 设备管理器应能识别 `candleLightFD gs_usb`。安装通信依赖后，可使用本目录的直接 `candle_api` 工具，只发送查询：

```powershell
python -m pip install "python-can>=3.3" "candle-api==0.0.9" pywin32
python canfd_query.py --ids 1,2,3 --disable-brs
```

需要连续实物调试时，使用持久 Candle 后端，避免反复打开/关闭 USB 设备：

```powershell
python main.py --hardware --host 127.0.0.1 --port 8081
```

Windows 实物模式自动使用该直接 `candle_api` 通道并禁用 BRS；Linux 仍使用 SocketCAN。

该工具使用 1 Mbps 标称位率、5 Mbps 数据位率；`--disable-brs` 是当前 candleLightFD 与 moteus 已验证所需的设置。它不会构造位置、使能、stop、rezero 或配置命令。仓库的 `CandleDevice` 目前依赖未在 PyPI `python-can 4.6.1` 中注册的 `candle` 后端，因此 Windows 调试使用此直接 API 工具。

### Linux SocketCAN

实物后端沿用 `moteus_hold` 的 Linux SocketCAN 方式。先确认 CAN 接口和权限：

```bash
python3 main.py --hardware --channel can1 --ids 1,2,3
```

### 固件坐标约定

本程序把 moteus `POSITION` 当作**电机轴 rev**，默认要求固件 `motor_position.rotor_to_output_ratio=1`，然后在软件中应用 8×2 减速。若固件已经配置了输出减速比，必须先统一约定，不能重复换算。

### 单编码器、掉电不记圈数

`rezero` 或 `set_output_nearest` 不能找回断电丢失的整数圈。因此本程序采用以下流程：

1. 程序启动、进程重启、控制器重连或位置跳变后，均为未标定；
2. 保持电机失能，人工把机构置于直立状态，并准备独立硬件急停；
3. 在网页确认“机构已人工直立”，执行**确认直立并标定**；
4. 程序只捕获三个当前位置作为本进程软件零位，不写控制器持久配置；
5. 标定后以当前位置为首目标并保持，之后允许正逆解命令；
6. 修改机构、ID、方向或减速比等配置后必须重新标定。

若机构失能后会坠落或自行运动，必须先提供可靠机械支撑，不能依赖本软件。

### 可选：正负机械极限标零

网页“极限位置标零”提供不依赖人工精确直立的另一种零位标定。启动前必须先 Stop，流程在同一个控制循环中执行：

1. 三轴同时以标定速度、加速度和扭矩/电流限制正转，寻找正转机械极限；
2. 每轴必须同时满足反馈新鲜、速度低于阈值、位置无漂移，并有预期方向的堵转电流；三轴持续稳定 `1 s` 后，记录窗口内平均位置；
3. 在正反转之间执行 `settle_s`（默认 0.5 s）的 `make_stop()` 短暂复位，清除 moteus 内部控制位置与 PID 积分，再开始反转；
4. 三轴反转，以相同方法记录反转极限；
5. 计算三轴正负行程，要求行程大于最小值且 `(max-min)/mean` 不超过 `max_span_imbalance`（默认 10%），否则判定可能卡滞并失败；
6. 以正负极限的平均 raw rev 为零位，使用低速五次轨迹归零，反馈进入容差后保持；
7. 任一阶段超时、Q 轴过流、控制器故障、反馈丢失或正解失败都会整组失能并作废零位；Stop 随时取消流程，不会自动重试。

默认标定速度 `1.5 rev/s`、加速度 `3 rev/s²`、工作电流 `drive_current_a=0.5 A`、独立保护阈值 `max_q_current_a=6 A`、堵转速度阈值 `0.04 rev/s`、位置漂移阈值 `2°`、单阶段超时 `30 s`。堵转搜索使用 `position=NaN` 的纯速度位置命令，速度和加速度由 moteus 固件轨迹器控制，包含换向减速，不使用上位机直接电流调速。

固件源码表明 `ilimit_scale` 只缩放 PID 积分项的 `ilimit`，并不限制 P/D 或最终 Q 轴总电流；`fixed_current_override` 是固定电流/合成电角度路径，也不是普通速度模式的限流。因此最终输出通过 `maximum_torque` 限制，并按实际电机 Kv 换算：`Kt=(3/2)*(1/√3)*(60/(2π))/Kv`，在现有 `rotor_to_output_ratio=1` 约定下 `maximum_torque=drive_current_a*Kt`。`motor_kv` 和真实固件 `servo.max_current_A` 都必须填写，当前默认分别为 `127.2` 和 `2 A`，更换电机或固件配置时必须同步修改。

故障 102（指令最大扭矩限幅）全局仅记录警告，不锁存或中断流程；故障 99（固件电流限值）、其他故障、Mode.FAULT/TIMEOUT、过流和超时仍立即中止。保护阈值必须大于工作电流。回零阶段使用位置轨迹和 `ilimit_scale=drive_current_a/firmware_max_current_a`。使用前必须确认 Kv、固件限值、机械堵转强度和热承受能力。该功能不能替代独立硬件急停，也不保证堵转不会损坏机构。

## 6. 动态配置与保存

网页“参数配置”编辑的是草稿：

- **应用**：仅当前进程生效；
- **应用并保存**：原子写入 `config.json`，下次启动加载；
- **恢复默认**：回到上述默认参数。

实物运动/保持期间拒绝配置修改，先 stop。修改后标定失效。活动零位不会作为有效标定跨上电恢复。

默认软限位 ±1300° 和 max_move 2600°仅覆盖理论需求包络，**不是实机安全额定值**。机构工作空间按截图参数 α1=45°、α2=60°、β=45° 重新计算；不可达或接近奇异的目标仍由后端拒绝。实机应根据机械限位、负载和低速试验收紧这些值。

## 7. Web API

| 方法 | 路径 | 内容 |
|---|---|---|
| GET | `/api/state` | 完整状态、配置、目标/实际姿态和几何 |
| POST | `/api/calibrate` | 捕获直立软件零位并保持 |
| POST | `/api/calibrate/extreme` | 启动三轴正负机械极限标零；需先 Stop，可用 `/api/stop` 取消 |
| POST | `/api/hold` | 以当前反馈位置保持 |
| POST | `/api/stop` | 整组停止/失能 |
| POST | `/api/clear` | 清 FAULT/TIMEOUT，成功后保持 |
| POST | `/api/command/motors` | `{"motor_deg":[d1,d2,d3],"duration_s":2.0}` |
| POST | `/api/command/rpy` | `{"rpy_deg":[roll,pitch,yaw],"duration_s":2.0}` |
| POST | `/api/config?save=true` | 完整配置对象，校验、保存并应用 |
| POST | `/api/config/reset?save=true` | 恢复默认配置 |
| WS | `/api/ws` | 默认约 20 Hz 状态推送 |

`duration_s` 是本次命令的目标到达时间（秒），正解和逆解均可在每次发送时单独指定。后端据此生成三轴同步五次曲线；省略该字段时仍按速度和加速度上限自动计算最短时长。若指定时间过短、会导致规划曲线突破这些上限，命令会被拒绝且不会部分更新目标。网页默认填写 2 秒。

FastAPI 文档位于 `/api/docs`。服务默认只绑定 localhost；不要在无认证和额外安全隔离的情况下暴露到不受信任网络。

## 8. 安全行为

- 三轴目标在全部计算和限幅通过后才原子更新，失败不部分运动；
- 每个位置周期命令包含速度、加速度、可选扭矩和 watchdog；极限标定额外携带 `ilimit_scale` 并监测 Q 轴电流；
- 任一关键电机离线、反馈过期、FAULT/TIMEOUT、位置跳变或反馈 FK 失败会整组停止；
- 目标 FK 采用最大 2°套轴角步长的路径延续；中间数值步骤检查收敛、条件指标和分支连续性，最终目标严格检查 RPY；
- 若逆解的直接路径接近奇异或不收敛，不放宽阈值；自动分别预检“当前位置→直立点”和“直立点→目标”，通过后用**连续三次样条**实际绕行：端点零速、中间 waypoint 无停顿，速度和加速度连续，指定到达时间为总时长；实际样条还会采样复核 FK、软限位和条件数；
- 三轴运动由上层五次多项式同步轨迹按同一归一化进度下发，并带速度前馈，避免三个控制器因行程不同而各自提前/延后完成；
- 目标指令严格使用 RPY 限位；实际反馈使用 `feedback_rpy_margin_deg`（基础默认 1°）加上本次预检路径超出命令边界的部分，允许已预检的过程姿态和轻微跟踪误差，避免正常复位/保持被误锁故障；
- 停止网页或断开 WebSocket 不会停止独立控制循环；进程退出会 best-effort 批量 stop；
- 软件 stop、软限位、FK 条件和网页确认均不能替代机械限位、控制器底层保护与独立硬件急停。

## 9. 测试

在本目录执行：

```bash
python -m unittest discover -s tests -v
```

测试覆盖 RPY/球冠坐标回环、637 点正逆解、跨圈/减速比/任意零位、配置校验与原子保存、标定状态机、故障/超时、API 和 moteus 模拟器端到端。

实机验收按以下顺序低速进行：

1. 独立急停和机械支撑就位，输出失能，核对 ID/支链映射；
2. 人工直立标定，检查电机相对角为 0、q=30/270/150°、RPY=0；
3. 当前位置保持，确认无首帧跳变，验证 stop；
4. 极小点动逐一确认电机方向和 16:1 换算；
5. 先做小电机角 FK，再做小 RPY IK；
6. 验证断线、反馈超时、故障复位、重启后标定失效；
7. 只有完成机械干涉、实际限位、负载和温升评估后，才逐步扩大范围。

## 来源

运动学公式与默认几何移植自同工作区 `parallel_delta/coaxial_delta`；控制通信结构参考本仓库 `scripts/moteus_hold` 和 `moteus` Python API。详见 `PROVENANCE.md`。
