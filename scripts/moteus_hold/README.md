# moteus_hold —— 开机自启的位置保持守护进程

开机后等待 CAN 接口（默认 `can1`）出现，持续向电机 1/2/3 发送位置保持指令；
通过命令行工具 `moteus-holdctl` 调试：按步长点动、设置目标、实时查看位置/速度/扭矩/电流。

| 文件 | 作用 |
|------|------|
| `moteus_hold_daemon.py` | 守护进程（控制循环 + Unix socket 调试接口） |
| `moteus_holdctl.py` | 调试命令行（交互模式 / 单条命令 / 键盘点动） |
| `moteus-hold.service` | systemd 服务模板 |
| `moteus-hold.default` | 参数文件模板，安装到 `/etc/default/moteus-hold` |
| `install.sh` / `uninstall.sh` | 安装 / 卸载服务 |

单位：位置 rev、速度 rev/s、扭矩 Nm、电流 A（均为 moteus 输出端，已含减速比配置）。

## 部署

下文 `<用户>` 指运行服务的普通用户，由部署者自行指定（`install.sh --user` 必填）。
服务以该用户身份运行，通过 systemd 授予 `CAP_NET_ADMIN` 来配置 CAN 接口，
**不需要 root，也不需要 sudo 免密**。

部署机上需要：`<用户>` 使用的 Python 能 `import moteus` 和 `python-can`（pip 安装自行完成），
以及本目录 `moteus_hold/`。两种放法任选：

| 方式 | 部署机上的内容 | moteus 来源 |
|------|----------------|-------------|
| A. 整个仓库 | 仓库目录 | 仓库里的 `moteus/`（优先于已安装版本） |
| B. 只拷本目录 | pip 安装的 moteus + `moteus_hold/` 目录 | pip 安装的 moteus |

### 方式 B：wheel + 单独拷贝本目录（部署机不放仓库）

在开发机上：

```bash
cd moteus-python-packages
pip wheel --no-deps ./moteus -w dist/          # 生成 dist/moteus-<版本>-py3-none-any.whl
tar czf moteus_hold.tar.gz -C scripts moteus_hold
scp dist/moteus-*.whl moteus_hold.tar.gz <部署机>:/tmp/
```

在部署机上：

```bash
pip install /tmp/moteus-*.whl                  # 以 <用户> 身份，或装进 venv（自行操作）
sudo mkdir -p /opt && sudo tar xzf /tmp/moteus_hold.tar.gz -C /opt
cd /opt/moteus_hold
sudo ./install.sh --user <用户>                # venv: 加 --python /path/to/venv/bin/python
```

- 服务直接运行该目录下的脚本，**安装后不要移动或删除该目录**（`/opt/moteus_hold` 这类位置即可，
  `<用户>` 需要可读）。
- wheel 必须包含本工具依赖的库改动（`PythonCanDevice` 的 `restart_ms` 参数）。`install.sh`
  会打印实际使用的 moteus 路径和版本，旧版本会给出警告。建议每次发布前提升版本号，避免和旧的
  wheel 同名。
- 拷贝过去的仍是 Python 源码（wheel 里的 moteus 也是 `.py`）。只想避免直接可读，可以用
  `python3 -m compileall -b` 只保留 `.pyc`（需与部署机 Python 小版本一致，服务和 `moteus-holdctl`
  的路径要相应改成 `.pyc`，保护很弱）；
  真正不留源码需要 Cython/Nuitka 等编译方案。

### 方式 A：部署机上放整个仓库

```bash
cd <仓库>/scripts/moteus_hold
sudo ./install.sh --user <用户>
```

之后 `git pull` 更新代码，再 `sudo systemctl restart moteus-hold` 即可。

### 先手动跑一遍（推荐）

```bash
# 无硬件：内置模拟器，验证命令行工具
python3 moteus_hold_daemon.py --sim 1,2,3
python3 moteus_holdctl.py                       # 另开一个终端

# 真实总线（手动运行时配置接口需要权限，用 sudo）
sudo python3 moteus_hold_daemon.py --channel can1 --ids 1,2,3
```

手动运行时 socket 默认在 `/tmp/moteus-hold.sock`，`moteus_holdctl.py` 会自动找到。
Ctrl-C 退出时会向在线电机发送 stop。

### install.sh 做的事

检查 `<用户>` 能否 import 依赖 → 写入 `/etc/systemd/system/moteus-hold.service` →
首次安装时写入 `/etc/default/moteus-hold` → 创建 `/usr/local/bin/moteus-holdctl` →
`systemctl enable` 并启动（`--no-start` 只设置开机自启）。

### 修改参数

```bash
sudo nano /etc/default/moteus-hold     # 改 MOTEUS_HOLD_ARGS
sudo systemctl restart moteus-hold
```

速度/加速度/扭矩上限等也可以运行时用 `moteus-holdctl param ...` 临时修改（重启后恢复）。

### 日常运维

```bash
systemctl status moteus-hold
journalctl -u moteus-hold -f           # 上线/离线、模式切换、故障、调试命令都会记录
sudo systemctl restart moteus-hold
sudo systemctl stop moteus-hold        # 默认向电机发 stop（--on-exit none 则交给看门狗）
sudo ./uninstall.sh                    # 卸载
```

调试接口 socket 为 `/run/moteus-hold/ctl.sock`，属主 `<用户>`、权限 0660，
`moteus-holdctl` 需以 `<用户>` 或其所在组的用户运行。

## 调试命令行

```bash
moteus-holdctl                 # 交互模式
moteus-holdctl status          # 单条命令
moteus-holdctl watch 10        # 10 Hz 持续刷新
moteus-holdctl jog             # 键盘点动
```

交互模式示例：

```
holdctl> ss 0.02          # 步长 0.02 rev
holdctl> 1+               # 电机 1 目标 +0.02
holdctl> 2-0.05           # 电机 2 目标 -0.05
holdctl> all+             # 三个电机一起 +0.02
holdctl> set 3 0.25       # 电机 3 目标 0.25 rev
holdctl> w                # 持续刷新，Ctrl-C 返回
```

每条命令执行后都会打印状态表：

```
bus can1  100/100 Hz  cycle 1.10 ms  timeout 0  reset 0  up 35s
vel_lim 0.5  acc_lim 2  max_tq ---  max_step 0.25  max_move 1
 ID STATE MODE             TARGET       POS      VEL  TORQUE   Iq(A)   Id(A)  VBUS TEMP FLT   AGE
  1 hold  POSITION         0.1000    0.0993   0.0114  -0.002  -0.001   0.000  24.0   35   0     7
```

- `STATE`：`hold` 保持中 / `free` 在线但未保持 / `off` 离线
- `AGE`：距离最近一次收到该电机应答的毫秒数

| 命令 | 说明 |
|------|------|
| `<id>+` `<id>-` | 按当前步长移动目标（`all+` 全部） |
| `<id>+<v>` | 按给定增量移动，如 `1+0.05` |
| `step <id> <delta>` / `set <id> <pos>` | 相对 / 绝对目标 |
| `ss [v]` | 查看/设置步长（默认 0.01 rev，可用 `--step` 指定初值） |
| `hold [id]` | 在当前位置保持 |
| `stop [id]` | 释放电机（moteus stop），掉线重连后也不会自动保持，需再 `hold` |
| `clear <id>` | 发 stop 清除 FAULT/TIMEOUT，然后在当前位置保持 |
| `limit <id> <lo> <hi>` | 软件限位（`none` 表示不限） |
| `param [name value]` | 查看/修改运行参数 |

键盘点动（`jog`）：数字键选电机、`a` 全部、`↑`/`+` 正向、`↓`/`-` 反向、
`[` `]` 步长减半/加倍、`h` 保持、`x` 释放、`c` 清故障、`q` 退出。

## 行为与安全

- **上电不跳变**：电机首次应答（以及掉线后重新应答，例如断电重启）时，以其当时的实际位置为保持目标；
  离线期间只发 query，不会把旧目标发给刚上电的电机。
- 三个电机在同一轮 `transport.cycle()` 里下发；某个电机不应答时只把它标记离线，其余电机照常控制。
- 移动由 moteus 板上轨迹生成器按 `velocity_limit` / `accel_limit` 平滑执行。
- 每次 `step` 不超过 `max_step`（默认 0.25 rev）；目标与当前位置相差不超过 `max_move`（默认 1 rev）；
  目标会被钳制到 `--limit` 软件限位内。
- 看门狗：每帧带 `watchdog_timeout`（默认 0.25 s）。守护进程卡死或退出后，moteus 按自身
  `servo.timeout_mode` 配置处理。看门狗超时后守护进程恢复时会先发 stop 清除 TIMEOUT，再保持在当前位置。
- FAULT：停止发送位置指令并记录日志，需人工检查后 `clear`。
- 总线持续无应答（例如发送队列卡死）时会重建连接并重新 down/up 接口，按 3 s 起指数退避。
- 软件限位和这些检查不能替代机械限位与急停。

## 注意

- **接口名可能变化**：`can0/can1/can2` 按驱动加载顺序分配。如果机器上同时有板载 CAN 和 USB-CAN，
  重启后编号可能变化，服务会连到错误的适配器。部署前用 `ip -d link show can1` 确认驱动
  （`f81601a` / `gs_usb` 等）；需要固定名称时可以用 udev 规则或 systemd `.link` 文件按设备路径重命名。
- Fintek F81601 板载 CAN 的驱动默认关闭了发送延迟补偿（TDC），5 Mbps 数据段发送不稳定。用板载口时
  建议在参数里加 `--disable-brs`，或以 `disable_ssp=0` 重新加载 `f81601a` 驱动。
