"""moteus 控制器离线模拟器

用法一：作为 Transport 注入
    from moteus.simulator import SimulatedTransport
    transport = SimulatedTransport([1, 2, 3])     # 模拟 ID=1,2,3 三个控制器
    c = moteus.Controller(id=1, transport=transport)

用法二：替换全局单例
    from moteus.simulator import SimulatedTransport, patch_singleton
    with patch_singleton([1, 2]):
        c = moteus.Controller(id=1)               # transport=None 也能用
        await c.set_position(position=0.5)

用法三：直接运行演示
    python -m moteus.simulator
    python -m moteus.simulator --ids 1 2 3 --verbose
"""

import argparse
import asyncio
import dataclasses
import io
import math
import struct
import time
import typing
from contextlib import contextmanager

from .protocol import Mode, Register, Writer, parse_registers
from .multiplex import (
    INT8, INT16, INT32, F32,
    REPLY_BASE, TYPES, saturate,
)
from .transport import Transport
from .transport_device import Frame, TransportDevice


# ---------------------------------------------------------------------------
# 寄存器值编解码
# ---------------------------------------------------------------------------

# (int8_scale, int16_scale, int32_scale) — 与 protocol.Writer 保持一致
_SCALES: dict = {
    Register.POSITION:              (0.01,           0.0001,          0.00001),
    Register.VELOCITY:              (0.1,            0.00025,         0.00001),
    Register.TORQUE:                (0.5,            0.01,            0.001),
    Register.Q_CURRENT:             (1.0,            0.1,             0.001),
    Register.D_CURRENT:             (1.0,            0.1,             0.001),
    Register.ABS_POSITION:          (0.01,           0.0001,          0.00001),
    Register.POWER:                 (10.0,           0.05,            0.0001),
    Register.MOTOR_TEMPERATURE:     (1.0,            0.1,             0.001),
    Register.VOLTAGE:               (0.5,            0.1,             0.001),
    Register.TEMPERATURE:           (1.0,            0.1,             0.001),
    Register.CONTROL_POSITION:      (0.01,           0.0001,          0.00001),
    Register.CONTROL_VELOCITY:      (0.1,            0.00025,         0.00001),
    Register.CONTROL_TORQUE:        (0.5,            0.01,            0.001),
    Register.POSITION_ERROR:        (0.01,           0.0001,          0.00001),
    Register.VELOCITY_ERROR:        (0.1,            0.00025,         0.00001),
    Register.TORQUE_ERROR:          (0.5,            0.01,            0.001),
    Register.ENCODER_0_POSITION:    (0.01,           0.0001,          0.00001),
    Register.ENCODER_0_VELOCITY:    (0.1,            0.00025,         0.00001),
}


def _write_varuint(buf: io.BytesIO, value: int) -> None:
    while True:
        this_byte = value & 0x7f
        value >>= 7
        this_byte |= 0x80 if value else 0x00
        buf.write(bytes([this_byte]))
        if value == 0:
            break


def _encode_value(reg: int, resolution: int, physical: float) -> bytes:
    """将物理量编码为 multiplex 协议的线路字节。"""
    if physical is None or (isinstance(physical, float) and math.isnan(physical)):
        # NaN 用各类型最小值表示
        nan_vals = {INT8: -128, INT16: -32768, INT32: -2147483648}
        if resolution == F32:
            return TYPES[F32].pack(math.nan)
        return TYPES[resolution].pack(nan_vals[resolution])

    scales = _SCALES.get(reg)
    if scales is None or resolution == F32:
        if resolution == F32:
            return TYPES[F32].pack(float(physical))
        # 整数型寄存器（MODE、FAULT 等）
        size_bits = [8, 16, 32, 32][resolution]
        lo = -(1 << (size_bits - 1))
        hi = (1 << (size_bits - 1)) - 1
        return TYPES[resolution].pack(max(lo, min(hi, int(round(physical)))))

    scale = scales[resolution]
    raw = saturate(physical, resolution, scale)
    return TYPES[resolution].pack(raw)


def _build_reply(queries: typing.List[typing.Tuple[int, int]],
                 values: dict) -> bytes:
    """根据查询列表 [(register, resolution), ...] 和值字典构造 REPLY 帧数据。"""
    buf = io.BytesIO()
    for reg, resolution in queries:
        if reg not in values:
            continue
        # REPLY 子帧：0x2n | (resolution<<2) | 1  （1 个寄存器）
        cmd = REPLY_BASE | (resolution << 2) | 0x01
        buf.write(bytes([cmd]))
        _write_varuint(buf, reg)
        buf.write(_encode_value(reg, resolution, values[reg]))
    return buf.getvalue()


# ---------------------------------------------------------------------------
# 控制器状态机
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class ControllerState:
    """单个虚拟 moteus 控制器的物理状态与命令状态。"""

    can_id: int

    # 状态寄存器
    mode: int = dataclasses.field(default=int(Mode.STOPPED))
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    voltage: float = 24.0
    temperature: float = 35.0
    fault: int = 0
    trajectory_complete: int = 0

    # 命令寄存器（来自上位机写入）
    cmd_position: float = math.nan
    cmd_velocity: float = math.nan
    cmd_feedforward: float = 0.0
    cmd_kp_scale: float = 1.0
    cmd_kd_scale: float = 1.0
    cmd_kiv_scale: float = 1.0
    cmd_max_torque: float = 5.0
    cmd_stop_position: float = math.nan
    cmd_velocity_limit: float = math.nan
    cmd_accel_limit: float = math.nan

    # 物理仿真参数
    kp: float = 10.0        # 位置增益 (rev/s / rev)
    kd: float = 0.5         # 速度阻尼 (rev/s² per rev/s error)
    ki: float = 0.0         # 速度积分增益（默认 0 禁用，与固件 ki_velocity 一致）
    ilimit: float = 0.0     # 速度积分限幅（Nm，0 = 积分钳零，与固件 ilimit_velocity 一致）
    inertia: float = 0.05   # 等效惯量 (rev/s² per Nm)

    # 内部状态
    vel_integral: float = 0.0  # 速度误差积分（对应固件 velocity_integral_Nm）

    _last_t: float = dataclasses.field(default_factory=time.monotonic)

    # ------------------------------------------------------------------ #

    def step(self, now: float) -> None:
        dt = now - self._last_t
        self._last_t = now
        dt = min(dt, 0.05)  # 防止首帧 dt 过大

        if self.mode in (int(Mode.STOPPED), int(Mode.FAULT)):
            self.velocity = 0.0
            self.torque = 0.0
            self.vel_integral = 0.0
            self.trajectory_complete = 0
            return

        if self.mode == int(Mode.POSITION):
            self._step_position(dt)
        elif self.mode == int(Mode.ZERO_VELOCITY):
            self._step_zero_velocity(dt)
        elif self.mode == int(Mode.BRAKE):
            self.velocity *= max(0.0, 1.0 - 30.0 * dt)
            self.torque = -self.kd * self.velocity
        elif self.mode == int(Mode.STAY_WITHIN):
            self._step_position(dt)   # 简化：复用位置模式
        else:
            pass  # VOLTAGE_FOC、CURRENT 等：保持当前状态

        # 温度随扭矩升高
        self.temperature = 35.0 + abs(self.torque) * 3.0

    def _step_position(self, dt: float) -> None:
        target = self.cmd_position if not math.isnan(self.cmd_position) else self.position
        kp = self.kp * self.cmd_kp_scale
        kd = self.kd * self.cmd_kd_scale

        pos_err = target - self.position
        vel_ff = 0.0 if math.isnan(self.cmd_velocity) else self.cmd_velocity

        # 速度限幅
        desired_vel = kp * pos_err + vel_ff
        if not math.isnan(self.cmd_velocity_limit):
            vl = abs(self.cmd_velocity_limit)
            desired_vel = max(-vl, min(vl, desired_vel))

        vel_err = desired_vel - self.velocity

        # 速度环积分（对应固件 ki_velocity / ilimit_velocity，默认禁用）
        ki_eff = self.ki * self.cmd_kiv_scale
        if ki_eff != 0.0 and self.ilimit > 0.0:
            vi_old = self.vel_integral
            vi_new = max(-self.ilimit,
                         min(self.ilimit, vi_old + vel_err * ki_eff * dt))
            torque_raw = kd * vel_err + vi_new + self.cmd_feedforward
            # 力矩饱和且积分继续推向饱和方向时冻结积分（简化条件积分）
            if abs(torque_raw) > self.cmd_max_torque and \
                    (vi_new - vi_old) * torque_raw > 0.0:
                vi_new = vi_old
                torque_raw = kd * vel_err + vi_old + self.cmd_feedforward
            self.vel_integral = vi_new
        else:
            self.vel_integral = 0.0
            torque_raw = kd * vel_err + self.cmd_feedforward

        # 扭矩限幅
        torque_raw = max(-self.cmd_max_torque, min(self.cmd_max_torque, torque_raw))
        self.torque = torque_raw

        # 加速度限幅
        accel = torque_raw / self.inertia
        if not math.isnan(self.cmd_accel_limit):
            al = abs(self.cmd_accel_limit)
            accel = max(-al, min(al, accel))

        self.velocity += accel * dt
        self.position += self.velocity * dt

        # 判断轨迹完成
        if abs(pos_err) < 0.001 and abs(self.velocity) < 0.005:
            self.trajectory_complete = 1
        else:
            self.trajectory_complete = 0

    def _step_zero_velocity(self, dt: float) -> None:
        torque_raw = -self.kd * self.velocity
        torque_raw = max(-self.cmd_max_torque, min(self.cmd_max_torque, torque_raw))
        self.torque = torque_raw
        accel = torque_raw / self.inertia
        self.velocity += accel * dt
        self.position += self.velocity * dt

    def apply_writes(self, writes: dict) -> None:
        """将上位机写入的寄存器应用到状态机。"""
        if Register.MODE in writes:
            new_mode = int(writes[Register.MODE])
            if new_mode != self.mode:
                # 与固件一致：任何模式切换都清零位置环/速度环积分
                self.vel_integral = 0.0
            self.mode = new_mode
            if new_mode == int(Mode.STOPPED):
                self.velocity = 0.0
                self.torque = 0.0

        _W = {
            Register.COMMAND_POSITION:          'cmd_position',
            Register.COMMAND_VELOCITY:          'cmd_velocity',
            Register.COMMAND_FEEDFORWARD_TORQUE:'cmd_feedforward',
            Register.COMMAND_KP_SCALE:          'cmd_kp_scale',
            Register.COMMAND_KD_SCALE:          'cmd_kd_scale',
            Register.COMMAND_KIV_SCALE:         'cmd_kiv_scale',
            Register.COMMAND_WITHIN_KIV_SCALE:  'cmd_kiv_scale',
            Register.COMMAND_POSITION_MAX_TORQUE:'cmd_max_torque',
            Register.COMMAND_STOP_POSITION:     'cmd_stop_position',
            Register.COMMAND_VELOCITY_LIMIT:    'cmd_velocity_limit',
            Register.COMMAND_ACCEL_LIMIT:       'cmd_accel_limit',
        }
        for reg, attr in _W.items():
            if reg in writes:
                setattr(self, attr, float(writes[reg]))

        # 处理 SET_OUTPUT_NEAREST（rezero）
        if Register.SET_OUTPUT_NEAREST in writes:
            self.position = float(writes[Register.SET_OUTPUT_NEAREST])
            self.velocity = 0.0

    def register_values(self) -> dict:
        """返回所有可回复寄存器的当前物理值。"""
        return {
            Register.MODE:              self.mode,
            Register.POSITION:          self.position,
            Register.VELOCITY:          self.velocity,
            Register.TORQUE:            self.torque,
            Register.VOLTAGE:           self.voltage,
            Register.TEMPERATURE:       self.temperature,
            Register.FAULT:             self.fault,
            Register.TRAJECTORY_COMPLETE: self.trajectory_complete,
            Register.MOTOR_TEMPERATURE: self.temperature + 5.0,
            Register.HOME_STATE:        0,
            Register.REZERO_STATE:      0,
            Register.ABS_POSITION:      self.position % 1.0,
            Register.Q_CURRENT:         self.torque * 0.3,
            Register.D_CURRENT:         0.0,
            Register.POWER:             abs(self.torque * self.velocity) * 2.0 * math.pi,
            Register.ENCODER_0_POSITION: self.position,
            Register.ENCODER_0_VELOCITY: self.velocity,
            Register.CONTROL_POSITION:  self.position,
            Register.CONTROL_VELOCITY:  self.velocity,
            Register.CONTROL_TORQUE:    self.torque,
            Register.POSITION_ERROR:    (self.cmd_position - self.position)
                                        if not math.isnan(self.cmd_position) else 0.0,
            Register.VELOCITY_ERROR:    0.0,
            Register.TORQUE_ERROR:      0.0,
            Register.FIRMWARE_VERSION:  0x00010017,
            Register.MODEL_NUMBER:      0x0005,
            Register.MULTIPLEX_ID:      self.can_id,
            Register.SERIAL_NUMBER1:    0x12345678 ^ (self.can_id << 16),
            Register.SERIAL_NUMBER2:    0xDEADBEEF,
            Register.SERIAL_NUMBER3:    0xCAFEBABE,
            # UUID（仅返回非零值，UUID_MASK_CAPABLE=1 触发 UUID 路由）
            Register.UUID1:             0x11223344 ^ self.can_id,
            Register.UUID2:             0x55667788,
            Register.UUID3:             0x99AABBCC,
            Register.UUID4:             0xDDEEFF00,
            Register.UUID_MASK_CAPABLE: 0,   # 0 = 不支持 UUID 寻址（保持 CAN ID 路由）
        }


# ---------------------------------------------------------------------------
# SimulatedTransportDevice
# ---------------------------------------------------------------------------

class SimulatedTransportDevice(TransportDevice):
    """将 Controller.cycle() 帧截获并通过状态机回复，无需任何真实硬件。

    Args:
        controller_ids: 要模拟的 CAN ID 列表，或 {id: ControllerState} 映射。
        verbose: 为 True 时将解码后的命令/回复打印到 stdout。
        physics_interval: 物理仿真步长（秒），默认 1 ms。
    """

    def __init__(
        self,
        controller_ids: typing.Union[
            typing.List[int],
            typing.Dict[int, ControllerState],
        ] = None,
        verbose: bool = False,
        physics_interval: float = 0.001,
    ):
        super().__init__()
        self.verbose = verbose
        self._physics_interval = physics_interval
        self._physics_task: typing.Optional[asyncio.Task] = None

        if controller_ids is None:
            controller_ids = [1]

        if isinstance(controller_ids, dict):
            self._controllers: typing.Dict[int, ControllerState] = controller_ids
        else:
            self._controllers = {cid: ControllerState(can_id=cid)
                                  for cid in controller_ids}

    # ------------------------------------------------------------------ #
    # TransportDevice interface
    # ------------------------------------------------------------------ #

    def empty_bus_tx_safe(self) -> bool:
        return True

    def close(self) -> None:
        if self._physics_task and not self._physics_task.done():
            self._physics_task.cancel()
            self._physics_task = None

    # ------------------------------------------------------------------ #
    # Physics loop
    # ------------------------------------------------------------------ #

    def _ensure_physics(self) -> None:
        if self._physics_task is None or self._physics_task.done():
            self._physics_task = asyncio.create_task(self._physics_loop())

    async def _physics_loop(self) -> None:
        while True:
            now = time.monotonic()
            for state in self._controllers.values():
                state.step(now)
            await asyncio.sleep(self._physics_interval)

    # ------------------------------------------------------------------ #
    # Frame processing
    # ------------------------------------------------------------------ #

    async def send_frame(self, frame: Frame) -> None:
        self._ensure_physics()

        dest_id   = frame.arbitration_id & 0x7f
        src_id    = (frame.arbitration_id >> 8) & 0x7f
        reply_req = bool(frame.arbitration_id & 0x8000)
        broadcast = (dest_id == 0x7f)

        # 解析帧内容
        parsed = parse_registers(frame.data)
        writes  = dict(parsed.command)
        queries = list(parsed.query)

        targets: typing.List[ControllerState]
        if broadcast:
            targets = list(self._controllers.values())
        else:
            s = self._controllers.get(dest_id)
            targets = [s] if s else []

        if not targets:
            return

        if self.verbose:
            self._log_tx(dest_id, src_id, writes, queries, broadcast)

        for state in targets:
            if writes:
                state.apply_writes(writes)

            if reply_req and queries:
                vals = state.register_values()
                data = _build_reply(queries, vals)
                if not data:
                    continue

                # 回包仲裁 ID：dest=src_id（回到主机）| source=controller_id（来自控制器）
                # transport._make_response_filter 检查：(arb>>8)&0x7f == controller_id
                #                                        arb&0x7f      == host_src_id
                reply_arb = src_id | (state.can_id << 8)
                reply_frame = Frame(
                    arbitration_id=reply_arb,
                    data=data,
                    is_extended_id=reply_arb > 0x7ff,
                    is_fd=True,
                    bitrate_switch=True,
                    channel=self,
                )
                if self.verbose:
                    self._log_rx(state, queries, vals)

                await self._handle_received_frame(reply_frame)

    async def receive_frame(self) -> Frame:
        return await super().receive_frame()

    async def transaction(
        self,
        requests: typing.List[TransportDevice.Request],
        **kwargs,
    ) -> None:
        def make_sub(request):
            fut: asyncio.Future = asyncio.Future()
            async def handler(frame, _r=request, _f=fut):
                if _f.done():
                    return
                _r.responses.append(frame)
                _f.set_result(None)
            return self._subscribe(request.frame_filter, handler), fut

        subs = [make_sub(r) for r in requests if r.frame_filter is not None]
        try:
            for req in requests:
                if req.frame is not None:
                    await self.send_frame(req.frame)
            if subs:
                await asyncio.gather(*[s[1] for s in subs])
        finally:
            for s in subs:
                s[0].cancel()

    # ------------------------------------------------------------------ #
    # Verbose logging
    # ------------------------------------------------------------------ #

    def _mode_name(self, m: int) -> str:
        try:
            return Mode(m).name
        except ValueError:
            return str(m)

    def _reg_name(self, r: int) -> str:
        try:
            return Register(r).name
        except ValueError:
            return f'0x{r:03x}'

    def _log_tx(self, dest, src, writes, queries, broadcast):
        tag = 'BCAST' if broadcast else f'→ ID{dest}'
        parts = []
        if writes:
            parts.append('WRITE {' + ', '.join(
                f'{self._reg_name(r)}={v:.4g}' if isinstance(v, float) else f'{self._reg_name(r)}={v}'
                for r, v in writes.items()
            ) + '}')
        if queries:
            parts.append('QUERY [' + ', '.join(self._reg_name(r) for r, _ in queries) + ']')
        print(f'[SIM TX src={src}] {tag}  ' + '  '.join(parts))

    def _log_rx(self, state, queries, vals):
        parts = []
        for r, _ in queries:
            if r in vals:
                v = vals[r]
                parts.append(
                    f'{self._reg_name(r)}={v:.4g}'
                    if isinstance(v, float) else f'{self._reg_name(r)}={v}'
                )
        print(f'[SIM RX ID{state.can_id}]  ' + ', '.join(parts))


# ---------------------------------------------------------------------------
# SimulatedTransport — 对外公开的便捷类
# ---------------------------------------------------------------------------

class SimulatedTransport(Transport):
    """SimulatedTransportDevice 的高级封装，直接用作 moteus.Transport。

    用法::
        transport = SimulatedTransport([1, 2, 3])
        c = moteus.Controller(id=1, transport=transport)
        await c.set_position(position=0.5, query=True)
    """

    def __init__(
        self,
        controller_ids=None,
        verbose: bool = False,
        physics_interval: float = 0.001,
    ):
        self._device = SimulatedTransportDevice(
            controller_ids=controller_ids,
            verbose=verbose,
            physics_interval=physics_interval,
        )
        super().__init__(self._device)

    @property
    def controllers(self) -> typing.Dict[int, ControllerState]:
        """直接访问各控制器状态，可在测试中读写。"""
        return self._device._controllers

    def close(self):
        self._device.close()
        super().close()


# ---------------------------------------------------------------------------
# patch_singleton — 上下文管理器，临时替换全局 Transport
# ---------------------------------------------------------------------------

@contextmanager
def patch_singleton(controller_ids=None, verbose: bool = False):
    """临时将 moteus 全局单例 Transport 替换为模拟器。

    用法::
        with patch_singleton([1, 2]):
            c = moteus.Controller(id=1)
            result = await c.set_position(position=0.5, query=True)
    """
    from . import transport_factory

    old = transport_factory.GLOBAL_TRANSPORT
    sim = SimulatedTransport(controller_ids, verbose=verbose)
    transport_factory.GLOBAL_TRANSPORT = sim
    try:
        yield sim
    finally:
        transport_factory.GLOBAL_TRANSPORT = old
        sim.close()


# ---------------------------------------------------------------------------
# 默认遥测寄存器集：模拟器主动发包时包含的寄存器
# 格式与 moteus 控制器的 REPLY 帧完全一致，主机端可直接用 parse_message() 解析
# ---------------------------------------------------------------------------

# (register, resolution) — 与 QueryResolution 默认值对齐
_DEFAULT_TELEMETRY: typing.List[typing.Tuple[int, int]] = [
    (int(Register.MODE),        INT8),
    (int(Register.POSITION),    F32),
    (int(Register.VELOCITY),    F32),
    (int(Register.TORQUE),      F32),
    (int(Register.VOLTAGE),     INT8),
    (int(Register.TEMPERATURE), INT8),
    (int(Register.FAULT),       INT8),
]


# ---------------------------------------------------------------------------
# CanEmulator — 连接真实 CAN 设备，主动发送模拟数据包
# ---------------------------------------------------------------------------

class CanEmulator:
    """在真实 CAN 总线上以固定频率发送模拟的 moteus 遥测包。

    工作原理
    --------
    1. 使用已打开的 TransportDevice 连接到真实 CAN 适配器。
    2. 物理仿真以 ``physics_interval`` 步进更新各控制器状态。
    3. 每隔 ``1/rate_hz`` 秒，为每个模拟控制器构造一帧 REPLY 格式的 CAN 帧
       并发送到总线——格式与真实 moteus 控制器的回包完全相同。
    4. 同时监听总线上发给我们 ID 的命令帧（写寄存器），应用到状态机，
       并对需要回复的查询帧发送对应 REPLY。

    发出帧的仲裁 ID 格式（与真实控制器一致）::

        arb_id = host_src_id | (controller_id << 8)
        # host_src_id 默认 0，controller_id 即 CAN ID

    主机侧用 ``parse_message(frame)`` 即可解析，``result.id`` = controller_id。

    Args:
        devices: TransportDevice 实例列表（来自 Transport.devices()）。
        controller_ids: 要模拟的 CAN ID 列表，或 {id: ControllerState} 映射。
        rate_hz: 主动发包频率（Hz），默认 100。
        telemetry: 每包包含的寄存器列表 [(register, resolution), ...]，
                   默认使用 _DEFAULT_TELEMETRY。
        host_src_id: 填入帧仲裁 ID 低 7 位的目标地址（主机源 ID），默认 0。
        verbose: 打印每帧发送详情。
        physics_interval: 物理仿真步长（秒）。
    """

    def __init__(
        self,
        devices: typing.List[TransportDevice],
        controller_ids: typing.Union[
            typing.List[int],
            typing.Dict[int, ControllerState],
        ] = None,
        rate_hz: float = 100.0,
        telemetry: typing.Optional[typing.List[typing.Tuple[int, int]]] = None,
        host_src_id: int = 0,
        verbose: bool = False,
        physics_interval: float = 0.001,
    ):
        self._devices = devices
        self._rate_hz = rate_hz
        self._telemetry = telemetry if telemetry is not None else _DEFAULT_TELEMETRY
        self._host_src_id = host_src_id
        self._verbose = verbose
        self._physics_interval = physics_interval

        if controller_ids is None:
            controller_ids = [1]
        if isinstance(controller_ids, dict):
            self._controllers: typing.Dict[int, ControllerState] = controller_ids
        else:
            self._controllers = {
                cid: ControllerState(can_id=cid) for cid in controller_ids
            }

    @property
    def controllers(self) -> typing.Dict[int, ControllerState]:
        return self._controllers

    async def run(self) -> None:
        """启动物理仿真、主动发包和命令接收，阻塞直到被取消。"""
        tasks = [
            asyncio.create_task(self._physics_loop()),
            asyncio.create_task(self._telemetry_loop()),
        ]
        for dev in self._devices:
            tasks.append(asyncio.create_task(self._recv_loop(dev)))
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            pass
        finally:
            for t in tasks:
                t.cancel()

    def close(self) -> None:
        for dev in self._devices:
            dev.close()

    # ------------------------------------------------------------------ #
    # 物理仿真
    # ------------------------------------------------------------------ #

    async def _physics_loop(self) -> None:
        while True:
            now = time.monotonic()
            for s in self._controllers.values():
                s.step(now)
            await asyncio.sleep(self._physics_interval)

    # ------------------------------------------------------------------ #
    # 主动发包：按 rate_hz 频率向总线发送遥测帧
    # ------------------------------------------------------------------ #

    async def _telemetry_loop(self) -> None:
        interval = 1.0 / self._rate_hz
        while True:
            await asyncio.sleep(interval)
            for state in self._controllers.values():
                vals = state.register_values()
                data = _build_reply(self._telemetry, vals)
                if not data:
                    continue

                # arb_id = host_src_id | (controller_id << 8)
                # 与真实控制器回包格式相同，parse_message() 可直接解析
                arb_id = self._host_src_id | (state.can_id << 8)
                frame = Frame(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=arb_id > 0x7ff,
                    is_fd=True,
                    bitrate_switch=True,
                )
                for dev in self._devices:
                    await dev.send_frame(frame)

                if self._verbose:
                    self._log_send(state, vals)

    # ------------------------------------------------------------------ #
    # 命令接收：处理主机写寄存器 / 有回复要求的查询
    # ------------------------------------------------------------------ #

    async def _recv_loop(self, device: TransportDevice) -> None:
        while True:
            frame = await device.receive_frame()
            await self._handle_command(frame, device)

    async def _handle_command(
        self, frame: Frame, device: TransportDevice
    ) -> None:
        dest_id   = frame.arbitration_id & 0x7f
        src_id    = (frame.arbitration_id >> 8) & 0x7f
        reply_req = bool(frame.arbitration_id & 0x8000)
        broadcast = (dest_id == 0x7f)

        if broadcast:
            targets = list(self._controllers.values())
        else:
            s = self._controllers.get(dest_id)
            if s is None:
                return
            targets = [s]

        parsed = parse_registers(frame.data)

        for state in targets:
            # 写命令（mode、position 等）立即应用
            if parsed.command:
                state.apply_writes(parsed.command)
                if self._verbose:
                    self._log_cmd(state, parsed.command)

            # 显式查询：立即回包（不等下一个遥测周期）
            if reply_req and parsed.query:
                vals = state.register_values()
                data = _build_reply(parsed.query, vals)
                if data:
                    reply_arb = src_id | (state.can_id << 8)
                    await device.send_frame(Frame(
                        arbitration_id=reply_arb,
                        data=data,
                        is_extended_id=reply_arb > 0x7ff,
                        is_fd=True,
                        bitrate_switch=True,
                    ))

    # ------------------------------------------------------------------ #
    # 日志
    # ------------------------------------------------------------------ #

    def _reg_name(self, r: int) -> str:
        try:
            return Register(r).name
        except ValueError:
            return f'0x{r:03x}'

    def _log_send(self, state: ControllerState, vals: dict) -> None:
        parts = []
        for reg, _ in self._telemetry:
            v = vals.get(reg)
            if v is None:
                continue
            name = self._reg_name(reg)
            parts.append(f'{name}={v:.4g}' if isinstance(v, float) else f'{name}={v}')
        try:
            mode_name = Mode(state.mode).name
        except ValueError:
            mode_name = str(state.mode)
        print(f'[EMU TX ID{state.can_id}] {mode_name}  ' + '  '.join(parts))

    def _log_cmd(self, state: ControllerState, writes: dict) -> None:
        parts = [
            f'{self._reg_name(r)}={v:.4g}' if isinstance(v, float) else
            f'{self._reg_name(r)}={v}'
            for r, v in writes.items()
        ]
        print(f'[EMU RX ID{state.can_id}] WRITE ' + ', '.join(parts))


# ---------------------------------------------------------------------------
# 命令行演示
# ---------------------------------------------------------------------------

async def _demo(ids: typing.List[int], verbose: bool) -> None:
    import moteus

    transport = SimulatedTransport(ids, verbose=verbose)
    controllers = [moteus.Controller(id=i, transport=transport) for i in ids]

    print(f'\n=== moteus 模拟器演示  控制器 ID: {ids} ===\n')

    # 发送 stop 确认连接
    for c in controllers:
        r = await c.set_stop(query=True)
        print(f'ID{c.id} STOP  → mode={r.values.get(moteus.Register.MODE)}  '
              f'pos={r.values.get(moteus.Register.POSITION):.4f}')

    await asyncio.sleep(0.05)

    # 下发位置命令
    targets = [i * 0.5 for i in range(len(ids))]
    print(f'\n下发位置命令: {targets}')
    for c, tgt in zip(controllers, targets):
        await c.set_position(
            position=tgt,
            velocity_limit=2.0,
            maximum_torque=3.0,
            query=False,
        )

    # 构造包含 TRAJECTORY_COMPLETE 的查询分辨率
    qr = moteus.QueryResolution()
    qr.trajectory_complete = moteus.INT8

    # 等待并轮询
    print('\n等待轨迹完成...')
    for _ in range(60):
        await asyncio.sleep(0.1)
        results = await transport.cycle([
            c.make_position(position=tgt, velocity_limit=2.0,
                            maximum_torque=3.0,
                            query=True, query_override=qr)
            for c, tgt in zip(controllers, targets)
        ])
        done = all(
            r.values.get(moteus.Register.TRAJECTORY_COMPLETE, 0)
            for r in results
        )
        status = '  '.join(
            f'ID{r.id} pos={r.values.get(moteus.Register.POSITION, math.nan):.4f}'
            f' vel={r.values.get(moteus.Register.VELOCITY, math.nan):.3f}'
            f' done={r.values.get(moteus.Register.TRAJECTORY_COMPLETE, "?")}'
            for r in results
        )
        print(f'  {status}')
        if done:
            print('\n✓ 所有轨迹完成')
            break
    else:
        print('\n⚠ 超时')

    # 打印最终状态
    print('\n最终状态:')
    for c in controllers:
        r = await c.query()
        if r:
            print(
                f'  ID{c.id}'
                f'  mode={Mode(r.values.get(moteus.Register.MODE, 0)).name}'
                f'  pos={r.values.get(moteus.Register.POSITION, math.nan):.4f} rev'
                f'  vel={r.values.get(moteus.Register.VELOCITY, math.nan):.3f} rev/s'
                f'  torque={r.values.get(moteus.Register.TORQUE, math.nan):.2f} Nm'
                f'  vbus={r.values.get(moteus.Register.VOLTAGE, math.nan):.1f} V'
                f'  temp={r.values.get(moteus.Register.TEMPERATURE, math.nan):.1f} °C'
            )

    # 全部停止
    for c in controllers:
        await c.set_stop()
    print('\n全部控制器已停止.')
    transport.close()


def main() -> None:
    from .transport_factory import (
        get_transport_factories,
        get_singleton_transport,
        make_transport_args,
    )

    parser = argparse.ArgumentParser(
        description=(
            'moteus 控制器模拟器\n'
            '\n'
            '不带 transport 参数：纯软件 mock，无需硬件，运行内置演示。\n'
            '带 transport 参数（--can-iface / --can-chan / --fdcanusb 等）：\n'
            '  打开真实 CAN 设备，在总线上监听并用软件状态机回包。'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--ids', type=int, nargs='+', default=[1],
        metavar='ID',
        help='要模拟的控制器 CAN ID（默认: 1）',
    )
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='打印每帧的寄存器读写细节',
    )
    parser.add_argument(
        '--rate', type=float, default=100.0,
        metavar='HZ',
        help='真实 CAN 模式下主动发包频率（默认: 100 Hz）',
    )
    make_transport_args(parser)
    args = parser.parse_args()

    # 判断是否有显式的 transport 参数 → 使用真实硬件
    use_real = any(
        f.is_args_set(args) for f in get_transport_factories()
    ) or getattr(args, 'force_transport', None)

    if use_real:
        asyncio.run(_run_emulator(args, args.ids, args.verbose))
    else:
        asyncio.run(_demo(args.ids, args.verbose))


async def _run_emulator(args, ids: typing.List[int], verbose: bool) -> None:
    """在真实 CAN 总线上运行硬件在环仿真（主动发包模式）。"""
    from .transport_factory import get_singleton_transport

    print('正在打开 CAN 设备...')
    real_transport = get_singleton_transport(args)
    devices = real_transport.devices()
    rate = getattr(args, 'rate', 100.0)

    emulator = CanEmulator(devices, ids, rate_hz=rate, verbose=verbose)

    print(f'=== moteus CAN 仿真器（硬件在环）===')
    print(f'控制器 ID : {ids}')
    print(f'CAN 设备  : {devices}')
    print(f'发包频率  : {rate} Hz')
    print(f'寄存器    : {[Register(r).name for r, _ in _DEFAULT_TELEMETRY]}')
    print('按 Ctrl+C 退出\n')

    try:
        await emulator.run()
    except KeyboardInterrupt:
        pass
    finally:
        emulator.close()
        print('\n仿真器已停止.')


if __name__ == '__main__':
    main()
