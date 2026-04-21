# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is the Python bindings package (`moteus` v0.3.101) for the mjbots moteus brushless controller. It provides an async Python API for communicating with moteus controllers over CAN-FD.

## Setup

```bash
pip3 install -e .
# or with dev dependencies
pip3 install pyserial python-can pyelftools scipy importlib_metadata packaging
```

## Architecture

### Layer model

```
User code
    ↓
Controller (moteus/moteus.py)     ← high-level per-servo API
    ↓
Transport (moteus/transport.py)   ← multi-device dispatch, routing, discovery
    ↓
TransportDevice (moteus/transport_device.py)  ← ABC for hardware devices
    ↓
FdcanusbDevice / PythonCanDevice  ← concrete hardware backends
```

### Key classes

- **`Controller`** (`moteus/moteus.py`) — The primary user-facing class. Each instance represents one physical servo. All commands are built with `make_*()` methods and executed with `execute()` or `set_*()` methods. The `transport=None` default triggers singleton auto-discovery.

- **`Transport`** (`moteus/transport.py`) — Dispatches CAN-FD frames to one or more `TransportDevice` instances. Handles multi-bus routing: builds a `_routing_table` by broadcasting UUID queries on first use. The `cycle()` method is the primary bus-efficient API for commanding multiple controllers in one round-trip.

- **`TransportDevice`** (`moteus/transport_device.py`) — ABC that defines `send_frame()`, `receive_frame()`, `transaction()`. Maintains a subscription/waiter queue for async frame delivery.

- **`FdcanusbDevice`** (`moteus/fdcanusb_device.py`) — Serial-port transport for the mjbots fdcanusb USB-CAN adapter.

- **`PythonCanDevice`** (`moteus/pythoncan_device.py`) — Transport backed by python-can (socketcan, PCAN, kvaser, vector). Handles fdcanusb deduplication via USB serial numbers to avoid double-counting the same physical device.

- **`CandleDevice`** (`moteus/candle_device.py`) — `TransportDevice` for candle CAN-FD USB adapters (Windows/Mac). Overrides `_frame_to_can_message` to honour `bitrate_switch` from the frame (necessary for CAN-FD BRS). `enumerate_devices()` calls `can.detect_available_configs("candle")`, parses `"serial:index"` channel strings, and creates one instance per channel.

- **`Candle`** (`moteus/candle.py`) — Convenience `TransportWrapper` around `CandleDevice`, mirroring the `Fdcanusb`/`PythonCan` pattern.

- **`transport_factory.py`** — Singleton transport construction via `get_singleton_transport()`. Tries `FdcanusbFactory` (priority 10), `PythonCanFactory` (priority 11), `CandleFactory` (priority 12) then any `moteus.transports2` entry points. `CandleFactory` only auto-enumerates devices on non-Linux; on Linux candle hardware appears as socketcan and is handled by `PythonCanFactory`. Additional transports (e.g., pi3hat) register via setuptools entry points in the `moteus.transports2` group.

### Resolution system

Commands and queries use typed resolution constants (`mp.INT8`, `mp.INT16`, `mp.INT32`, `mp.F32`, `mp.IGNORE`) defined in `moteus/multiplex.py`. `QueryResolution` and `PositionResolution` classes in `moteus/moteus.py` control the wire encoding per field. `mp.WriteCombiner` consolidates adjacent non-`IGNORE` fields into compact register-write opcodes.

### Wire protocol

`moteus/protocol.py` — CAN arbitration ID encoding: `dest_id | (reply_required << 15) | (source_id << 8) | (can_prefix << 16)`. Register read/write opcodes follow the moteus multiplex protocol documented at `https://github.com/mjbots/moteus/blob/main/docs/reference.md`.

### DeviceAddress and UUID routing

Controllers can be addressed by integer CAN ID or by `DeviceAddress` (containing optional UUID bytes). `Transport.discover()` broadcasts a UUID query and builds a map. UUID-based addressing is used when multiple controllers share the same CAN ID across different buses.

### CLI tool

`moteus_tool` entry point → `moteus/moteus_tool.py`. Uses `make_transport_args()` + `get_singleton_transport()` for CLI-driven transport selection.

## Common patterns

Single controller (transport auto-discovered):
```python
import asyncio, moteus

async def main():
    c = moteus.Controller(id=1)
    await c.set_stop()
    result = await c.set_position(position=0.5, query=True)

asyncio.run(main())
```

Bus-optimized multi-controller (one CAN cycle):
```python
transport = moteus.Fdcanusb()
c1, c2 = moteus.Controller(id=1), moteus.Controller(id=2)
results = await transport.cycle([
    c1.make_position(position=0.0, query=True),
    c2.make_position(position=0.0, query=True),
])
```

## Type checking

```bash
mypy moteus/
```

Config is in `pyproject.toml` (`[tool.mypy]`). Currently lenient — `disallow_untyped_defs` and strict checking are disabled to allow gradual adoption. Test files are excluded from mypy checks.
