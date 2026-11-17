# Code and model provenance

## Mechanism model

The mechanism defaults, coaxial 3-RRR geometry, analytic inverse-kinematics
branch equations, hinge-point convention, and Jacobian conditioning index were
ported from the adjacent user project:

- `parallel_delta/coaxial_delta/params.py`
- `parallel_delta/coaxial_delta/geometry.py`
- `parallel_delta/coaxial_delta/spm.py`

The implementation is intentionally self-contained: this project does not
import that directory at runtime. The source model's `theta/phi/psi` are
spherical-cap coordinates, not standard RPY. `pose.py` adds explicit active
ZYX RPY conversion. `kinematics.py` adds continuous unwrapping, path-continued
Newton forward kinematics, and motor/transmission mapping.

## Motor control

The shared-transport asyncio pattern, `transport.cycle()` batching, query
resolution, current-position target capture, watchdog position commands, and
best-effort shutdown stop were derived from:

- `scripts/moteus_hold/moteus_hold_daemon.py`
- `moteus/moteus/moteus.py`
- `moteus/moteus/simulator.py`

Unlike `moteus_hold`, this coordinated mechanism starts uncalibrated in real
hardware mode, stops the whole group on a critical feedback fault, and never
uses `rezero` as a substitute for recovering lost encoder turns.

## Frontend

The browser UI and orthographic three-dimensional Canvas projection are
implemented with local HTML/CSS/JavaScript only. No external JavaScript code
or CDN asset is required.
