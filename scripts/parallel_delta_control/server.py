"""Local FastAPI REST/WebSocket interface."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from controller import ControlError, MechanismController
from kinematics import KinematicsError
from params import AppConfig, ConfigError, save_config


STATIC_DIR = Path(__file__).resolve().parent / "static"
MAX_STATE_PUBLISH_HZ = 20.0


def telemetry_period_s(rate_hz: float) -> float:
    """Never let browser telemetry starve the hardware control loop."""
    return 1.0 / min(float(rate_hz), MAX_STATE_PUBLISH_HZ)


def create_app(controller: MechanismController, config_path: str | Path) -> FastAPI:
    app = FastAPI(title="三自由度并联机构控制", docs_url="/api/docs",
                  redoc_url=None, openapi_url="/api/openapi.json")

    @app.middleware("http")
    async def no_cache_control_assets(request: Request, call_next):
        response = await call_next(request)
        if request.url.path == "/" or request.url.path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-cache, must-revalidate"
        return response

    @app.exception_handler(ControlError)
    async def control_error(_request: Request, exc: ControlError):
        return JSONResponse(status_code=409, content={"detail": str(exc)})

    @app.exception_handler(KinematicsError)
    async def kinematics_error(_request: Request, exc: KinematicsError):
        return JSONResponse(status_code=400, content={"detail": str(exc)})

    @app.exception_handler(ConfigError)
    async def config_error(_request: Request, exc: ConfigError):
        return JSONResponse(status_code=400, content={"detail": str(exc)})

    @app.get("/api/state")
    async def state() -> dict[str, Any]:
        return controller.snapshot()

    @app.post("/api/calibrate")
    async def calibrate() -> dict[str, Any]:
        controller.calibrate()
        return controller.snapshot()

    @app.post("/api/calibrate/extreme")
    async def extreme_calibrate() -> dict[str, Any]:
        controller.start_extreme_calibration()
        return controller.snapshot()

    @app.post("/api/hold")
    async def hold() -> dict[str, Any]:
        controller.hold()
        return controller.snapshot()

    @app.post("/api/stop")
    async def stop() -> dict[str, Any]:
        controller.stop()
        return controller.snapshot()

    @app.post("/api/clear")
    async def clear() -> dict[str, Any]:
        controller.clear()
        return controller.snapshot()

    @app.post("/api/command/motors")
    async def command_motors(request: Request) -> dict[str, Any]:
        body = await request.json()
        controller.command_motors(
            body["motor_deg"], duration_s=body.get("duration_s"))
        return controller.snapshot()

    @app.post("/api/command/rpy")
    async def command_rpy(request: Request) -> dict[str, Any]:
        body = await request.json()
        await controller.command_rpy_async(
            body["rpy_deg"], duration_s=body.get("duration_s"))
        return controller.snapshot()

    @app.post("/api/command/velocity/motors")
    async def command_motor_velocity(request: Request) -> dict[str, Any]:
        body = await request.json()
        controller.set_motor_velocity(body["motor_rev_s"])
        return controller.snapshot()

    @app.post("/api/command/velocity/rpy")
    async def command_rpy_velocity(request: Request) -> dict[str, Any]:
        body = await request.json()
        controller.set_rpy_velocity(body["rpy_deg_s"])
        return controller.snapshot()

    @app.post("/api/config")
    async def update_config(request: Request, save: bool = False) -> dict[str, Any]:
        if controller.enabled:
            raise ControlError("修改配置前必须先 stop")
        body = await request.json()
        config = AppConfig.from_dict(body)
        if save:
            save_config(config, config_path)
        await controller.update_config(config)
        return controller.snapshot()

    @app.post("/api/config/reset")
    async def reset_config(save: bool = False) -> dict[str, Any]:
        if controller.enabled:
            raise ControlError("恢复默认配置前必须先 stop")
        config = AppConfig()
        if save:
            save_config(config, config_path)
        await controller.update_config(config)
        return controller.snapshot()

    @app.websocket("/api/ws")
    async def telemetry(websocket: WebSocket) -> None:
        await websocket.accept()
        period = telemetry_period_s(controller.config.safety.state_publish_hz)
        try:
            while True:
                await websocket.send_json(controller.snapshot())
                period = telemetry_period_s(controller.config.safety.state_publish_hz)
                await asyncio.sleep(period)
        except (WebSocketDisconnect, asyncio.CancelledError):
            return
        except RuntimeError:
            # Socket closed between send and the next telemetry cycle.
            return

    @app.get("/", include_in_schema=False)
    async def index() -> FileResponse:
        return FileResponse(STATIC_DIR / "index.html")

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
    return app
