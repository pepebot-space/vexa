#!/usr/bin/env python3
import asyncio
import time
from typing import Any, Dict, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.responses import JSONResponse, FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from bleak import BleakClient, BleakScanner

# Import the new local database and agent core
import database
import agent_core

SVC_NUS = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHR_WRITE_NUS = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
CHR_NOTIFY_NUS = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

SVC_FFE = "0000FFE0-0000-1000-8000-00805F9B34FB"
CHR_FFE = "0000FFE1-0000-1000-8000-00805F9B34FB"

NAME_PREFIXES = ("M6", "Mini", "A6")

REG: Dict[str, int] = {
    "limit": 0x72,
    "lock": 0x70,
    "lock2": 0x71,
    "normal_speed": 0x73,
    "train_speed": 0x74,
    "max_speed": 0x7D,
    "turn_scale": 0xA1,
    "riding_scale": 0xA2,
    "power_balance": 0xA3,
    "riding_balance": 0xFC,
    "light_flags": 0xD3,
}

robot_state: Dict[str, Optional[str]] = {
    "address": None,
    "name": None,
    "connected": "false",
    "last_command": None,
    "last_response": None,
}

client_instance: Optional[BleakClient] = None
robot_lock = asyncio.Lock()


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def maybe_xor(data: bytes, key: str) -> bytes:
    if key == "none":
        return data
    b = 0x55 if key == "55" else 0xD8
    return bytes((x ^ b) for x in data)


def parse_one_frame(data: bytes) -> Optional[tuple[int, int, int, bytes]]:
    if len(data) < 8 or data[0] != 0x55 or data[1] != 0xAA:
        return None
    total = data[2] + 6
    if total > len(data):
        return None
    frame = data[:total]
    checksum = ((frame[-1] << 8) | frame[-2]) & 0xFFFF
    calc = (~sum(frame[2:-2])) & 0xFFFF
    if checksum != calc:
        return None
    payload_len = frame[2] - 2
    payload = frame[6 : 6 + payload_len]
    return frame[3], frame[4], frame[5], payload


def decode_frame_any(data: bytes) -> Optional[tuple[int, int, int, bytes, str]]:
    for mode, key in (("none", None), ("55", 0x55), ("d8", 0xD8)):
        probe = data if key is None else bytes((b ^ key) for b in data)
        parsed = parse_one_frame(probe)
        if parsed is not None:
            cmd, typ, addr, payload = parsed
            return cmd, typ, addr, payload, mode
    return None


def estimate_percent(raw: int, min_dv: int, max_dv: int) -> int:
    if max_dv <= min_dv:
        return 0
    pct = int(round((raw - min_dv) * 100.0 / (max_dv - min_dv)))
    return max(0, min(100, pct))


def extract_battery1_level(payload: bytes, raw_deci_volt: int) -> Optional[int]:
    if len(payload) < 4:
        return None

    raw2 = raw_deci_volt.to_bytes(2, "little", signed=False)
    candidates: list[int] = []

    for i in range(0, len(payload) - 1):
        if payload[i : i + 2] != raw2:
            continue
        if i >= 2:
            lv1 = payload[i - 2]
            lv2 = payload[i - 1]
            if 0 <= lv1 <= 100 and 0 <= lv2 <= 100:
                candidates.append(lv1)
        if i >= 1:
            lv = payload[i - 1]
            if 0 <= lv <= 100:
                candidates.append(lv)

    if not candidates:
        return None

    counts: Dict[int, int] = {}
    for v in candidates:
        counts[v] = counts.get(v, 0) + 1
    return max(counts, key=lambda x: counts[x])


def build_frame(cmd: int, typ: int, addr: int, payload: bytes) -> bytes:
    if not (0 <= cmd <= 0xFF and 0 <= typ <= 0xFF and 0 <= addr <= 0xFF):
        raise ValueError("cmd/type/addr must be 0..255")
    if len(payload) > 255:
        raise ValueError("payload too long")

    frame = bytearray()
    frame.extend((0x55, 0xAA))
    frame.append((len(payload) + 2) & 0xFF)
    frame.append(cmd & 0xFF)
    frame.append(typ & 0xFF)
    frame.append(addr & 0xFF)
    frame.extend(payload)

    checksum = (~sum(frame[2:])) & 0xFFFF
    frame.append(checksum & 0xFF)
    frame.append((checksum >> 8) & 0xFF)
    return bytes(frame)


def write_cmd2(addr: int, value: int) -> bytes:
    payload = int(value & 0xFFFF).to_bytes(2, "little", signed=False)
    return build_frame(cmd=0x0A, typ=0x03, addr=addr, payload=payload)


def write_cmd(addr: int, value: int) -> bytes:
    payload = int(value & 0xFFFF).to_bytes(2, "little", signed=False)
    return build_frame(cmd=0x06, typ=0x03, addr=addr, payload=payload)


def write_array_cmd2(addr: int, payload: bytes) -> bytes:
    return build_frame(cmd=0x0A, typ=0x03, addr=addr, payload=payload)


def read_cmd2(addr: int, sub: int) -> bytes:
    payload = bytes([sub & 0xFF])
    return build_frame(cmd=0x0A, typ=0x01, addr=addr, payload=payload)


def remote_channel_addr(channel: int) -> int:
    if not (1 <= channel <= 7):
        raise ValueError("channel must be 1..7")
    return 0xC8 + (channel * 2)


def encode_drive_value(axis: int) -> int:
    if axis < -128 or axis > 127:
        raise ValueError("axis must be -128..127")
    raw = ((axis << 8) + 0x00F0) & 0xFFFF
    if raw >= 0x8000:
        raw -= 0x10000
    return raw


def choose_profile(services) -> str:
    uuids = {s.uuid.upper() for s in services}
    if SVC_NUS in uuids:
        return "nus"
    if SVC_FFE in uuids:
        return "ffe"
    return "unknown"


async def find_target(name: Optional[str], address: Optional[str], timeout: float = 8.0):
    if address:
        d = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return d

    devices = await BleakScanner.discover(timeout=timeout)
    if name:
        name_l = name.lower()
        for d in devices:
            if d.name and name_l in d.name.lower():
                return d
        return None

    for d in devices:
        if d.name and d.name.startswith(NAME_PREFIXES):
            return d
    return None


async def get_client() -> BleakClient:
    global client_instance
    if client_instance and client_instance.is_connected:
        return client_instance
    raise HTTPException(status_code=503, detail="Robot not connected")


async def send_frame(frame: bytes, xor_key: str = "55", delay: float = 0.08) -> str:
    client = await get_client()
    services = client.services
    profile = choose_profile(services)

    if profile == "nus":
        write_uuid = CHR_WRITE_NUS
    elif profile == "ffe":
        write_uuid = CHR_FFE
    else:
        raise HTTPException(status_code=500, detail="Unknown BLE profile")

    wire = maybe_xor(frame, xor_key)
    await client.write_gatt_char(write_uuid, wire, response=False)
    await asyncio.sleep(delay)
    return hexdump(wire)


async def send_frames(frames: list, xor_key: str = "55", delay: float = 0.08) -> list:
    results = []
    for f in frames:
        result = await send_frame(f, xor_key, delay)
        results.append(result)
    return results


class ConnectRequest(BaseModel):
    address: Optional[str] = None
    name: Optional[str] = None
    scan_timeout: float = 8.0
    xor_key: str = "55"


class MoveRequest(BaseModel):
    action: str
    power: int = 9000
    duration: float = 1.5
    drive_interval: float = 0.04
    poll_interval: float = 0.20
    xor_key: str = "55"


class JoystickRequest(BaseModel):
    x: int
    y: int
    duration: float = 2.0
    drive_interval: float = 0.04
    poll_interval: float = 0.20
    xor_key: str = "55"
    unlock: bool = True
    remote_enable: bool = True
    remote_disable: bool = False


class WriteRequest(BaseModel):
    reg: str
    value: int
    protocol: str = "cmd2"
    xor_key: str = "55"


class LightRequest(BaseModel):
    headlight: bool = False
    brakelight: bool = False
    lock_shutdown: bool = False
    lock_warn: bool = False
    back_alarm: bool = False
    carlight: bool = False
    xor_key: str = "55"


class RawRequest(BaseModel):
    hex: str
    xor_key: str = "55"


class DriveRequest(BaseModel):
    channel: int = 1
    axis: Optional[int] = None
    percent: int = 0
    repeat: int = 1
    stop_after: bool = False
    xor_key: str = "55"


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    global client_instance
    if client_instance and client_instance.is_connected:
        await asyncio.to_thread(database.add_log, "INFO", "System", "Server shutdown, disconnecting.")
        await client_instance.disconnect()
    client_instance = None
    robot_state["connected"] = "false"
    agent_core.stop_agent()


app = FastAPI(
    title="MiniRobot Control Panel API",
    description="REST API for controlling MiniRobot balancing car with SQLite backend for NEXA UI",
    version="2.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Wait to mount static until after roots so we can override "/" easily
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/")
async def root():
    return FileResponse(path="static/index.html", media_type="text/html")


async def video_generator():
    while True:
        frame = agent_core.get_latest_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # Yield an empty frame or just sleep
            pass
        await asyncio.sleep(0.05)


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(video_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/api/agent/start")
async def api_start_agent():
    agent_core.start_agent()
    await asyncio.to_thread(database.add_log, "INFO", "Agent", "Voice/Video Agent starting...")
    return JSONResponse(content={"status": "starting"})


@app.post("/api/agent/stop")
async def api_stop_agent():
    agent_core.stop_agent()
    await asyncio.to_thread(database.add_log, "INFO", "Agent", "Voice/Video Agent stopped.")
    return JSONResponse(content={"status": "stopped"})


@app.get("/api/agent/status")
async def api_agent_status():
    return JSONResponse(content={"status": agent_core.get_status()})


@app.get("/api/status")
async def get_status():
    return JSONResponse(content=robot_state)

@app.get("/api/logs")
async def get_logs(limit: int = 50):
    logs = await asyncio.to_thread(database.get_logs, limit)
    return JSONResponse(content={"logs": logs})

@app.get("/api/battery_history")
async def get_battery_history(limit: int = 24):
    history = await asyncio.to_thread(database.get_battery_history, limit)
    return JSONResponse(content={"history": history})

@app.get("/api/scan")
async def scan_devices(timeout: float = 8.0):
    await asyncio.to_thread(database.add_log, "INFO", "BLE", "Scanning for devices...")
    devices = await BleakScanner.discover(timeout=timeout)
    result = []
    for d in devices:
        if d.name and d.name.startswith(NAME_PREFIXES):
            result.append({"name": d.name, "address": d.address})
    await asyncio.to_thread(database.add_log, "INFO", "BLE", f"Scan complete. Found {len(result)} devices.")
    return JSONResponse(content={"devices": result})


@app.post("/api/connect")
async def connect(request: ConnectRequest):
    global client_instance, robot_state

    async with robot_lock:
        if client_instance and client_instance.is_connected:
            return JSONResponse(
                content={
                    "status": "already_connected",
                    "address": robot_state["address"],
                }
            )

        await asyncio.to_thread(database.add_log, "INFO", "BLE", f"Attempting connection to {request.address or request.name or 'any'}")
        device = await find_target(request.name, request.address, request.scan_timeout)
        if not device:
            await asyncio.to_thread(database.add_log, "ERROR", "BLE", "Device not found.")
            raise HTTPException(status_code=404, detail="Device not found")

        try:
            client = BleakClient(device, timeout=20.0)
            await client.connect()
            client_instance = client

            robot_state["connected"] = "true"
            robot_state["address"] = device.address
            robot_state["name"] = device.name
            
            await asyncio.to_thread(database.add_log, "SUCCESS", "BLE", f"Connected to {device.name} at {device.address}")

            return JSONResponse(
                content={
                    "status": "connected",
                    "address": device.address,
                    "name": device.name,
                }
            )
        except Exception as e:
            await asyncio.to_thread(database.add_log, "ERROR", "BLE", f"Connection failed: {str(e)}")
            raise e

@app.post("/api/disconnect")
async def disconnect():
    global client_instance, robot_state

    async with robot_lock:
        if client_instance and client_instance.is_connected:
            await client_instance.disconnect()
            client_instance = None
            await asyncio.to_thread(database.add_log, "INFO", "BLE", "Disconnected from device.")

        robot_state["connected"] = "false"
        robot_state["address"] = None
        robot_state["name"] = None

        return JSONResponse(content={"status": "disconnected"})


@app.post("/api/move")
async def move(request: MoveRequest, background_tasks: BackgroundTasks):
    if request.action not in ("forward", "backward", "left", "right", "stop"):
        raise HTTPException(
            status_code=400,
            detail="action must be forward/backward/left/right/stop",
        )

    amp = abs(request.power)
    if amp > 32767:
        raise HTTPException(status_code=400, detail="power must be 0..32767")

    mapping = {
        "left": (0, amp),
        "right": (0, -amp),
        "forward": (amp, 0),
        "backward": (-amp, 0),
        "stop": (0, 0),
    }
    x, y = mapping[request.action]

    async def run_movement():
        async with robot_lock:
            try:
                client = await get_client()
                services = client.services
                profile = choose_profile(services)

                if profile == "nus":
                    write_uuid = CHR_WRITE_NUS
                elif profile == "ffe":
                    write_uuid = CHR_FFE
                else:
                    return

                async def tx(frame: bytes):
                    wire = maybe_xor(frame, request.xor_key)
                    await client.write_gatt_char(write_uuid, wire, response=False)
                    await asyncio.sleep(request.drive_interval)

                await tx(write_cmd(0x70, 0))
                await tx(write_cmd2(0x71, 0))
                await tx(write_cmd2(0x7A, 1))
                await tx(read_cmd2(0x1F, 0x02))
                await tx(read_cmd2(0xBB, 0x02))
                await tx(read_cmd2(0x7D, 0x02))

                payload = x.to_bytes(2, "little", signed=True) + y.to_bytes(
                    2, "little", signed=True
                )

                t_end = time.monotonic() + max(0.2, request.duration)
                next_drive = time.monotonic()
                next_poll = time.monotonic()
                while time.monotonic() < t_end:
                    now = time.monotonic()
                    if now >= next_drive:
                        await tx(write_array_cmd2(0x7B, payload))
                        next_drive += request.drive_interval
                    if now >= next_poll:
                        await tx(read_cmd2(0x1F, 0x10))
                        next_poll += request.poll_interval
                    await asyncio.sleep(0.005)

                zero = (0).to_bytes(2, "little", signed=True)
                await tx(write_array_cmd2(0x7B, zero + zero))

                robot_state["last_command"] = f"move:{request.action}"
                if request.action != 'stop':
                    await asyncio.to_thread(database.add_log, "CMD", "Drive", f"Move {request.action} at power {request.power}")

            except Exception as e:
                robot_state["last_response"] = f"error: {str(e)}"
                await asyncio.to_thread(database.add_log, "ERROR", "Drive", f"Move failed: {str(e)}")

    background_tasks.add_task(run_movement)

    return JSONResponse(
        content={
            "status": "started",
            "action": request.action,
            "power": request.power,
            "duration": request.duration,
        }
    )


@app.post("/api/joystick")
async def joystick(request: JoystickRequest, background_tasks: BackgroundTasks):
    if request.x < -32768 or request.x > 32767:
        raise HTTPException(status_code=400, detail="x must be -32768..32767")
    if request.y < -32768 or request.y > 32767:
        raise HTTPException(status_code=400, detail="y must be -32768..32767")

    async def run_joystick():
        async with robot_lock:
            try:
                client = await get_client()
                services = client.services
                profile = choose_profile(services)

                if profile == "nus":
                    write_uuid = CHR_WRITE_NUS
                elif profile == "ffe":
                    write_uuid = CHR_FFE
                else:
                    return

                async def tx(frame: bytes):
                    wire = maybe_xor(frame, request.xor_key)
                    await client.write_gatt_char(write_uuid, wire, response=False)
                    await asyncio.sleep(0.04)

                if request.unlock:
                    await tx(write_cmd(0x70, 0))
                    await tx(write_cmd2(0x71, 0))

                if request.remote_enable:
                    await tx(write_cmd2(0x7A, 1))

                await tx(read_cmd2(0x1F, 0x02))
                await tx(read_cmd2(0xBB, 0x02))
                await tx(read_cmd2(0x7D, 0x02))

                payload = request.x.to_bytes(2, "little", signed=True) + request.y.to_bytes(
                    2, "little", signed=True
                )

                t_end = time.monotonic() + max(0.2, request.duration)
                next_drive = time.monotonic()
                next_poll = time.monotonic()
                while time.monotonic() < t_end:
                    now = time.monotonic()
                    if now >= next_drive:
                        await tx(write_array_cmd2(0x7B, payload))
                        next_drive += request.drive_interval
                    if now >= next_poll:
                        await tx(read_cmd2(0x1F, 0x10))
                        next_poll += request.poll_interval
                    await asyncio.sleep(0.005)

                zero = (0).to_bytes(2, "little", signed=True)
                await tx(write_array_cmd2(0x7B, zero + zero))

                if request.remote_disable:
                    await tx(write_cmd2(0x7A, 0))

                robot_state["last_command"] = f"joystick:x={request.x},y={request.y}"

            except Exception as e:
                robot_state["last_response"] = f"error: {str(e)}"
                await asyncio.to_thread(database.add_log, "ERROR", "Joystick", f"Joystick error: {str(e)}")

    background_tasks.add_task(run_joystick)

    return JSONResponse(
        content={
            "status": "started",
            "x": request.x,
            "y": request.y,
            "duration": request.duration,
        }
    )


@app.post("/api/write")
async def write_register(request: WriteRequest):
    if request.reg not in REG:
        raise HTTPException(
            status_code=400,
            detail=f"Unknown register. Valid: {', '.join(REG.keys())}",
        )

    reg = REG[request.reg]

    if request.protocol == "cmd":
        frame = write_cmd(reg, request.value)
    else:
        frame = write_cmd2(reg, request.value)

    result = await send_frame(frame, request.xor_key)
    robot_state["last_command"] = f"write:{request.reg}={request.value}"
    await asyncio.to_thread(database.add_log, "CMD", "Register", f"Wrote {request.value} to {request.reg}")

    return JSONResponse(
        content={
            "status": "sent",
            "register": request.reg,
            "address": hex(reg),
            "value": request.value,
            "frame": result,
        }
    )


@app.post("/api/light")
async def set_light(request: LightRequest):
    val = 0
    if request.headlight: val |= 0x01
    if request.brakelight: val |= 0x02
    if request.lock_shutdown: val |= 0x04
    if request.lock_warn: val |= 0x08
    if request.back_alarm: val |= 0x10
    if request.carlight: val |= 0x80

    frame = write_cmd2(REG["light_flags"], val)
    result = await send_frame(frame, request.xor_key)
    robot_state["last_command"] = f"light:{val}"
    await asyncio.to_thread(database.add_log, "CMD", "Light", f"Set lights val={val}")

    return JSONResponse(
        content={
            "status": "sent",
            "value": val,
            "bits": {
                "headlight": request.headlight,
                "brakelight": request.brakelight,
                "lock_shutdown": request.lock_shutdown,
                "lock_warn": request.lock_warn,
                "back_alarm": request.back_alarm,
                "carlight": request.carlight,
            },
            "frame": result,
        }
    )


@app.post("/api/raw")
async def send_raw(request: RawRequest):
    clean = "".join(ch for ch in request.hex if ch in "0123456789abcdefABCDEF")
    if len(clean) % 2:
        clean = "0" + clean
    if not clean:
        raise HTTPException(status_code=400, detail="Invalid hex data")

    frame = bytes.fromhex(clean)
    result = await send_frame(frame, request.xor_key)
    robot_state["last_command"] = f"raw:{request.hex}"
    await asyncio.to_thread(database.add_log, "CMD", "Raw", f"Sent raw hex: {request.hex}")

    return JSONResponse(
        content={
            "status": "sent",
            "frame": result,
        }
    )


@app.post("/api/drive")
async def drive_channel(request: DriveRequest):
    if request.axis is not None:
        axis = request.axis
        if axis < -128 or axis > 127:
            raise HTTPException(status_code=400, detail="axis must be -128..127")
    else:
        pct = max(-100, min(100, request.percent))
        axis = int(round((pct * 127) / 100))

    value = encode_drive_value(axis)
    addr = remote_channel_addr(request.channel)

    frames = []
    for _ in range(max(1, request.repeat)):
        frames.append(write_cmd2(addr, value))
    if request.stop_after:
        frames.append(write_cmd2(addr, encode_drive_value(0)))

    results = await send_frames(frames, request.xor_key)
    robot_state["last_command"] = f"drive:ch={request.channel},axis={axis}"
    await asyncio.to_thread(database.add_log, "CMD", "Drive", f"Sent drive channel {request.channel} axis {axis}")

    return JSONResponse(
        content={
            "status": "sent",
            "channel": request.channel,
            "axis": axis,
            "frames": results,
        }
    )


@app.get("/api/registers")
async def list_registers():
    return JSONResponse(
        content={
            "registers": {k: hex(v) for k, v in REG.items()},
        }
    )


@app.get("/api/battery")
async def read_battery(
    xor_key: str = "55",
    tries: int = 4,
    interval: float = 0.15,
    timeout: float = 4.0,
    min_dv: int = 195,
    max_dv: int = 252,
):
    if xor_key not in ("none", "55", "d8"):
        raise HTTPException(status_code=400, detail="xor_key must be none/55/d8")
    if tries < 1:
        raise HTTPException(status_code=400, detail="tries must be >= 1")
    if interval < 0:
        raise HTTPException(status_code=400, detail="interval must be >= 0")
    if timeout <= 0:
        raise HTTPException(status_code=400, detail="timeout must be > 0")

    async with robot_lock:
        client = await get_client()
        profile = choose_profile(client.services)
        if profile == "nus":
            write_uuid = CHR_WRITE_NUS
            notify_uuid = CHR_NOTIFY_NUS
        elif profile == "ffe":
            write_uuid = CHR_FFE
            notify_uuid = CHR_FFE
        else:
            raise HTTPException(status_code=500, detail="Unknown BLE profile")

        got = asyncio.Event()
        result: dict[str, Any] = {
            "raw": None,
            "mode": "unknown",
            "telemetry_payload": None,
            "telemetry_mode": "unknown",
        }

        def on_notify(_, data: bytearray):
            parsed = decode_frame_any(bytes(data))
            if not parsed:
                return
            cmd, typ, addr, payload, mode = parsed
            if cmd == 0x0D and typ == 0x01 and addr == 0xBB and len(payload) >= 2:
                result["raw"] = int.from_bytes(payload[:2], "little", signed=False)
                result["mode"] = mode
                got.set()
            elif cmd == 0x0D and typ == 0x01 and addr == 0x1F and len(payload) >= 4:
                result["telemetry_payload"] = bytes(payload)
                result["telemetry_mode"] = mode

        await client.start_notify(notify_uuid, on_notify)
        try:
            frame = read_cmd2(0xBB, 0x02)
            telemetry_frame = read_cmd2(0x1F, 0x10)
            wire = maybe_xor(frame, xor_key)
            telemetry_wire = maybe_xor(telemetry_frame, xor_key)
            for _ in range(tries):
                await client.write_gatt_char(write_uuid, telemetry_wire, response=False)
                await asyncio.sleep(interval)
                await client.write_gatt_char(write_uuid, wire, response=False)
                await asyncio.sleep(interval)

            try:
                await asyncio.wait_for(got.wait(), timeout=timeout)
            except asyncio.TimeoutError as exc:
                await asyncio.to_thread(database.add_log, "WARN", "Battery", "Battery read timeout")
                raise HTTPException(
                    status_code=504,
                    detail="No battery response received",
                ) from exc
        finally:
            await client.stop_notify(notify_uuid)

    raw = result["raw"]
    volts = raw / 10.0
    pct_est = estimate_percent(raw, min_dv, max_dv)

    payload_1f = result.get("telemetry_payload")
    pct_level = None
    if payload_1f:
        pct_level = extract_battery1_level(payload_1f, raw)

    if pct_level is not None:
        pct = pct_level
        pct_source = "battery1Level(0x1F)"
    else:
        pct = pct_est
        pct_source = "voltage_estimate(0xBB)"

    robot_state["last_command"] = "battery:read"
    
    # Save to SQLite
    await asyncio.to_thread(database.add_battery_record, pct, volts, raw)
    await asyncio.to_thread(database.add_log, "INFO", "Battery", f"Recorded battery: {pct}% ({volts}V)")

    return JSONResponse(
        content={
            "status": "ok",
            "raw_deci_volt": raw,
            "voltage": round(volts, 2),
            "battery_percent": pct,
            "battery_percent_source": pct_source,
            "battery_percent_estimate": pct_est,
            "battery1_level": pct_level,
            "range_deci_volt": {"min": min_dv, "max": max_dv},
            "range_volt": {"min": round(min_dv / 10.0, 2), "max": round(max_dv / 10.0, 2)},
            "rx_decode_mode": result["mode"],
            "telemetry_decode_mode": result.get("telemetry_mode", "unknown"),
            "request_xor_key": xor_key,
        }
    )


if __name__ == "__main__":
    import uvicorn
    import os
    if not os.path.exists("static"):
        os.makedirs("static")
    uvicorn.run(app, host="0.0.0.0", port=8000)
