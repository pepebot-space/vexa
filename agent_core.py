import asyncio
import base64
import json
import os
import traceback
from typing import Optional

import audioop
import pyaudio
import websockets

try:
    import cv2
except ImportError:
    cv2 = None

# Config
INPUT_RATE = 16000
OUTPUT_RATE = 24000
CHANNELS = 1
SAMPLE_WIDTH = 2
FORMAT = pyaudio.paInt16

INPUT_CHUNK = 2048
OUTPUT_CHUNK = 4096
OUTPUT_PREBUFFER_CHUNKS = 3

URL = "ws://localhost:18790/v1/live"

ENABLE_NOISE_GATE = True
NOISE_FLOOR_ALPHA = 0.95
NOISE_GATE_MULTIPLIER = 2.0
NOISE_GATE_MIN_RMS = 180
NOISE_GATE_HANGOVER = 3

ENABLE_BARGE_IN = False
BOT_SPEAKING_HOLD_SEC = 0.8

ENABLE_CAMERA = True
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", "0"))
VIDEO_MIME = "image/jpeg"
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 360
VIDEO_JPEG_QUALITY = 70
VIDEO_INTERVAL_SEC = 0.5

INPUT_DEVICE_INDEX_ENV = os.getenv("INPUT_DEVICE_INDEX")
INPUT_DEVICE_NAME = os.getenv("INPUT_DEVICE_NAME")
INPUT_DEVICE_INDEX = int(INPUT_DEVICE_INDEX_ENV) if INPUT_DEVICE_INDEX_ENV else None


class NoiseGate:
    def __init__(self):
        self.noise_floor = float(NOISE_GATE_MIN_RMS)
        self.hangover_left = 0

    def process(self, pcm_bytes: bytes) -> bytes:
        if not pcm_bytes:
            return pcm_bytes
        rms = audioop.rms(pcm_bytes, SAMPLE_WIDTH)
        if rms < self.noise_floor * 1.5:
            self.noise_floor = (
                NOISE_FLOOR_ALPHA * self.noise_floor + (1 - NOISE_FLOOR_ALPHA) * rms
            )
        threshold = max(NOISE_GATE_MIN_RMS, self.noise_floor * NOISE_GATE_MULTIPLIER)
        is_speech = rms >= threshold
        if is_speech:
            self.hangover_left = NOISE_GATE_HANGOVER
            return pcm_bytes
        if self.hangover_left > 0:
            self.hangover_left -= 1
            return pcm_bytes
        return b"\x00" * len(pcm_bytes)


def try_parse_json(data):
    try:
        if isinstance(data, bytes):
            return json.loads(data.decode("utf-8", errors="ignore"))
        return json.loads(data)
    except Exception:
        return None


def extract_inline_audio(parsed: dict) -> Optional[bytes]:
    if not isinstance(parsed.get("serverContent"), dict):
        return None
    if not isinstance(parsed["serverContent"].get("modelTurn"), dict):
        return None
    parts = parsed["serverContent"]["modelTurn"].get("parts")
    if not isinstance(parts, list):
        return None

    chunks = []
    for part in parts:
        if not isinstance(part, dict):
            continue
        if not isinstance(part.get("inlineData"), dict):
            continue
        b64_audio = part["inlineData"].get("data")
        if not isinstance(b64_audio, str) or not b64_audio:
            continue

        normalized = b64_audio.replace("-", "+").replace("_", "/")
        while len(normalized) % 4 != 0:
            normalized += "="
        try:
            chunks.append(base64.b64decode(normalized))
        except Exception:
            pass

    return b"".join(chunks) if chunks else None


# Global state
agent_task = None
stop_event = None
latest_frame = None
agent_status = "idle"


async def start_agent_loop(target_speaker_name: Optional[str] = None):
    global latest_frame, agent_status
    agent_status = "starting"

    p = pyaudio.PyAudio()
    output_queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=256)
    bot_speaking_until = 0.0
    video_enabled = False

    output_device_index = None
    if target_speaker_name:
        # First pass: try to match the exact BLE device name
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info.get("maxOutputChannels", 0) > 0 and info.get("name"):
                if target_speaker_name.lower() in info["name"].lower():
                    output_device_index = i
                    print(f"Matched BLE Speaker: {info['name']}")
                    break

        # Second pass: macOS often just names Bluetooth audio sinks "Bluetooth"
        if output_device_index is None:
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if info.get("maxOutputChannels", 0) > 0 and info.get("name"):
                    if "bluetooth" in info["name"].lower():
                        output_device_index = i
                        print(
                            f"Fallback matched Generic Bluetooth Speaker: {info['name']}"
                        )
                        break

    actual_out_rate = OUTPUT_RATE
    actual_out_channels = CHANNELS

    try:
        stream_out = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=OUTPUT_RATE,
            output=True,
            frames_per_buffer=OUTPUT_CHUNK,
            output_device_index=output_device_index,
        )
    except Exception as e:
        print(f"Could not open device {output_device_index} at 24kHz Mono: {e}")
        if output_device_index is not None:
            try:
                info = p.get_device_info_by_index(output_device_index)
                actual_out_rate = int(info.get("defaultSampleRate", 44100))
                actual_out_channels = int(info.get("maxOutputChannels", 2))
                if actual_out_channels < 1:
                    actual_out_channels = 2
                stream_out = p.open(
                    format=FORMAT,
                    channels=actual_out_channels,
                    rate=actual_out_rate,
                    output=True,
                    frames_per_buffer=OUTPUT_CHUNK,
                    output_device_index=output_device_index,
                )
                print(f"Fallback success: {actual_out_rate}Hz, {actual_out_channels}Ch")
            except Exception as e2:
                print(f"Fallback to native settings failed: {e2}")
                output_device_index = None
                actual_out_rate = OUTPUT_RATE
                actual_out_channels = CHANNELS
                stream_out = p.open(
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=OUTPUT_RATE,
                    output=True,
                    frames_per_buffer=OUTPUT_CHUNK,
                    output_device_index=None,
                )
        else:
            stream_out = p.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=OUTPUT_RATE,
                output=True,
                frames_per_buffer=OUTPUT_CHUNK,
                output_device_index=None,
            )

    input_device_index = INPUT_DEVICE_INDEX
    input_device_info = None
    if input_device_index is None and INPUT_DEVICE_NAME:
        target = INPUT_DEVICE_NAME.lower()
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            name = info.get("name") if isinstance(info.get("name"), str) else None
            max_in = info.get("maxInputChannels", 0)
            if (
                isinstance(max_in, (int, float))
                and max_in > 0
                and name
                and target in name.lower()
            ):
                input_device_index = i
                input_device_info = info
                print(f"Matched input device: {name} (index {i})")
                break

    if input_device_index is not None and input_device_info is None:
        try:
            input_device_info = p.get_device_info_by_index(input_device_index)
        except Exception:
            input_device_info = None

    actual_in_channels = CHANNELS
    stream_in = None
    try:
        stream_in = p.open(
            format=FORMAT,
            channels=actual_in_channels,
            rate=INPUT_RATE,
            input=True,
            frames_per_buffer=INPUT_CHUNK,
            input_device_index=input_device_index,
        )
    except OSError as e:
        print(
            f"Could not open input device {input_device_index} at{INPUT_RATE}Hz {actual_in_channels}ch: {e}"
        )
        reopened = False
        if input_device_info is None and input_device_index is not None:
            try:
                input_device_info = p.get_device_info_by_index(input_device_index)
            except Exception:
                input_device_info = None
        if input_device_info:
            max_in = input_device_info.get("maxInputChannels", 0)
            if isinstance(max_in, (int, float)) and max_in > 0:
                fallback_channels = int(max(1, min(max_in, 2)))
                if fallback_channels != actual_in_channels:
                    try:
                        stream_in = p.open(
                            format=FORMAT,
                            channels=fallback_channels,
                            rate=INPUT_RATE,
                            input=True,
                            frames_per_buffer=INPUT_CHUNK,
                            input_device_index=input_device_index,
                        )
                        actual_in_channels = fallback_channels
                        reopened = True
                        print(f"Fallback input open success: {fallback_channels}ch")
                    except Exception as e2:
                        print(f"Fallback open with {fallback_channels}ch failed: {e2}")
        if not reopened:
            try:
                stream_in = p.open(
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=INPUT_RATE,
                    input=True,
                    frames_per_buffer=INPUT_CHUNK,
                    input_device_index=None,
                )
                input_device_index = None
                actual_in_channels = CHANNELS
                reopened = True
                print("Fallback to default input device succeeded")
            except Exception as e3:
                print(f"Fallback to default input device failed: {e3}")
                raise

    if stream_in is None:
        raise RuntimeError("Failed to open any input audio device")

    noise_gate = NoiseGate()

    async def enqueue_audio(pcm: bytes):
        nonlocal bot_speaking_until
        if not pcm or len(pcm) % 2 != 0 and not (pcm := pcm[:-1]):
            return
        try:
            await asyncio.wait_for(output_queue.put(pcm), timeout=0.5)
            bot_speaking_until = max(
                bot_speaking_until,
                asyncio.get_running_loop().time() + BOT_SPEAKING_HOLD_SEC,
            )
        except asyncio.TimeoutError:
            pass

    async def playback_worker():
        bytes_per_out_chunk = OUTPUT_CHUNK * SAMPLE_WIDTH
        prebuffer_target = OUTPUT_PREBUFFER_CHUNKS * bytes_per_out_chunk
        pending = bytearray()
        started = False
        resample_state = None

        while not stop_event.is_set():
            try:
                pcm = await asyncio.wait_for(output_queue.get(), timeout=0.02)
                pending.extend(pcm)
            except asyncio.TimeoutError:
                pass
            if not started:
                if len(pending) < prebuffer_target:
                    continue
                started = True
            if len(pending) >= bytes_per_out_chunk:
                frame = bytes(pending[:bytes_per_out_chunk])
                del pending[:bytes_per_out_chunk]
            else:
                frame = bytes(pending) + (
                    b"\x00" * (bytes_per_out_chunk - len(pending))
                )
                pending.clear()

            final_frame = frame
            if actual_out_rate != OUTPUT_RATE:
                final_frame, resample_state = audioop.ratecv(
                    final_frame,
                    SAMPLE_WIDTH,
                    CHANNELS,
                    OUTPUT_RATE,
                    actual_out_rate,
                    resample_state,
                )
            if actual_out_channels == 2 and CHANNELS == 1:
                final_frame = audioop.tostereo(final_frame, SAMPLE_WIDTH, 1, 1)

            try:
                await asyncio.to_thread(stream_out.write, final_frame)
            except Exception:
                return

    try:
        async with websockets.connect(
            URL,
            max_size=20 * 1024 * 1024,
            ping_interval=20,
            ping_timeout=20,
            close_timeout=5,
        ) as ws:
            agent_status = "connected"
            await ws.send(
                json.dumps(
                    {
                        "setup": {
                            "provider": "vertex",
                            "model": "gemini-live-2.5-flash-native-audio",
                            "agent": "default",
                            "enable_tools": True,
                        }
                    }
                )
            )

            setup_ok = False
            while not setup_ok and not stop_event.is_set():
                msg = await asyncio.wait_for(ws.recv(), timeout=15)
                parsed = try_parse_json(msg)
                if not parsed:
                    continue
                if parsed.get("error"):
                    agent_status = f"error: {parsed['error']}"
                    return
                if parsed.get("status") == "connected":
                    video_enabled = bool(parsed.get("video", {}).get("enabled"))
                    continue
                if "setupComplete" in parsed:
                    setup_ok = True

            if not setup_ok:
                return
            agent_status = "live"

            async def sender_audio():
                while not stop_event.is_set():
                    try:
                        if (
                            not ENABLE_BARGE_IN
                            and asyncio.get_running_loop().time() < bot_speaking_until
                        ):
                            await asyncio.sleep(0.02)
                            continue
                        data = await asyncio.to_thread(
                            stream_in.read, INPUT_CHUNK, exception_on_overflow=False
                        )
                        if ENABLE_NOISE_GATE:
                            data = noise_gate.process(data)
                        b64_data = base64.b64encode(data).decode("utf-8")
                        await ws.send(
                            json.dumps(
                                {
                                    "realtimeInput": {
                                        "mediaChunks": [
                                            {
                                                "mimeType": "audio/pcm;rate=16000",
                                                "data": b64_data,
                                            }
                                        ]
                                    }
                                }
                            )
                        )
                    except Exception as e:
                        print(f"sender_audio exception: {e}")
                        stop_event.set()
                        return

            async def sender_video():
                global latest_frame
                if not ENABLE_CAMERA or cv2 is None:
                    return
                cap = await asyncio.to_thread(cv2.VideoCapture, CAMERA_INDEX)
                if not cap or not cap.isOpened():
                    print("Could not open camera")
                    return
                await asyncio.to_thread(cap.set, cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
                await asyncio.to_thread(
                    cap.set, cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT
                )
                try:
                    while not stop_event.is_set():
                        ok, frame = await asyncio.to_thread(cap.read)
                        if not ok:
                            await asyncio.sleep(VIDEO_INTERVAL_SEC)
                            continue
                        ok_jpg, encoded = await asyncio.to_thread(
                            cv2.imencode,
                            ".jpg",
                            frame,
                            [int(cv2.IMWRITE_JPEG_QUALITY), VIDEO_JPEG_QUALITY],
                        )
                        if not ok_jpg:
                            await asyncio.sleep(VIDEO_INTERVAL_SEC)
                            continue
                        latest_frame = encoded.tobytes()
                        b64 = base64.b64encode(latest_frame).decode("utf-8")
                        if video_enabled:
                            await ws.send(
                                json.dumps(
                                    {
                                        "realtimeInput": {
                                            "mediaChunks": [
                                                {"mimeType": VIDEO_MIME, "data": b64}
                                            ]
                                        }
                                    }
                                )
                            )
                        await asyncio.sleep(VIDEO_INTERVAL_SEC)
                except Exception as e:
                    print(f"sender_video exception: {e}")
                finally:
                    await asyncio.to_thread(cap.release)

            async def receiver():
                while not stop_event.is_set():
                    try:
                        message = await ws.recv()
                    except Exception as e:
                        print(f"receiver exception: {e.__class__.__name__} - {e}")
                        stop_event.set()
                        return
                    if isinstance(message, bytes):
                        parsed_bin = try_parse_json(message)
                        audio_inline = (
                            extract_inline_audio(parsed_bin)
                            if isinstance(parsed_bin, dict)
                            else None
                        )
                    else:
                        parsed = try_parse_json(message)
                        if parsed and parsed.get("error"):
                            print(f"Server error message: {parsed['error']}")
                            continue
                        audio_inline = extract_inline_audio(parsed) if parsed else None

                    if (
                        audio_inline
                        and len(audio_inline) >= 2
                        and len(audio_inline) % 2 == 0
                    ):
                        await enqueue_audio(audio_inline)

            tasks = [
                asyncio.create_task(playback_worker()),
                asyncio.create_task(sender_audio()),
                asyncio.create_task(sender_video()),
                asyncio.create_task(receiver()),
            ]
            print("Agent looping...")
            await stop_event.wait()
            print("stop_event was set, exiting agent loop.")
            for t in tasks:
                t.cancel()
    except Exception as e:
        agent_status = "error"
        print("Agent error:", e)
    finally:
        agent_status = "idle"
        print("Agent finally block, cleaning up...")
        try:
            stream_in.stop_stream()
            stream_in.close()
        except Exception:
            pass
        try:
            stream_out.stop_stream()
            stream_out.close()
        except Exception:
            pass
        p.terminate()


def start_agent(target_speaker_name: Optional[str] = None):
    global agent_task, stop_event
    if agent_task and not agent_task.done():
        return
    stop_event = asyncio.Event()
    agent_task = asyncio.create_task(start_agent_loop(target_speaker_name))


def stop_agent():
    if stop_event:
        stop_event.set()


def get_latest_frame():
    return latest_frame


def get_status():
    return agent_status
