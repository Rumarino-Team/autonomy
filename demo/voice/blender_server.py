"""Blender command server on 127.0.0.1:9876.

Start it inside the Blender that has the pool open:

    blender auvs/stonefish_sim/data/pool_scene.blend --python demo/voice/blender_server.py

The harness speaks the same localhost JSON protocol as the Blender MCP addon
(`execute_code`), plus a `voice_command` message that only runs the demo
commands in blender_commands.py. Each command is applied on Blender's main
thread. Mutating commands save the .blend file so the simulator reloads it.

Set BLENDER_MCP_PORT to use a port other than 9876.
"""

import json
import os
import queue
import socket
import sys
import traceback
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))

import blender_commands

HOST = "127.0.0.1"
PORT = int(os.environ.get("BLENDER_MCP_PORT", "9876"))
JOBS = queue.Queue()


def _execute(command: dict) -> dict:
    kind = command.get("type")
    params = command.get("params") or {}
    if kind == "ping":
        return {"status": "success", "result": "voice"}
    if kind == "voice_command":
        return {"status": "success", "result": blender_commands.dispatch(params)}
    if kind == "execute_code":
        namespace = {"bpy": bpy, "result": None}
        exec(params.get("code", ""), namespace)
        return {"status": "success", "result": namespace.get("result")}
    return {"status": "error", "message": f"Unknown command type: {kind}"}


def _pump() -> float:
    try:
        while True:
            client, command = JOBS.get_nowait()
            try:
                response = _execute(command)
            except Exception:
                response = {"status": "error", "message": traceback.format_exc(limit=4)}
            try:
                client.sendall(json.dumps(response, default=str).encode("utf-8"))
            except OSError:
                pass
            finally:
                client.close()
    except queue.Empty:
        pass
    return 0.05


def _read_command(client: socket.socket) -> dict:
    chunks = []
    client.settimeout(30)
    while sum(len(chunk) for chunk in chunks) < 1_000_000:
        try:
            chunk = client.recv(65536)
        except socket.timeout:
            break
        if not chunk:
            break
        chunks.append(chunk)
        try:
            return json.loads(b"".join(chunks).decode("utf-8"))
        except json.JSONDecodeError:
            continue
    raise ValueError("expected one JSON command")


def _serve(listener: socket.socket) -> None:
    while True:
        try:
            client, _address = listener.accept()
        except OSError:
            return
        try:
            command = _read_command(client)
        except Exception as exc:
            try:
                client.sendall(json.dumps({"status": "error", "message": str(exc)}).encode("utf-8"))
            except OSError:
                pass
            client.close()
            continue
        JOBS.put((client, command))


def start() -> None:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        listener.bind((HOST, PORT))
    except OSError as exc:
        print(f"[voice] blender command port {HOST}:{PORT} is already in use ({exc})")
        return
    listener.listen(4)
    listener.settimeout(None)
    bpy.app.timers.register(_pump, first_interval=0.05, persistent=True)
    import threading

    thread = threading.Thread(target=_serve, args=(listener,), name="voice-blender", daemon=True)
    thread.start()
    print(f"[voice] blender commands listening on {HOST}:{PORT}")


start()
