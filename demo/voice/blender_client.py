"""Run a blender_commands operation in a live Blender, or in a one-shot process.

A live Blender that was started with blender_server.py (or the Blender MCP
addon) is preferred, because it already has the pool file open. If nothing is
listening, this falls back to `blender --background`, which loads the blend
file from platform.json, applies one command, and saves it.
"""

import ast
import json
import os
import shutil
import socket
import subprocess
from pathlib import Path

from json_edit import FILES, REPO

HOST = "127.0.0.1"
PORT = int(os.environ.get("BLENDER_MCP_PORT", "9876"))
RUNNER = Path(__file__).resolve().parent / "blender_run.py"
COMMANDS = Path(__file__).resolve().parent / "blender_commands.py"


def blend_file() -> Path:
    platform = json.loads(FILES["platform"].read_text(encoding="utf-8"))
    path = (REPO / platform["data_path"] / platform["blend_file"]).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _request(payload: dict) -> dict:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.4)
        try:
            sock.connect((HOST, PORT))
        except OSError as exc:
            raise ConnectionError(f"{HOST}:{PORT} is not open ({exc})") from exc
        sock.settimeout(60)
        sock.sendall(json.dumps(payload).encode("utf-8"))
        chunks = []
        while sum(len(chunk) for chunk in chunks) < 1_000_000:
            try:
                chunk = sock.recv(65536)
            except socket.timeout as exc:
                raise ConnectionError("blender command server timed out") from exc
            if not chunk:
                break
            chunks.append(chunk)
            try:
                return json.loads(b"".join(chunks).decode("utf-8"))
            except json.JSONDecodeError:
                continue
    if not chunks:
        raise ConnectionError("blender command server closed without a response")
    raise ConnectionError("blender command server returned incomplete JSON")


def _parse_result(result):
    if isinstance(result, dict):
        return result
    if isinstance(result, str):
        try:
            parsed = json.loads(result)
        except json.JSONDecodeError:
            parsed = ast.literal_eval(result)
        if isinstance(parsed, dict):
            return parsed
    raise ValueError(f"blender returned {result!r}")


def _execute_code(command: dict) -> str:
    payload = json.dumps(command)
    path = json.dumps(str(COMMANDS))
    return (
        "import json\n"
        "import importlib.util\n"
        f"spec = importlib.util.spec_from_file_location('voice_blender_commands', {path})\n"
        "mod = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(mod)\n"
        f"result = mod.dispatch(json.loads({json.dumps(payload)}))\n"
    )


def _via_socket(command: dict) -> dict:
    response = _request({"type": "voice_command", "params": command})
    message = str(response.get("message", ""))
    if response.get("status") != "success" and (
        "unknown" in message.lower() or "voice_command" in message.lower()
    ):
        response = _request({"type": "execute_code", "params": {"code": _execute_code(command)}})
    if response.get("status") != "success":
        return {"ok": False, "error": message or "blender command failed"}
    return _parse_result(response.get("result"))


def _via_background(command: dict) -> dict:
    blender = shutil.which("blender")
    if blender is None:
        return {"ok": False, "error": "blender is not on PATH and no command server is listening"}
    blend = blend_file()
    try:
        proc = subprocess.run(
            [
                blender,
                "--background",
                str(blend),
                "--python",
                str(RUNNER),
                "--",
                json.dumps(command),
            ],
            cwd=REPO,
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "blender timed out after 180s"}
    marker = "VOICE_RESULT "
    for line in reversed(proc.stdout.splitlines()):
        if line.startswith(marker):
            return json.loads(line[len(marker) :])
    tail = "\n".join((proc.stderr or proc.stdout).splitlines()[-20:])
    return {
        "ok": False,
        "error": f"blender exited {proc.returncode} without a result",
        "tail": tail,
    }


def run(command: dict) -> dict:
    if os.environ.get("BLENDER_MCP", "1") == "0":
        return _via_background(command)
    try:
        return _via_socket(command)
    except (ConnectionError, OSError):
        return _via_background(command)
