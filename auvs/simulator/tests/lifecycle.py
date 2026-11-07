"""Exercise a real AuvLoader reload, SIGINT cleanup, and the legacy plugin."""
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

executable, plugin, backend, config, stationary, stationary_config = map(lambda p: str(Path(p).resolve()), sys.argv[1:7])
env = {k: v for k, v in os.environ.items() if not k.startswith("AUTONOMY_SIM_")}
env.update(AUTONOMY_SIM_BACKEND=backend, AUTONOMY_SIM_SCENE="robot_only", AUTONOMY_SIM_REALTIME="1")
if len(sys.argv) > 7:
    env["LD_PRELOAD"] = sys.argv[7] + (":" + env["LD_PRELOAD"] if env.get("LD_PRELOAD") else "")

def wait_for(process, path, condition):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        text = path.read_text()
        if condition(text):
            return
        assert process.poll() is None, text[-4000:]
        time.sleep(.01)
    raise AssertionError(path.read_text()[-4000:])

def controller_config(source, destination):
    with socket.socket() as port:
        port.bind(("127.0.0.1", 0))
        number = port.getsockname()[1]
    value = json.loads(Path(source).read_text())
    value["teleop_port"] = number
    destination.write_text(json.dumps(value))

with tempfile.TemporaryDirectory(prefix="sim-zig-lifecycle-") as temporary:
    root = Path(temporary)
    local_plugin = root / "libauv_test.so"
    shutil.copy2(plugin, local_plugin)
    cfg = root / "controller.json"
    controller_config(config, cfg)
    stdout_path, stderr_path = root / "out", root / "err"
    with stdout_path.open("w") as out, stderr_path.open("w") as err:
        process = subprocess.Popen([executable, str(local_plugin), "teleop", str(cfg)], env=env, cwd=root, stdout=out, stderr=err)
        try:
            wait_for(process, stderr_path, lambda s: "control:" in s)
            replacement = root / "replacement.so"
            shutil.copy2(plugin, replacement)
            os.replace(replacement, local_plugin)  # MOVED_TO event; never overwrite a mapped ELF.
            wait_for(process, stderr_path, lambda s: "restarted auv loop" in s)
            time.sleep(.05)
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=15) == 0, stderr_path.read_text()[-4000:]
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    summaries = [json.loads(s) for s in stdout_path.read_text().splitlines() if s.startswith('{"steps":')]
    assert len(summaries) == 2 and all(s["status"] == 0 and s["steps"] > 0 for s in summaries), summaries
    assert "RestartMission" not in stderr_path.read_text(), stderr_path.read_text()[-4000:]
    controller_config(stationary_config, cfg)
    with stdout_path.open("w") as out, stderr_path.open("w") as err:
        process = subprocess.Popen([executable, stationary, "teleop", str(cfg)], env=env, cwd=root, stdout=out, stderr=err)
        try:
            wait_for(process, stderr_path, lambda s: "control:" in s)
            time.sleep(.05)
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=15) == 0, stderr_path.read_text()[-4000:]
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
print("same-ABI plugin reload creates a fresh world; legacy stationary plugin and SIGINT cleanup passed")
