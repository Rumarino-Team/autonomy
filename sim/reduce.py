#!/usr/bin/env python3
"""Reduce a saved physics failure while preserving the first failing condition."""
import argparse
import base64
import copy
import importlib.util
import json
from pathlib import Path
import subprocess

MASK = (1 << 64) - 1

def mix64(x):
    x = (x + 0x9E3779B97F4A7C15) & MASK
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & MASK
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & MASK
    return x ^ (x >> 31)

def scene_wire():
    path = Path(__file__).resolve().parents[1] / "tools/blender/autonomy_sim/wire.py"
    spec = importlib.util.spec_from_file_location("scene_wire", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("failure", type=Path, help="failure.json with the scene, seed, ticks, and backend")
    parser.add_argument("--simctl", type=Path, default=Path("build/portable/sim/simctl"))
    parser.add_argument("--attempts", type=int, default=64)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    manifest = json.loads(args.failure.read_text())
    if manifest["mode"] != "physics":
        parser.error("mission failures replay with the saved seed; this reducer targets physics inputs")
    directory = args.failure.resolve().parent
    embedded = base64.b64decode(manifest["scene"]) if manifest.get("scene") else None
    source = None if embedded is not None or manifest.get("source_scenario") in (None, "-") else Path(manifest["source_scenario"])
    if embedded is None and source is None:
        parser.error("failure.json needs a .blend source_scenario or an embedded scene")
    seed = int(manifest["seed"])
    ticks = int(manifest["ticks"])
    backend = manifest["backend"]
    attempts = 0
    signature = None
    wire = scene_wire()
    def reason(run):
        text = run.stderr.decode(errors="replace")
        return next((line for line in reversed(text.splitlines()) if line.startswith("simctl:")), "")
    def invoke(command, payload):
        return subprocess.run(command, input=payload, capture_output=True, timeout=args.timeout)
    def scene_arg(payload):
        return "-" if payload is not None else str(source)
    def reproduces_ticks(length):
        nonlocal attempts, signature
        if attempts >= args.attempts:
            return False
        attempts += 1
        try:
            run = invoke([str(args.simctl), "run", backend, scene_arg(embedded), str(length), str(seed)], embedded)
        except subprocess.TimeoutExpired:
            return False
        text = reason(run)
        if not run.returncode or not text:
            return False
        if signature is None:
            signature = text
        return text == signature
    def reproduces(document, length):
        nonlocal attempts
        if attempts >= args.attempts:
            return False
        attempts += 1
        payload = wire.encode(document)
        try:
            run = invoke([str(args.simctl), "run", backend, "-", str(length), str(seed), "--resolved"], payload)
        except subprocess.TimeoutExpired:
            return False
        return bool(run.returncode) and reason(run) == signature
    if not reproduces_ticks(ticks):
        raise SystemExit("Original failure did not reproduce; preserved its artifacts unchanged.")
    lo, hi = 1, ticks
    while lo < hi and attempts < args.attempts:
        middle = (lo + hi) // 2
        if reproduces_ticks(middle):
            hi = middle
        else:
            lo = middle + 1
    ticks = hi
    try:
        resolved_run = invoke([str(args.simctl), "resolve", scene_arg(embedded), str(seed)], embedded)
    except subprocess.TimeoutExpired:
        raise SystemExit("resolve timed out")
    if resolved_run.returncode:
        raise SystemExit(resolved_run.stderr.decode(errors="replace").strip() or "resolve failed")
    scenario = wire.decode(resolved_run.stdout)
    count = len(scenario.get("thrusters", []))
    def command(tick):
        return [(mix64((seed + (1000 + i) * 0x94D049BB133111EB + (tick // 72) * 0xD1B54A32D192ED03) & MASK) >> 11) * 2**-53 * .4 - .2 for i in range(count)]
    sequence = scenario.get("command_sequence") or []
    if sequence:
        scenario["command_sequence"] = sequence[:ticks]
    else:
        scenario["command_sequence"] = [command(tick) for tick in range(ticks)]
    if not reproduces(scenario, ticks):
        raise SystemExit("Resolved scene did not reproduce the failure; preserved its artifacts unchanged.")
    index = 0
    while index < len(scenario.get("faults", [])) and attempts < args.attempts:
        candidate_document = copy.deepcopy(scenario)
        del candidate_document["faults"][index]
        if reproduces(candidate_document, ticks):
            scenario = candidate_document
        else:
            index += 1
    chunk = max(1, ticks // 2)
    while chunk and attempts < args.attempts:
        for start in range(0, ticks, chunk):
            candidate_document = copy.deepcopy(scenario)
            width = min(chunk, ticks - start)
            candidate_document["command_sequence"][start:start + chunk] = [[0.0] * count for _ in range(width)]
            if reproduces(candidate_document, ticks):
                scenario = candidate_document
            if attempts >= args.attempts:
                break
        chunk //= 2
    payload = wire.encode(scenario)
    replay = [str(args.simctl.resolve()), "run", backend, "-", str(ticks), str(seed), "--resolved"]
    report = directory / "reduced-replay.json"
    report.write_text(json.dumps({
        "ticks": ticks, "attempts": attempts, "condition": signature, "command": replay,
        "scene": base64.b64encode(payload).decode("ascii")}, indent=2) + "\n")
    print(json.dumps({"replay": str(report), "ticks": ticks, "attempts": attempts}))

if __name__ == "__main__":
    main()
