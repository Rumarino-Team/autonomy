#!/usr/bin/env python3
"""Reduce a saved physics failure while preserving the first failing condition."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile

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
    source = Path(manifest["source_scenario"])
    seed = int(manifest["seed"])
    ticks = int(manifest["ticks"])
    backend = manifest["backend"]
    attempts = 0
    signature = None
    with tempfile.TemporaryDirectory(prefix="autonomy-reduce-") as temporary:
        wire = scene_wire()
        candidate = Path(temporary) / "candidate.asim"
        resolved = Path(temporary) / "resolved.asim"
        def reason(run):
            return next((line for line in reversed(run.stderr.splitlines()) if line.startswith("simctl:")), "")
        def reproduces_ticks(length):
            nonlocal attempts, signature
            if attempts >= args.attempts:
                return False
            attempts += 1
            try:
                run = subprocess.run([str(args.simctl.resolve()), "run", backend, str(source), str(length), str(seed)], capture_output=True, text=True, timeout=args.timeout)
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
            candidate.write_bytes(wire.encode(document))
            try:
                run = subprocess.run([str(args.simctl.resolve()), "run", backend, str(candidate), str(length), str(seed), "--resolved"], capture_output=True, text=True, timeout=args.timeout)
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
        resolved_run = subprocess.run([str(args.simctl.resolve()), "resolve", str(source), str(resolved), str(seed)], capture_output=True, text=True, timeout=args.timeout)
        if resolved_run.returncode:
            raise SystemExit(resolved_run.stderr.strip() or "resolve failed")
        scenario = wire.decode(resolved.read_bytes())
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
    output = directory / "reduced.asim"
    output.write_bytes(scene_wire().encode(scenario))
    replay = [str(args.simctl.resolve()), "run", backend, str(output), str(ticks), str(seed), "--resolved"]
    (directory / "reduced-replay.json").write_text(json.dumps({"ticks": ticks, "attempts": attempts, "condition": signature, "command": replay}, indent=2) + "\n")
    print(json.dumps({"scenario": str(output), "ticks": ticks, "attempts": attempts}))

if __name__ == "__main__":
    main()
