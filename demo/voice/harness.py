"""Local handlers for the pool demo.

Codex, or anything else, calls these. They edit the JSON and blend files the
simulator already reloads.

    python3 demo/voice/harness.py call describe_controls
    python3 demo/voice/harness.py call set_number '{"file":"hydrus","path":"non_odometry.target_height","value":1.1}'
    python3 demo/voice/harness.py call list_scene_objects
    python3 demo/voice/harness.py call move_object '{"name":"Gate","x":1,"y":0,"z":-2}'

Live Blender, so moves land in the open file instead of launching Blender each time:

    blender auvs/stonefish_sim/data/pool_scene.blend --python demo/voice/blender_server.py
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import handlers


def command_call(name: str, raw: str) -> int:
    try:
        arguments = json.loads(raw)
    except json.JSONDecodeError as exc:
        print(f"arguments are not JSON: {exc}", file=sys.stderr)
        return 2
    result = handlers.call(name, arguments)
    print(json.dumps(result, indent=2))
    return 0 if result.get("ok") else 1


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Pool demo handlers")
    sub = parser.add_subparsers(dest="command", required=True)
    call = sub.add_parser("call", help="Run one handler")
    call.add_argument("name", choices=sorted(handlers.HANDLERS))
    call.add_argument("args", nargs="?", default="{}")
    args = parser.parse_args(argv)
    return command_call(args.name, args.args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
