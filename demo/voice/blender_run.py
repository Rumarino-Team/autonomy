"""One-shot Blender entry point.

    blender pool_scene.blend --background --python demo/voice/blender_run.py -- '{"op":"list_objects"}'

Prints one `VOICE_RESULT` JSON line. Mutating commands save the blend file
before returning, which is what makes the running simulator reload the scene.
"""

import json
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import blender_commands


def main() -> None:
    try:
        if "--" not in sys.argv:
            result = {"ok": False, "error": "missing command after --"}
        else:
            raw = sys.argv[sys.argv.index("--") + 1]
            result = blender_commands.dispatch(json.loads(raw))
    except Exception:
        result = {"ok": False, "error": traceback.format_exc(limit=4)}
    print(blender_commands.SENTINEL + json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
