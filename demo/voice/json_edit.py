"""Edit allowlisted simulation JSON and ocean YAML in place.

The running simulator watches these files and reloads them. Writes go to a
temporary file in the same directory and then replace the target, so the
watcher's rename event sees the final name.
"""

import json
import math
import os
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

FILES = {
    "platform": REPO / "auvs/stonefish_sim/platform.json",
    "hydrus": REPO / "auvs/stonefish_sim/hydrus.json",
    "proteus": REPO / "auvs/stonefish_sim/proteus.json",
    "bluerov2": REPO / "auvs/stonefish_sim/bluerov2.json",
    "girona500": REPO / "auvs/stonefish_sim/girona500.json",
    "ocean": REPO / "auvs/stonefish_sim/blender_stonefish/config.yaml",
}

ROBOTS = ("hydrus", "proteus", "bluerov2", "girona500")


def parse_path(path: str) -> list:
    if not isinstance(path, str) or not path or ".." in path:
        raise ValueError("path must be a dotted field such as non_odometry.target_height")
    parts = []
    for token in path.replace("[", ".").replace("]", "").split("."):
        if token == "":
            continue
        if token.isdigit():
            parts.append(int(token))
        elif re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", token):
            parts.append(token)
        else:
            raise ValueError(f"bad path token {token!r}")
    if not parts:
        raise ValueError("empty path")
    return parts


def format_number(value) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("value must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("value must be finite")
    if isinstance(value, int):
        return str(value)
    return format(number, ".12g")


def _skip(text: str, index: int) -> int:
    while index < len(text) and text[index] in " \t\r\n":
        index += 1
    return index


def _parse_string(text: str, index: int) -> int:
    if text[index] != '"':
        raise ValueError("expected a string")
    index += 1
    while index < len(text):
        char = text[index]
        if char == "\\":
            if index + 1 < len(text) and text[index + 1] == "u":
                index += 6
            else:
                index += 2
            continue
        if char == '"':
            return index + 1
        index += 1
    raise ValueError("unterminated string")


def _scan_number(text: str, index: int) -> int:
    start = index
    if text[index] == "-":
        index += 1
    if index >= len(text) or not text[index].isdigit():
        raise ValueError(f"bad number at {start}")
    if text[index] == "0":
        index += 1
    else:
        while index < len(text) and text[index].isdigit():
            index += 1
    if index < len(text) and text[index] == ".":
        index += 1
        if index >= len(text) or not text[index].isdigit():
            raise ValueError("bad number fraction")
        while index < len(text) and text[index].isdigit():
            index += 1
    if index < len(text) and text[index] in "eE":
        index += 1
        if index < len(text) and text[index] in "+-":
            index += 1
        if index >= len(text) or not text[index].isdigit():
            raise ValueError("bad number exponent")
        while index < len(text) and text[index].isdigit():
            index += 1
    return index


def _walk(text: str, index: int, path: list, target: list, hit: list) -> int:
    index = _skip(text, index)
    if index >= len(text):
        raise ValueError("unexpected end of json")
    char = text[index]
    if char == "{":
        index += 1
        while True:
            index = _skip(text, index)
            if text[index] == "}":
                return index + 1
            key_start = index
            key_end = _parse_string(text, index)
            key = json.loads(text[key_start:key_end])
            index = _skip(text, key_end)
            if text[index] != ":":
                raise ValueError("expected ':'")
            child = path + [key]
            index = _walk(text, index + 1, child, target, hit)
            index = _skip(text, index)
            if text[index] == ",":
                index += 1
                continue
            if text[index] == "}":
                return index + 1
            raise ValueError("expected ',' or '}'")
    if char == "[":
        index += 1
        item = 0
        while True:
            index = _skip(text, index)
            if text[index] == "]":
                return index + 1
            child = path + [item]
            index = _walk(text, index, child, target, hit)
            item += 1
            index = _skip(text, index)
            if text[index] == ",":
                index += 1
                continue
            if text[index] == "]":
                return index + 1
            raise ValueError("expected ',' or ']'")
    if char == '"':
        end = _parse_string(text, index)
        kind = "string"
    elif text.startswith("true", index):
        end = index + 4
        kind = "bool"
    elif text.startswith("false", index):
        end = index + 5
        kind = "bool"
    elif text.startswith("null", index):
        end = index + 4
        kind = "null"
    else:
        end = _scan_number(text, index)
        kind = "number"
    if path == target:
        hit.append((index, end, kind))
    return end


def replace_json_span(text: str, path: list, literal: str, expect: str) -> str:
    hit = []
    end = _walk(text, 0, [], path, hit)
    _skip(text, end)
    if len(hit) != 1:
        joined = ".".join(str(part) for part in path)
        raise KeyError(f"{joined} not found" if not hit else f"{joined} matched more than once")
    start, stop, kind = hit[0]
    if kind != expect:
        joined = ".".join(str(part) for part in path)
        raise TypeError(f"{joined} is a {kind}, not a {expect}")
    return text[:start] + literal + text[stop:]


def _split_yaml_comment(rest: str) -> tuple[str, str]:
    bracket = 0
    for index, char in enumerate(rest):
        if char == "[":
            bracket += 1
        elif char == "]":
            bracket -= 1
        elif char == "#" and bracket == 0:
            return rest[:index], rest[index:]
    return rest, ""


def _line_ending(line: str) -> tuple[str, str]:
    if line.endswith("\r\n"):
        return line[:-2], "\r\n"
    if line.endswith("\n"):
        return line[:-1], "\n"
    return line, ""


def _rewrite_yaml_line(line: str, rendered: str) -> str:
    core, ending = _line_ending(line)
    indent = core[: len(core) - len(core.lstrip(" "))]
    body = core.strip()
    key, _, rest = body.partition(":")
    _value, comment = _split_yaml_comment(rest)
    suffix = f"  {comment.strip()}" if comment.strip() else ""
    return f"{indent}{key}: {rendered}{suffix}{ending}"


def set_yaml_number(text: str, path: list, value) -> str:
    keys = list(path)
    index = None
    if keys and isinstance(keys[-1], int):
        index = keys.pop()
    if not keys or any(isinstance(part, int) for part in keys):
        raise ValueError("yaml path must name a field, with an optional final index")
    lines = text.splitlines(keepends=True)
    stack: list[tuple[int, str]] = []
    for line_index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("- "):
            continue
        if ":" not in stripped:
            continue
        indent = len(line) - len(line.lstrip(" "))
        key = stripped.split(":", 1)[0].strip()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        chain = [item[1] for item in stack] + [key]
        rest = stripped.split(":", 1)[1]
        value_part, _comment = _split_yaml_comment(rest)
        nested = value_part.strip() == ""
        if chain == keys:
            if index is None:
                if nested:
                    raise ValueError("path is a group of fields, not a number")
                lines[line_index] = _rewrite_yaml_line(line, format_number(value))
                return "".join(lines)
            rendered = _replace_flow(value_part.strip(), index, value)
            lines[line_index] = _rewrite_yaml_line(line, rendered)
            return "".join(lines)
        if nested:
            stack.append((indent, key))
    joined = ".".join(str(part) for part in path)
    raise KeyError(joined)


def _replace_flow(token: str, index: int, value) -> str:
    if not (token.startswith("[") and token.endswith("]")):
        raise ValueError("expected a list on one line, such as velocity: [0, 0, 0]")
    parts = [part.strip() for part in token[1:-1].split(",")]
    if index < 0 or index >= len(parts):
        raise IndexError(f"index {index} outside 0..{len(parts) - 1}")
    parts[index] = format_number(value)
    return "[" + ", ".join(parts) + "]"


def atomic_write(path: Path, text: str) -> None:
    temporary = path.with_name(path.name + ".voice-tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)


def resolve(file: str) -> Path:
    try:
        path = FILES[file]
    except KeyError:
        known = ", ".join(FILES)
        raise KeyError(f"unknown file {file!r}; use one of: {known}") from None
    return path


def read_tree(file: str):
    path = resolve(file)
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".json":
        return json.loads(text), text
    import yaml

    return yaml.safe_load(text), text


def lookup(tree, path: list):
    current = tree
    for part in path:
        current = current[part]
    return current


def get_value(file: str, path: str):
    tree, _text = read_tree(file)
    return lookup(tree, parse_path(path))


def set_number(file: str, path: str, value) -> dict:
    parts = parse_path(path)
    target = resolve(file)
    current, text = read_tree(file)
    old = lookup(current, parts)
    if isinstance(old, bool) or not isinstance(old, (int, float)):
        raise TypeError(f"{path} is not a number")
    literal = format_number(value)
    if target.suffix == ".json":
        updated = replace_json_span(text, parts, literal, "number")
    else:
        updated = set_yaml_number(text, parts, value)
    atomic_write(target, updated)
    return {"ok": True, "file": file, "path": path, "old": old, "value": json.loads(literal)}


def set_robot(robot: str) -> dict:
    if robot not in ROBOTS:
        raise ValueError(f"robot must be one of: {', '.join(ROBOTS)}")
    target = FILES["platform"]
    text = target.read_text(encoding="utf-8")
    updated = replace_json_span(text, ["robot"], json.dumps(robot), "string")
    atomic_write(target, updated)
    return {"ok": True, "file": "platform", "path": "robot", "value": robot}


def describe_controls() -> dict:
    platform = json.loads(FILES["platform"].read_text(encoding="utf-8"))
    return {
        "ok": True,
        "platform": {
            "robot": platform.get("robot"),
            "blend_file": platform.get("blend_file"),
            "steps_per_second": platform.get("steps_per_second"),
            "realtime_factor_cap": platform.get("realtime_factor_cap"),
            "tracking_ok_seconds": platform.get("tracking_ok_seconds"),
            "min_render_interval_ms": platform.get("min_render_interval_ms"),
        },
        "robots": list(ROBOTS),
        "files": list(FILES),
        "number_examples": [
            "platform steps_per_second",
            "platform realtime_factor_cap",
            "hydrus non_odometry.target_height",
            "hydrus non_odometry.center_deadband",
            "hydrus odometry.kp[0]",
            "ocean environment.ocean.wave_height",
            "ocean environment.ocean.water_density",
            "ocean environment.sun.elevation",
            "ocean environment.ocean.current.velocity[0]",
        ],
        "scene_examples": ["Gate", "Bin", "Board", "Marker", "Pole1"],
        "coordinates": "Blender meters. The simulator mirrors them: sim_x=-x, sim_y=y, sim_z=-z.",
    }


def self_test() -> None:
    hydrus = FILES["hydrus"].read_text(encoding="utf-8")
    tam = hydrus.split('"odometry"', 1)[0]
    updated = replace_json_span(hydrus, ["odometry", "kp", 0], "1.5", "number")
    assert updated.split('"odometry"', 1)[0] == tam
    parsed = json.loads(updated)
    assert parsed["odometry"]["kp"][0] == 1.5
    assert parsed["odometry"]["kp"][1] == json.loads(hydrus)["odometry"]["kp"][1]
    height = replace_json_span(hydrus, ["non_odometry", "target_height"], "1.25", "number")
    assert json.loads(height)["non_odometry"]["target_height"] == 1.25
    platform = FILES["platform"].read_text(encoding="utf-8")
    swapped = replace_json_span(platform, ["robot"], json.dumps("bluerov2"), "string")
    assert json.loads(swapped)["robot"] == "bluerov2"
    assert json.loads(swapped)["steps_per_second"] == json.loads(platform)["steps_per_second"]
    ocean = FILES["ocean"].read_text(encoding="utf-8")
    waves = set_yaml_number(ocean, ["environment", "ocean", "wave_height"], 0.4)
    assert "wave_height: 0.4" in waves
    assert "water_density: 1025.0" in waves
    current = set_yaml_number(ocean, ["environment", "ocean", "current", "velocity", 1], 0.2)
    assert "velocity: [0.0, 0.2, 0.0]" in current
    try:
        replace_json_span(platform, ["console"], "1", "number")
    except TypeError:
        pass
    else:
        raise AssertionError("console is a flag and must not be overwritten")
    print("json_edit self_test ok")


if __name__ == "__main__":
    self_test()
