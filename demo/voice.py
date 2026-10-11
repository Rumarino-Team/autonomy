#!/usr/bin/env python3
"""Single-file Stonefish JSON voice demo (Python 3.10+).

Install: python -m pip install httpx websocket-client
Linux audio: parec and pactl (pulseaudio-utils; also works with PipeWire Pulse).
Set OPENAI_API_KEY in your environment, then:
    python demo/voice.py                         # microphone; Ctrl+C stops
    python demo/voice.py --push-to-send          # speak, then press s (no Enter)
    python demo/voice.py --text 'camera two'      # typed command, no microphone
    python demo/voice.py --text 'speed point five' --dry-run
    python demo/voice.py --two-step --text 'set real-time factor' --value 'one point five' --dry-run
    python demo/voice.py --silence-ms 2000        # allow longer spoken pauses
    python demo/voice.py --text 'camera two' --dry-run --debug-decisions
    python demo/voice.py --text 'cámara dos' --dry-run
    python demo/voice.py --two-step --text 'set real-time factor' --value 'one point five' --dry-run
    python demo/voice.py --controls              # offline field inventory
    python demo/voice.py --list-devices

Examples: camera three; set real-time factor to point five; turn particles off;
set third odometry KP to five; increase second non-odometry KD by ten percent.
Spanish and English work together: cámara dos; velocidad cero coma cinco;
pon el factor de tiempo real a 2,5; apaga las partículas;
pon el tercer KP de odometría a cinco; aumenta la velocidad un diez por ciento.
Wait for "Listening" before each command. While processing, captured audio is
discarded. This prototype uses an adjustable RMS speech gate, not robust VAD.
Commands end after 1.2 seconds of silence by default; --silence-ms adjusts this.
With --push-to-send, only the s key submits; pauses never submit automatically.
Manual commands are discarded after 60 seconds. Keep this terminal focused.
With --two-step, say the setting/action first; numeric edits then prompt for a
separate value utterance. Use --text and --value to test that flow without a mic.
It edits scalar fields in platform.json and the active robot's controller JSON.
Paths, robot selection, and console mode are excluded because they change the
scene source or require a restart. No Blender, YAML, or simulation lifecycle calls.

Transcription: gpt-live-transcribe, persistent transcription-only WebSocket.
Original wording goes unchanged to POST /v1/decisions (gpt-6-luna): setting,
operation, and discrete camera/toggle/quality values. No word dictionary.
Numeric arguments use a second Decisions request. Real-time factor uses exact preset
buckets; other numbers use an ordered score scale. Python validates the typed command,
calculates relative edits, and writes JSON. Reads and boolean flips
skip numeric scoring. Camera/toggle/quality edits use one request; numeric edits use two.
Prototype confidence defaults: selected field and operation 0.5
(--min-command-confidence); numeric and discrete values 0.8 (--min-confidence).
A command must identify one supported field and one compatible operation;
ambiguous or unrelated speech leaves JSON unchanged.
Timing covers command handling only, not the simulator's application of the file.
"""

import argparse
import array
import base64
import json
import math
import os
import queue
import select as terminal_select
import shutil
import stat
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

ROBOTS = {"hydrus", "proteus", "bluerov2", "girona500"}
CAMERAS = tuple(f"Camera {n}" for n in range(1, 6))
LIMITS = {
    "/steps_per_second": (0, None, False),
    "/realtime_factor_cap": (0, None, False),
    "/tracking_ok_seconds": (0, None, False),
    "/min_render_interval_ms": (0, None, True),
    "/graphics/sun/azimuth_deg": (-360, 360, False),
    "/graphics/sun/elevation_deg": (-90, 90, False),
    "/graphics/ocean/jerlov": (0, 1, False),
    "/graphics/ocean/wave_height": (0, 2, False),
    "/graphics/viewer_exposure_ev": (-20, 20, False),
    "/graphics/camera_exposure_ev": (-20, 20, False),
    "/graphics/rtx/samples_per_pixel": (1, 4096, True),
    "/graphics/rtx/max_bounces": (0, 64, True),
}
TOGGLES = {
    "/bbox_only_in_front_of_camera", "/graphics/ocean/render_enabled",
    "/graphics/ocean/particles", "/graphics/raster/anti_aliasing",
    "/graphics/raster/ambient_occlusion", "/graphics/raster/screen_space_reflections",
    "/graphics/rtx/enabled", "/graphics/rtx/dlss", "/graphics/rtx/debug_albedo",
}
ALIASES = {
    "/view_camera": "active scene camera/viewpoint; selección de cámara y punto de vista; Camera 1 through Camera 5",
    "/realtime_factor_cap": "simulation speed cap, real-time factor; velocidad de simulación, factor de tiempo real",
    "/steps_per_second": "physics steps per second, physics frequency",
    "/min_render_interval_ms": "minimum render interval in milliseconds",
}
OPERATIONS = {
    "none": "No single clear request to read or change one setting; includes unrelated discussion, no-change requests, and unresolved alternatives.",
    "set": "Assign the requested value or select a camera. Both enable and disable (on/off, true/false) are set.",
    "get": "Read a setting without changing it.",
    "toggle": "Flip the current boolean value without specifying on or off.",
    "increase": "Add an explicit amount, or increase by an explicit percentage.",
    "decrease": "Subtract an explicit amount, or decrease by an explicit percentage.",
    "multiply": "Multiply by an explicit factor; double means 2, halve means 0.5.",
}

# The score question is an ordered rubric, so keep its grid small and adapt its
# range to the selected JSON field. A score's probability-weighted index maps back
# to a numeric estimate; it is an experiment, not exact number transcription.
SCORE_LEVELS = 10
REALTIME_FACTOR_BUCKETS = (0.5, 1, 1.5, 2, 2.5, 3, 4, 5, 10)
REALTIME_FACTOR_SPOKEN = {
    "0.5": "zero point five, point five, half, cero coma cinco, medio",
    "1": "one, uno",
    "1.5": "one point five, one and a half, uno coma cinco",
    "2": "two, dos",
    "2.5": "two point five, dos coma cinco",
    "3": "three, tres",
    "4": "four, cuatro",
    "5": "five, cinco",
    "10": "ten, diez",
}
UNBOUNDED_SCORE_RANGES = {
    "/steps_per_second": (0, 720),
    "/realtime_factor_cap": (0, 10),
    "/tracking_ok_seconds": (0, 1_000_000),
    "/min_render_interval_ms": (0, 200),
}
SCORE_PADDING_FRACTION = {"/realtime_factor_cap": 0.01}


def loads(text):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    def constant(value):
        raise ValueError(f"Invalid JSON number: {value}")

    return json.loads(text, object_pairs_hook=pairs, parse_constant=constant)


def leaves(tree, path=()):
    if isinstance(tree, dict):
        for key, value in tree.items():
            yield from leaves(value, path + (key,))
    elif isinstance(tree, list):
        for index, value in enumerate(tree):
            yield from leaves(value, path + (index,))
    else:
        yield path, tree


def pointer(path):
    return "/" + "/".join(str(p).replace("~", "~0").replace("/", "~1") for p in path)


def lookup(tree, path):
    for key in path:
        tree = tree[key]
    return tree


@dataclass(frozen=True)
class Control:
    file: str
    path: tuple
    kind: str
    description: str

    @property
    def id(self):
        return self.file + ":" + pointer(self.path)


def controls(repo, controller=None):
    folder = repo / "auvs/stonefish_sim"
    platform = loads((folder / "platform.json").read_text())
    robot = controller or platform["robot"]
    if robot not in ROBOTS:
        raise ValueError("Unsupported controller")
    result = {}
    for file, tree in (("platform", platform), (robot, loads((folder / f"{robot}.json").read_text()))):
        for path, value in leaves(tree):
            p = pointer(path)
            description = f"{file}: {p}"
            if file == "platform":
                if p == "/view_camera":
                    kind = "camera"
                elif p == "/graphics/rtx/dlss_quality":
                    kind = "quality"
                elif p in TOGGLES:
                    kind = "bool"
                elif p in LIMITS or (p.startswith("/graphics/looks/") and path[-1] in ("roughness", "metalness", "reflectivity")):
                    kind = "number"
                elif p.startswith("/graphics/looks/") and len(path) == 5 and path[-2] == "color" and isinstance(path[-1], int):
                    kind = "number"
                else:
                    continue
                description += " " + ALIASES.get(p, "")
            else:
                if not isinstance(value, (int, float)) or isinstance(value, bool):
                    continue
                if path[0] not in ("tam", "odometry", "non_odometry"):
                    continue
                kind = "number"
                if path[0] == "tam" and len(path) == 3:
                    description += f" thruster matrix row {path[1] + 1}, column {path[2] + 1}"
                elif len(path) == 3 and isinstance(path[-1], int):
                    description += f" {path[0]} {path[1]} item {path[-1] + 1} (spoken ordinal, index {path[-1]})"
            control = Control(file, path, kind, description.strip())
            result[control.id] = control
    return result


def decision_payload(transcript, available, two_step=False):
    # The original transcript is the model's evidence. Never rewrite its wording.
    return {
        "model": "gpt-6-luna", "input": transcript,
        "questions": [
            {"type": "choice", "name": "target", "instructions":
             "Which one supplied setting does the user want to read or change? Interpret the original Spanish, "
             "English or mixed-language request, including paraphrases and polite requests. Select the setting, "
             "not its value. Scene camera numbers refer to view_camera. Real-time factor controls simulation "
             "speed. Short selections such as Cámara dos refer to view_camera even without a verb. "
             "Use spoken ordinals for controller items (third means index 2), but do not invent axes "
             "or missing indices. Select none for an unsupported or ambiguous field or multiple settings.",
             "choices": [{"value": "none", "description": "No single supported field"}] +
                        [{"value": c.id, "description": c.description} for c in available.values()]},
            {"type": "choice", "name": "operation", "instructions":
             "Interpret the requested operation semantically in Spanish, English or mixed language. "
             "An absolute desired value is set, even when below one; reducing TO 0.5 is set, reducing BY "
             "10 percent is decrease. Enabling/disabling or assigning true/false is set. A polite request "
             "to see a numbered camera is set (switch viewpoint); get only asks for the current setting value. "
            "A short camera name/number alone also means set. A negated preference followed by an instruction "
            "to disable a feature means set(false), not a request to make no change. "
            "can change a value. A request to flip a boolean without specifying on/off is toggle. "
            "Use get for reading the current setting, multiply for scaling/doubling/halving. "
            "Choose none if there is no single clear operation, the user says to leave the setting unchanged, "
             "or the request is unrelated. The selected target and operation together determine whether this is actionable. " +
             ("In this two-step interaction, the user may intentionally omit a numeric value because they will say it next. "
              "For an explicit command such as 'set real-time factor', choose set even though the value is not present yet; "
              "do not choose none just because the number is deferred." if two_step else ""),
             "choices": [{"value": op, "description": desc} for op, desc in OPERATIONS.items()]},
            {"type": "choice", "name": "camera_value", "instructions":
             "If the user requests switching the scene viewpoint to one camera, which camera is the final "
             "requested value? Understand numbers and ordinals in Spanish/English, synonyms for switching "
             "views, and clear final corrections. A bare camera name/number is also a switch request. "
             "Choose none if no camera selection, an unsupported camera, "
             "or unresolved alternatives. Camera numbers are not numeric controller arguments.",
             "choices": [{"value": name, "description": f"Scene viewpoint/camera number {i}; cámara {i}"}
                         for i, name in enumerate(CAMERAS, 1)] + [{"value": "none", "description": "No clear supported camera value"}]},
            {"type": "choice", "name": "toggle_value", "instructions":
             "If the user explicitly requests enabling or disabling one boolean setting, what final value "
             "is requested? Interpret Spanish, English and paraphrases: enable/on is true; disable/off is false. "
             "Choose none for no explicit boolean value, a request merely to flip the current state, "
             "a no-change request, or unresolved alternatives.",
             "choices": [{"value": True, "description": "Enable the setting"},
                         {"value": False, "description": "Disable the setting"},
                         {"value": "none", "description": "No explicit boolean value"}]},
            {"type": "choice", "name": "quality_value", "instructions":
             "Which DLSS quality preset is explicitly requested? Interpret Spanish and English "
             "names for these presets. Choose none if no DLSS preset is requested or it is ambiguous.",
             "choices": [{"value": x, "description": f"DLSS {x} preset"}
                         for x in ("quality", "balanced", "performance", "ultra", "dlaa")] +
                        [{"value": "none", "description": "No explicit DLSS quality preset"}]},
        ],
    }


def answers_by_name(response):
    answers = response.get("answers")
    if not isinstance(answers, list) or any(not isinstance(a, dict) for a in answers):
        raise ValueError("Malformed Decisions response; no JSON change")
    named = {a.get("name"): a for a in answers}
    if len(named) != len(answers):
        raise ValueError("Duplicate Decisions answer names; no JSON change")
    return named


def probability(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and 0 <= value <= 1


def choice(response, name, allowed, minimum):
    answer = answers_by_name(response).get(name, {})
    if answer.get("type") != "choice":
        raise ValueError(f"Decisions did not return a usable {name} choice")
    confidence = answer.get("confidence")
    if not probability(confidence) or confidence < minimum:
        raise ValueError(f"Decision uncertain for {name}: choice={answer.get('choice')!r}, confidence={confidence!r}, required>={minimum:g}; no JSON change")
    value = answer.get("choice")
    # Decisions distinguishes boolean values from strings (and integers).
    if not any(type(value) is type(candidate) and value == candidate for candidate in allowed):
        raise ValueError(f"Unknown {name} choice; no JSON change")
    return value


def select(response, available, command_minimum, value_minimum):
    target = choice(response, "target", ["none", *available], command_minimum)
    if target == "none":
        return None, "ignore"
    control = available[target]
    if control.kind in ("camera", "quality"):
        # A confident discrete target/value pair is a selection command. These
        # short forms ("camera two", a preset name) intentionally omit a verb.
        return control, "set"
    allowed_operations = {
        "bool": ["get", "set", "toggle", "none"],
        "number": ["get", "set", "increase", "decrease", "multiply", "none"],
    }[control.kind]
    operation = choice(response, "operation", allowed_operations, command_minimum)
    if operation == "none":
        return None, "ignore"
    return control, operation


@dataclass(frozen=True)
class Argument:
    value: object
    percent: bool = False
    confidence: float | None = None
    grid_step: float | None = None


@dataclass(frozen=True)
class PendingCommand:
    transcript: str
    decision: dict
    available: dict
    control: Control
    operation: str
    started: float
    decided: float


def decision_argument(response, control, minimum):
    name, allowed = {
        "camera": ("camera_value", [*CAMERAS, "none"]),
        "bool": ("toggle_value", [True, False, "none"]),
        "quality": ("quality_value", ["quality", "balanced", "performance", "ultra", "dlaa", "none"]),
    }[control.kind]
    value = choice(response, name, allowed, minimum)
    if value == "none":
        raise ValueError("Specify one supported value for the selected setting")
    return Argument(value)


def numeric_range(control, repo):
    """Return the numeric domain for one field's ordered score scale."""
    path = pointer(control.path)
    if control.file == "platform":
        low, high, integer = LIMITS.get(path, (None, None, False))
        if path.startswith("/graphics/looks/"):
            low, high = 0, 1
        if low is not None and high is not None:
            return low, high, integer
        if path in UNBOUNDED_SCORE_RANGES:
            low, high = UNBOUNDED_SCORE_RANGES[path]
            return low, high, integer
    elif path.startswith("/tam/"):
        return -2, 2, False
    elif path.endswith("/center_deadband"):
        return 0, 1, False
    elif path.endswith("/target_height"):
        return 0, 10, False
    elif path.startswith(("/odometry/", "/non_odometry/")):
        return -20, 20, False
    raise ValueError(f"No Decisions score range configured for {control.id}")


def score_levels(low, high, integer=False):
    """Create a bounded, uniformly spaced ordered rubric for numeric scoring."""
    if not math.isfinite(low) or not math.isfinite(high) or high <= low:
        raise ValueError("Invalid numeric score range")
    count = min(SCORE_LEVELS, int(high - low) + 1) if integer else SCORE_LEVELS
    if count < 2:
        count = 2
    step = (high - low) / (count - 1)
    values = [low + index * step for index in range(count)]
    if integer:
        values = [round(value) for value in values]
        # Integer grids must stay distinct and ordered; use a coarser regular grid
        # when the allowed domain is wider than the API rubric.
        values = list(dict.fromkeys(values))
    return values


def numeric_score_payload(transcript, control, operation, repo, value_only=False):
    low, high, integer = numeric_range(control, repo)
    # Pad bounded config ranges so a spoken out-of-range value lands outside the
    # legal domain and is rejected by validate(), rather than clipping to a limit.
    padding = max((high - low) * SCORE_PADDING_FRACTION.get(pointer(control.path), 0.05), 1e-6)
    low, high = low - padding, high + padding
    values = score_levels(low, high, integer)
    percent_choices = [format(index / 2, ".1f") for index in range(201)] + ["none"]

    def levels_for(grid):
        return [{"label": format(value, ".12g")} for value in grid]

    if control.file == "platform" and pointer(control.path) == "/realtime_factor_cap":
        bucket_choices = [format(value, ".12g") for value in REALTIME_FACTOR_BUCKETS]
        numeric_argument_question = {
            "type": "choice", "name": "numeric_argument",
            "instructions":
                f"Choose the exact real-time-factor value the user requested for operation {operation}. "
                "Interpret English or Spanish number words and decimal commas. Choose a listed number only "
                "when it matches the requested value; do not round to a nearby preset. Choose other when "
                "the requested numeric value is not exactly one of the listed presets. " +
                ("This is the user's second-turn answer to a prompt asking for the value; a bare number word "
                 "such as 'three' is a complete value response." if value_only else ""),
            "choices": [{"value": value,
                         "description": f"Set/use exactly {value}; spoken forms: {REALTIME_FACTOR_SPOKEN[value]}"}
                        for value in bucket_choices] +
                       [{"value": "other", "description": "The requested value is not one of these exact presets"}],
        }
        numeric_values = REALTIME_FACTOR_BUCKETS
    else:
        numeric_argument_question = {
            "type": "score", "name": "numeric_argument", "instructions":
             f"Score the numeric argument stated by the user for setting {control.id} ({control.description}) "
             f"and operation {operation}. This is numeric extraction, not a severity judgment. Choose the "
             "ordered level matching the spoken numeric value. Interpret Spanish and English number words, "
             "decimal commas/points, and fractions. For set, score the desired final value. For increase or "
             "decrease, score the stated nonnegative amount. For multiply, double is 2 and half speed is 0.5. "
             "Do not calculate against the current JSON value. If the value falls outside the listed scale, "
             "use the nearest endpoint; the application will reject endpoint estimates.",
             "levels": levels_for(values),
        }
        numeric_values = values

    return {
        "model": "gpt-6-luna", "input": transcript,
        "questions": [
            {"type": "predicate", "name": "argument_ready", "instructions":
             "Is there exactly one clear numeric argument for the proposed setting and operation in this "
             "Spanish, English, or mixed-language request? Interpret number words, decimal commas/points, "
             "fractions, and polite wording. False for missing or ambiguous values, multiple edits, a "
             "no-change request, or unrelated text. Controller ordinals identify the setting, not the value. " +
             ("The input is a prompted second-turn value response; a bare number word counts as a complete argument."
              if value_only else "")},
            {"type": "choice", "name": "numeric_unit", "instructions":
             "Does the requested numeric argument mean a percentage change, an absolute amount/value, or is "
             "its unit unclear? For set and multiply, the argument must be absolute. Choose none if no "
             "numeric argument is clear. " +
             ("This is a prompted second-turn number response; interpret it as the requested value unless it "
              "explicitly says percent." if value_only else ""),
             "choices": [{"value": "absolute", "description": "An absolute value, amount, or multiplication factor."},
                         {"value": "percent", "description": "An explicit percentage for an increase or decrease."},
                         {"value": "none", "description": "No clear numeric argument or unit."}]},
            numeric_argument_question,
            {"type": "choice", "name": "numeric_percent_value", "instructions":
             "If the user explicitly requests an increase or decrease by a percentage, choose the exact "
             "percentage amount from these options. Understand Spanish and English number words and decimal "
             "commas. Choose none unless the utterance clearly states a percentage matching one listed value.",
             "choices": [{"value": value, "description": f"{value} percent"}
                         for value in percent_choices]},
        ],
    }, numeric_values, percent_choices


def scored_argument(response, values, percent_choices, operation, minimum):
    answers = answers_by_name(response)
    answer = answers.get("numeric_argument", {})
    if answer.get("type") == "choice":
        bucket_choices = [format(value, ".12g") for value in REALTIME_FACTOR_BUCKETS]
        selected = choice(response, "numeric_argument", [*bucket_choices, "other"], minimum)
        if selected == "other":
            raise ValueError("Use one of the real-time-factor presets: " +
                             ", ".join(bucket_choices) + "; no JSON change")
        return Argument(float(selected), False, answer["confidence"], None)
    ready = answers.get("argument_ready", {})
    if ready.get("type") != "predicate" or not probability(ready.get("probability")) or ready["probability"] < minimum:
        raise ValueError("No single clear numeric argument; no JSON change")
    unit = choice(response, "numeric_unit", ["absolute", "percent", "none"], minimum)
    if unit == "none":
        raise ValueError("Specify one clear numeric argument")
    percent = unit == "percent"
    if percent and operation not in ("increase", "decrease"):
        raise ValueError("Percentages only work with increase or decrease")
    if percent:
        selected = choice(response, "numeric_percent_value", percent_choices, minimum)
        if selected == "none":
            raise ValueError("Percentage does not match a supported half-percent step")
        return Argument(float(selected), True, answers["numeric_percent_value"]["confidence"], 0.5)
    grid = values
    if answer.get("type") != "score":
        raise ValueError("Decisions did not return a numeric score")
    confidence, score = answer.get("confidence"), answer.get("score")
    if not probability(confidence) or confidence < minimum:
        raise ValueError(f"Decision uncertain for numeric_argument: confidence={confidence!r}; no JSON change")
    if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score) or not 0 <= score <= len(grid) - 1:
        raise ValueError("Invalid numeric score")
    if score <= 0.5 or score >= len(grid) - 1.5:
        raise ValueError("Numeric value is outside the score scale; no JSON change")
    # The response exposes probability per ordered level. Weight the mapped values
    # directly so integer grids remain accurate even when spacing varies.
    probabilities = answer.get("probabilities")
    if not isinstance(probabilities, list) or len(probabilities) != len(grid):
        raise ValueError("Decisions omitted the numeric score distribution")
    weighted, total = 0.0, 0.0
    seen = set()
    for item in probabilities:
        if not isinstance(item, dict):
            raise ValueError("Invalid numeric score distribution")
        index, chance = item.get("value"), item.get("probability")
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(grid) or not probability(chance):
            raise ValueError("Invalid numeric score distribution")
        if index in seen:
            raise ValueError("Duplicate numeric score level")
        seen.add(index)
        weighted += grid[index] * chance
        total += chance
    if total <= 0 or len(seen) != len(grid):
        raise ValueError("Empty numeric score distribution")
    value = weighted / total
    if all(isinstance(v, int) for v in grid):
        value = round(value)
    step = max((grid[i + 1] - grid[i] for i in range(len(grid) - 1)), default=0)
    return Argument(value, False, confidence, step)


def replace_value(text, path, value):
    # Locate a JSON value span with the decoder, preserving every other byte.
    decoder = json.JSONDecoder()
    hits = []

    def skip(i):
        while i < len(text) and text[i].isspace():
            i += 1
        return i

    def walk(i, current):
        i = skip(i)
        if current == path:
            _, end = decoder.raw_decode(text, i)
            hits.append((i, end))
            return end
        if text[i] not in "{[":
            return decoder.raw_decode(text, i)[1]
        object_ = text[i] == "{"
        close = "}" if object_ else "]"
        i, index = skip(i + 1), 0
        while text[i] != close:
            if object_:
                key, i = decoder.raw_decode(text, i)
                i = skip(i)
                if text[i] != ":":
                    raise ValueError("Invalid JSON object")
                i += 1
            else:
                key = index
            i = skip(walk(i, current + (key,)))
            index += 1
            if text[i] == ",":
                i = skip(i + 1)
            elif text[i] != close:
                raise ValueError("Invalid JSON separator")
        return i + 1

    loads(text)
    walk(0, ())
    if len(hits) != 1:
        raise ValueError("Field is missing or ambiguous")
    start, end = hits[0]
    updated = text[:start] + json.dumps(value, allow_nan=False) + text[end:]
    loads(updated)
    return updated


def validate(control, value):
    if control.kind == "camera":
        if value not in CAMERAS:
            raise ValueError("Unknown camera")
    elif control.kind == "bool":
        if not isinstance(value, bool):
            raise ValueError("Expected a toggle")
    elif control.kind == "quality":
        if value not in ("quality", "balanced", "performance", "ultra", "dlaa"):
            raise ValueError("Unknown DLSS quality")
    else:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("Expected a finite number")
        p = pointer(control.path)
        lo, hi, integer = LIMITS.get(p, (None, None, False))
        if p.startswith("/graphics/looks/"):
            lo, hi = 0, 1
        if control.file != "platform":
            lo, hi = -3.4028234663852886e38, 3.4028234663852886e38
        if (lo is not None and value < lo) or (hi is not None and value > hi):
            raise ValueError(f"Value outside allowed range {lo}..{hi}")
        if p == "/steps_per_second" and value <= 0:
            raise ValueError("Physics frequency must be positive")
        if integer:
            if not float(value).is_integer():
                raise ValueError("Expected an integer")
            value = int(value)
    return value


def apply(repo, control, operation, argument=None, dry_run=False):
    path = repo / "auvs/stonefish_sim" / f"{control.file}.json"
    original = path.read_text()
    old = lookup(loads(original), control.path)
    result = {"file": control.file, "path": pointer(control.path), "old": old}
    if operation == "get":
        return dict(result, status="read")
    if operation == "toggle":
        if control.kind != "bool" or not isinstance(old, bool):
            raise ValueError("Toggle requires a boolean setting")
        value, percent = not old, False
    elif isinstance(argument, Argument) and type(argument.percent) is bool:
        value, percent = argument.value, argument.percent
    else:
        raise ValueError("Missing typed argument")
    if control.kind != "number" and operation not in ("set", "toggle"):
        raise ValueError("This field only supports set or get")
    if percent and operation not in ("increase", "decrease"):
        raise ValueError("Percentages require increase or decrease")
    if operation not in ("set", "toggle"):
        if not isinstance(old, (int, float)) or isinstance(old, bool):
            raise ValueError("Relative edits require an existing number")
        if operation in ("increase", "decrease"):
            if value < 0:
                raise ValueError("Say a nonnegative increase/decrease amount")
            delta = old * value / 100 if percent else value
            value = old + delta if operation == "increase" else old - delta
        elif operation == "multiply":
            value = old * value
        else:
            raise ValueError("Unsupported operation")
    value = validate(control, value)
    updated = replace_value(original, control.path, value)
    if not dry_run and value != old:
        fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=".voice-", suffix=".json")
        try:
            with os.fdopen(fd, "w") as stream:
                stream.write(updated)
            os.chmod(temporary, stat.S_IMODE(path.stat().st_mode))
            if path.read_text() != original:
                raise ValueError("Config changed during edit; please repeat the command")
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return dict(result, value=value, status="dry-run" if dry_run else "unchanged" if value == old else "saved")


def short_control_name(control):
    names = {
        "/realtime_factor_cap": "real-time factor",
        "/steps_per_second": "physics steps per second",
        "/view_camera": "camera",
    }
    path = pointer(control.path)
    return names.get(path, path.rsplit("/", 1)[-1].replace("_", " "))


def process(transcript, args, client, speech_end=None, *, defer_numeric=False,
            pending=None, argument_transcript=None):
    if pending is None:
        print(f"Heard: {transcript}", flush=True)
        available = controls(args.repo, args.controller)
        start = time.perf_counter()
        response = client.post("https://api.openai.com/v1/decisions",
                               json=decision_payload(transcript, available, args.two_step))
        response.raise_for_status()
        decision = response.json()
        if getattr(args, "debug_decisions", False):
            summaries = []
            for answer in decision.get("answers", []):
                summary = {key: answer.get(key) for key in ("name", "type", "choice", "confidence", "probability")}
                summary["top_choices"] = sorted(answer.get("probabilities", []), key=lambda p: p.get("probability", 0), reverse=True)[:3]
                summaries.append(summary)
            print("Decisions: " + json.dumps(summaries), flush=True)
        chosen, operation = select(decision, available, args.min_command_confidence,
                                   args.min_confidence)
        decided = time.perf_counter()
    else:
        available, decision = pending.available, pending.decision
        chosen, operation = pending.control, pending.operation
        start, decided = pending.started, pending.decided
        transcript = pending.transcript
        if argument_transcript is None:
            raise ValueError("The two-step command needs a value; no JSON change")
        print(f"Heard value: {argument_transcript}", flush=True)
    argument_started = time.perf_counter() if pending is not None else decided
    if chosen is None:
        print("Ignored: no single supported command.", flush=True)
        return
    if defer_numeric and operation not in ("get", "toggle") and chosen.kind == "number":
        return PendingCommand(transcript, decision, available, chosen, operation, start, decided)
    argument = None
    source = "read" if operation == "get" else "local_toggle" if operation == "toggle" else "decisions"
    if operation not in ("get", "toggle"):
        if chosen.kind == "number":
            source = ("decisions_buckets" if chosen.file == "platform" and
                      pointer(chosen.path) == "/realtime_factor_cap" else "decisions_score")
            numeric_transcript = argument_transcript or transcript
            if pending is not None:
                numeric_transcript = (f"The command was: {transcript}. "
                                      f"The user responded with this value: {argument_transcript}.")
            payload, values, percent_values = numeric_score_payload(
                numeric_transcript, chosen, operation, args.repo,
                value_only=pending is not None)
            response = client.post("https://api.openai.com/v1/decisions", json=payload)
            response.raise_for_status()
            score_decision = response.json()
            if getattr(args, "debug_decisions", False):
                details = []
                for answer in score_decision.get("answers", []):
                    summary = {key: answer.get(key) for key in ("name", "type", "score", "confidence", "probability")}
                    summary["top_levels"] = sorted(answer.get("probabilities", []),
                                                   key=lambda p: p.get("probability", 0), reverse=True)[:3]
                    details.append(summary)
                print("Numeric Decisions: " + json.dumps(details), flush=True)
            argument = scored_argument(score_decision, values, percent_values,
                                        operation, args.min_confidence)
        else:
            argument = decision_argument(decision, chosen, args.min_confidence)
    extracted = time.perf_counter()
    if getattr(args, "debug_decisions", False) and argument is not None:
        print("Argument: " + json.dumps({"source": source, "value": argument.value,
                                        "percent": argument.percent,
                                        "score_confidence": argument.confidence,
                                        "grid_step": argument.grid_step}), flush=True)
    # Reread controller selection before writing: don't act on a stale vehicle.
    if chosen.id not in controls(args.repo, args.controller):
        raise ValueError("Active controller changed; repeat the command")
    result = apply(args.repo, chosen, operation, argument, args.dry_run)
    end = time.perf_counter()
    result["decision_ms"] = round((decided - start) * 1000)
    result["argument_source"] = source
    if argument is not None and argument.confidence is not None:
        result["argument_confidence"] = argument.confidence
        if argument.grid_step is not None:
            result["score_grid_step"] = argument.grid_step
    result["extraction_ms"] = round((extracted - argument_started) * 1000)
    result["validate_write_ms"] = round((end - extracted) * 1000)
    result["command_ms"] = round((end - start) * 1000)
    if speech_end is not None:
        result["speech_to_result_ms"] = round((end - speech_end) * 1000)
    print(json.dumps(result), flush=True)  # "saved" confirms the file, not simulator application.


class Transcriber:
    def __init__(self, key):
        import websocket

        self.events = queue.Queue()
        self.stopped = threading.Event()
        self.ws = websocket.create_connection(
            "wss://api.openai.com/v1/realtime?intent=transcription",
            header=[f"Authorization: Bearer {key}"], timeout=10,
        )
        self.ws.settimeout(1)
        self.reader = threading.Thread(target=self.receive, daemon=True)
        self.reader.start()
        try:
            self.send({"type": "session.update", "session": {
                "type": "transcription", "audio": {"input": {
                    "format": {"type": "audio/pcm", "rate": 24000},
                    "transcription": {"model": "gpt-live-transcribe", "languages": ["es", "en"], "delay": "minimal",
                                      "keywords": ["camera", "cámara", "real-time factor", "factor de tiempo real", "odometry", "odometría", "KP", "KI", "KD"]},
                    "turn_detection": None,
                }},
            }})
            deadline = time.perf_counter() + 15
            while self.event(deadline).get("type") != "session.updated":
                pass
        except BaseException:
            self.close()
            raise

    def receive(self):
        import websocket

        while not self.stopped.is_set():
            try:
                raw = self.ws.recv()
                if not raw:
                    raise RuntimeError("Transcription connection closed")
                event = json.loads(raw)
                if event.get("type") != "conversation.item.input_audio_transcription.delta":
                    self.events.put(event)
            except websocket.WebSocketTimeoutException:
                continue
            except Exception as exc:
                if not self.stopped.is_set():
                    self.events.put({"type": "error", "error": {"message": str(exc)}})
                return

    def send(self, event):
        self.ws.send(json.dumps(event))

    def event(self, deadline):
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            raise TimeoutError("Transcription timed out; no JSON change")
        try:
            event = self.events.get(timeout=remaining)
        except queue.Empty:
            raise TimeoutError("Transcription timed out; no JSON change") from None
        if event.get("type") in ("error", "conversation.item.input_audio_transcription.failed"):
            raise RuntimeError(event.get("error", {}).get("message", "Transcription failed"))
        return event

    def append(self, pcm):
        self.send({"type": "input_audio_buffer.append", "audio": base64.b64encode(pcm).decode()})

    def finish(self):
        self.send({"type": "input_audio_buffer.commit"})
        deadline = time.perf_counter() + 15
        item = None
        completed = {}
        while True:
            event = self.event(deadline)
            if event.get("type") == "input_audio_buffer.committed":
                item = event["item_id"]
            elif event.get("type") == "conversation.item.input_audio_transcription.completed":
                completed[event["item_id"]] = event.get("transcript", "")
            if item in completed:
                return completed[item]

    def close(self):
        self.stopped.set()
        self.ws.close()


class Microphone:
    """Use the desktop's native audio capture, emitting 20 ms PCM16 chunks."""

    def __init__(self, device=None):
        if not shutil.which("parec"):
            raise RuntimeError("Install pulseaudio-utils to provide parec microphone capture")
        self.audio = queue.Queue(maxsize=500)
        self.overflow = threading.Event()
        self.closed = threading.Event()
        self.failure = None
        command = ["parec", "--raw", "--format=s16le", "--rate=24000", "--channels=1", "--latency-msec=20"]
        if device:
            command += ["--device", device]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.reader = threading.Thread(target=self.capture, daemon=True)
        self.reader.start()

    def capture(self):
        pending = bytearray()
        try:
            while not self.closed.is_set():
                chunk = self.process.stdout.read(960 - len(pending))
                if not chunk:
                    raise RuntimeError("Microphone stopped: " + self.process.stderr.read().decode(errors="replace").strip()[:300])
                pending.extend(chunk)
                if len(pending) == 960:
                    try:
                        self.audio.put_nowait((bytes(pending), time.perf_counter()))
                    except queue.Full:
                        self.overflow.set()
                    pending.clear()
        except Exception as exc:
            if not self.closed.is_set():
                self.failure = str(exc)

    def close(self):
        self.closed.set()
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self.reader.join(timeout=1)
        self.process.stdout.close()
        self.process.stderr.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class SendKey:
    """Read a single terminal key without Enter; always restore terminal settings."""

    def __init__(self, enabled):
        self.enabled = enabled
        self.settings = None

    def __enter__(self):
        if self.enabled:
            import termios
            import tty

            if not sys.stdin.isatty():
                raise ValueError("--push-to-send requires an interactive terminal")
            self.fd = sys.stdin.fileno()
            self.settings = termios.tcgetattr(self.fd)
            try:
                tty.setcbreak(self.fd, termios.TCSANOW)
            except BaseException:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.settings)
                raise
        return self

    def pressed(self):
        pressed = False
        if self.enabled:
            while terminal_select.select([self.fd], [], [], 0)[0]:
                key = os.read(self.fd, 1)
                if not key:
                    raise RuntimeError("Terminal input closed")
                pressed = pressed or key.lower() == b"s"
        return pressed

    def __exit__(self, *args):
        if self.settings is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSANOW, self.settings)


def capture_utterance(args, transcriber, microphone, send_key, prompt):
    audio, overflow = microphone.audio, microphone.overflow
    while not audio.empty():
        audio.get_nowait()
    overflow.clear()
    send_key.pressed()  # Discard keys pressed while the previous stage was processing.
    ending = "press s to send (no Enter needed)" if args.push_to_send else f"pause for {args.silence_ms / 1000:g} seconds"
    print(f"Listening for {prompt} — {ending}. Ctrl+C stops.", flush=True)
    prefix = deque(maxlen=10)
    active = False
    duration = voiced = silence = 0.0
    last_voice = None
    while True:
        if microphone.failure:
            raise RuntimeError(microphone.failure)
        if not transcriber.reader.is_alive():
            transcriber.event(time.perf_counter() + 1)
            raise RuntimeError("Transcription disconnected; restart the listener")
        if overflow.is_set():
            raise RuntimeError("Microphone audio overflow; no command executed")
        submit = send_key.pressed()
        if submit:
            if not active or voiced < 0.12:
                print("No speech captured yet; say it, then press s.", flush=True)
            else:
                transcript = transcriber.finish().strip()
                return (transcript, last_voice) if transcript else None
        try:
            pcm, timestamp = audio.get(timeout=0.05 if args.push_to_send else 1)
        except queue.Empty:
            continue
        if overflow.is_set():
            raise RuntimeError("Microphone audio overflow; no command executed")
        samples = array.array("h", pcm)
        if sys.byteorder != "little":
            samples.byteswap()
        rms = math.sqrt(sum(x * x for x in samples) / len(samples)) / 32768
        # Keep quieter words once speech starts; use a higher threshold
        # to start a command so background noise doesn't trigger it.
        loud = rms >= args.threshold * (0.5 if active else 1.0)
        if not active:
            prefix.append(pcm)
            if not loud:
                continue
            active = True
            for chunk in prefix:
                transcriber.append(chunk)
        else:
            transcriber.append(pcm)
        duration += 0.02
        if loud:
            voiced += 0.02
            last_voice = timestamp
            silence = 0.0
        else:
            silence += 0.02
        if duration >= (60 if args.push_to_send else 10):
            transcriber.send({"type": "input_audio_buffer.clear"})
            print("Command too long; no JSON change.", flush=True)
            return None
        if not args.push_to_send and silence >= args.silence_ms / 1000:
            if voiced < 0.12:
                transcriber.send({"type": "input_audio_buffer.clear"})
                return None
            transcript = transcriber.finish().strip()
            return (transcript, last_voice) if transcript else None


def listen(args, client, key):
    transcriber = Transcriber(key)
    try:
        with SendKey(args.push_to_send) as send_key, Microphone(args.device) as microphone:
            while True:
                first = capture_utterance(args, transcriber, microphone, send_key, "command")
                if first is None:
                    continue
                transcript, speech_end = first
                try:
                    pending = process(transcript, args, client, speech_end,
                                      defer_numeric=args.two_step)
                    if pending is None:
                        continue
                    print(f"Command understood: {pending.operation} {short_control_name(pending.control)}.", flush=True)
                    for attempt in range(3):
                        second = capture_utterance(args, transcriber, microphone, send_key, "the value")
                        if second is None:
                            print("No value captured; command cancelled without a JSON change.", flush=True)
                            break
                        value_transcript, value_end = second
                        try:
                            process(transcript, args, client, value_end, pending=pending,
                                    argument_transcript=value_transcript)
                            break
                        except ValueError as exc:
                            if attempt == 2:
                                print(f"No edit after three value attempts: {exc}", file=sys.stderr, flush=True)
                            else:
                                print(f"Value not accepted: {exc}. Please say the value again.", flush=True)
                except Exception as exc:
                    print(f"No edit: {exc}", file=sys.stderr, flush=True)
    finally:
        transcriber.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--controller", choices=sorted(ROBOTS), help="Default is the platform-selected robot")
    parser.add_argument("--text", help="Send a typed command through Decisions, without the microphone")
    parser.add_argument("--value", help="Second-step value for a typed --two-step command")
    parser.add_argument("--two-step", action="store_true",
                        help="For numeric edits, classify the command first, then listen for the value")
    parser.add_argument("--dry-run", action="store_true", help="Transcribe and decide, but do not save JSON")
    parser.add_argument("--controls", action="store_true", help="List allowed fields without API calls")
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--device", help="PulseAudio microphone source name; default is the desktop input")
    parser.add_argument("--push-to-send", action="store_true",
                        help="Speak, then press s to submit; disables automatic submission on silence")
    parser.add_argument("--silence-ms", type=int, default=1200,
                        help="Continuous silence before finishing a command, 100..5000 ms (default: 1200)")
    parser.add_argument("--threshold", type=float, default=0.015, help="Microphone RMS threshold, 0..1")
    parser.add_argument("--min-confidence", type=float, default=0.8)
    parser.add_argument("--min-command-confidence", type=float, default=0.5,
                        help="Confidence required for the selected field and operation (default: 0.5)")
    parser.add_argument("--debug-decisions", action="store_true",
                        help="Print selected choices, confidence, and the top three probabilities")
    args = parser.parse_args()
    if args.value is not None and (not args.two_step or args.text is None):
        parser.error("--value requires --two-step and --text")
    if args.push_to_send and args.text is not None:
        parser.error("--push-to-send is for microphone commands; omit --text")
    if args.push_to_send and not args.controls and not args.list_devices and not sys.stdin.isatty():
        parser.error("--push-to-send requires an interactive terminal")
    if not 100 <= args.silence_ms <= 5000 or not 0 < args.threshold < 1 or not 0 <= args.min_confidence <= 1 or not 0 <= args.min_command_confidence <= 1:
        parser.error("Invalid silence, threshold, or confidence setting")
    if args.list_devices:
        subprocess.run(["pactl", "list", "short", "sources"], check=True)
        return
    available = controls(args.repo, args.controller)
    if args.controls:
        for control in available.values():
            print(control.description)
        return
    key = os.getenv("OPENAI_API_KEY")
    if not key:
        parser.error("Set OPENAI_API_KEY in your shell before running the API demo")
    import httpx
    # Direct HTTP avoids requiring a newer OpenAI SDK just for Decisions.
    with httpx.Client(headers={"Authorization": f"Bearer {key}"}, timeout=15) as client:
        if args.text is not None:
            pending = process(args.text, args, client, defer_numeric=args.two_step)
            if isinstance(pending, PendingCommand):
                if args.value is None:
                    print("Command understood. Provide --value to finish this typed two-step test; no JSON change.")
                else:
                    process(args.text, args, client, pending=pending,
                            argument_transcript=args.value)
        else:
            listen(args, client, key)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)
