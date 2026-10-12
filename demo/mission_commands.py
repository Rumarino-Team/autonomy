"""UDP Motion/Navigation client and voice command extraction. No file edits.

Only the transcription and language interpretation use OpenAI. Raw commands,
handshakes, schema validation, retries, status and completion use local UDP.
"""
import copy
import json
import math
import queue
import secrets
import socket
import threading
import time

VERSION = 1
MAX_DATAGRAM = 60000
TERMINAL = {"completed", "failed", "cancelled"}
PLAN_COMMANDS = {"move_to", "move_relative", "follow_path", "move_to_object",
                 "follow_object", "orbit_object", "pass_gate", "look_at_object",
                 "look_at_point", "set_yaw", "set_attitude", "rotate", "search",
                 "visual_target", "body_move"}


def loads(text):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result
    def invalid(value):
        raise ValueError(f"Invalid number: {value}")
    return json.loads(text, object_pairs_hook=pairs, parse_constant=invalid)


class MissionClient:
    """One controller session, serialized request IDs and byte-identical retries.

    The receiver runs separately so completion events never get mistaken for
    acknowledgements. Heartbeats keep the session alive during transcription.
    Disconnect releases ownership; accepted autonomous commands continue.
    """
    def __init__(self, host="127.0.0.1", port=8080, timeout=0.5, retries=5):
        if not 1 <= port <= 65535 or timeout <= 0 or retries < 1:
            raise ValueError("Invalid UDP connection settings")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((host, port))
        self.socket.settimeout(0.2)
        self.timeout, self.retries = timeout, retries
        self.lock = threading.Lock()
        self.pending_lock = threading.Lock()
        self.pending = {}
        self.events = queue.Queue()
        self.stopped = threading.Event()
        self.disconnected = None
        self.next_id = 1
        self.session = None
        self.on_event = None
        self.last_status = {}
        try:
            nonce = secrets.token_hex(16)
            packet = self.encode({"version": VERSION, "kind": "hello", "id": 0, "nonce": nonce})
            welcome = None
            for _ in range(retries):
                self.socket.send(packet)
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    try:
                        reply = loads(self.socket.recv(MAX_DATAGRAM + 1))
                    except socket.timeout:
                        continue
                    if reply.get("version") != VERSION or reply.get("kind") != "welcome":
                        continue
                    if not reply.get("ok"):
                        raise RuntimeError("Handshake rejected: " + str(reply.get("error")))
                    if reply.get("data", {}).get("nonce") != nonce:
                        continue
                    welcome = reply
                    break
                if welcome:
                    break
            if welcome is None:
                raise TimeoutError("No handshake response. Start the teleop_command mission on the selected port.")
            self.session = welcome["session"]
            self.commands = effective_schema_catalog(resolve_references(welcome["data"]["commands"], welcome["data"]["definitions"]))
            self.last_status = welcome["data"]["status"]
            self.reader = threading.Thread(target=self.receive, daemon=True)
            self.heartbeat = threading.Thread(target=self.keepalive, daemon=True)
            self.reader.start()
            self.heartbeat.start()
        except BaseException:
            self.socket.close()
            raise

    @staticmethod
    def encode(packet):
        data = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()
        if len(data) > MAX_DATAGRAM:
            raise ValueError("Command exceeds the maximum UDP datagram size")
        return data

    def receive(self):
        while not self.stopped.is_set():
            try:
                raw = self.socket.recv(MAX_DATAGRAM + 1)
                if len(raw) > MAX_DATAGRAM:
                    continue
                message = loads(raw)
                if not isinstance(message, dict):
                    continue
                if message.get("version") != VERSION:
                    continue
                lost_session = message.get("error") == "HandshakeRequired"
                if message.get("session") != self.session and not lost_session:
                    continue
                if lost_session:
                    self.disconnected = "Server session lost; reconnect before sending another command"
                if isinstance(message.get("data"), dict) and "navigation" in message["data"]:
                    self.last_status = message["data"]
                if message.get("kind") == "event":
                    self.events.put(message)
                    if self.on_event is not None:
                        self.on_event(message)
                elif message.get("kind") == "reply":
                    with self.pending_lock:
                        inbox = self.pending.get(message.get("id"))
                    if inbox is not None:
                        inbox.put(message)
            except socket.timeout:
                continue
            except (ValueError, json.JSONDecodeError):
                continue
            except OSError as exc:
                if not self.stopped.is_set():
                    self.disconnected = str(exc)
                return

    def request(self, kind="request", command=None):
        with self.lock:
            if self.disconnected:
                raise RuntimeError(self.disconnected)
            request_id = self.next_id
            self.next_id += 1
            packet = {"version": VERSION, "kind": kind, "id": request_id, "session": self.session}
            if command is not None:
                packet["command"] = command
            data = self.encode(packet)
            inbox = queue.Queue()
            with self.pending_lock:
                self.pending[request_id] = inbox
            try:
                for _ in range(self.retries):
                    self.socket.send(data)
                    try:
                        reply = inbox.get(timeout=self.timeout)
                    except queue.Empty:
                        continue
                    if not reply.get("ok"):
                        raise ValueError("Command rejected: " + str(reply.get("error")))
                    return reply
                # An acknowledgement timeout does not prove nonexecution.
                raise TimeoutError(f"No acknowledgement for request {request_id}; it may have executed. Query status before repeating it.")
            finally:
                with self.pending_lock:
                    self.pending.pop(request_id, None)

    def keepalive(self):
        while not self.stopped.wait(2):
            try:
                self.request("heartbeat")
            except Exception as exc:
                self.disconnected = str(exc)
                return

    def send_command(self, command):
        validate_command(command, self.commands)
        return self.request(command=command)

    def status(self):
        return self.request(command={"status": {}})["data"]

    def wait(self, acknowledgement):
        state = acknowledgement.get("data") or {}
        operation = state.get("operation_id"), state.get("operation_session")
        if state.get("state") in TERMINAL:
            return state
        while True:
            state = self.status()
            if (state.get("operation_id"), state.get("operation_session")) != operation:
                return {"state": "cancelled", "reason": "Replaced by another command"}
            if state.get("state") in TERMINAL:
                return state
            time.sleep(0.2)

    def close(self):
        if not self.disconnected:
            try:
                self.request("bye")
            except Exception:
                pass
        self.stopped.set()
        self.socket.close()
        self.reader.join(timeout=1)
        self.heartbeat.join(timeout=1)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def resolve_references(value, definitions):
    if isinstance(value, dict):
        if "$ref" in value:
            prefix = "#/definitions/"
            reference = value["$ref"]
            if not reference.startswith(prefix) or reference[len(prefix):] not in definitions:
                raise ValueError("Unknown command schema reference")
            return resolve_references(definitions[reference[len(prefix):]], definitions)
        return {name: resolve_references(child, definitions) for name, child in value.items()}
    if isinstance(value, list):
        return [resolve_references(child, definitions) for child in value]
    return value


def effective_schema(schema, inherited=None):
    """Account for a parent struct's default when validating a partial child."""
    schema = copy.deepcopy(schema)
    if schema.get("type") == "object":
        defaults = dict(schema.get("defaults", {}))
        if isinstance(inherited, dict):
            defaults.update(inherited)
        schema["required"] = [key for key in schema.get("required", []) if key not in defaults]
        schema["properties"] = {name: effective_schema(child, defaults.get(name))
                                for name, child in schema.get("properties", {}).items()}
    if "anyOf" in schema:
        schema["anyOf"] = [effective_schema(child) for child in schema["anyOf"]]
    if "items" in schema:
        schema["items"] = effective_schema(schema["items"])
    return schema


def effective_schema_catalog(commands):
    result = copy.deepcopy(commands)
    for name, command in result.items():
        command["schema"] = effective_schema(command["schema"])
        if name in {"configure_motion", "configure_navigation"}:
            command["schema"]["properties"]["options"]["required"] = []
    return result


def validate(value, schema, path="args"):
    if "anyOf" in schema:
        for branch in schema["anyOf"]:
            try:
                validate(value, branch, path)
                return
            except ValueError:
                pass
        raise ValueError(f"{path}: value does not match an allowed variant")
    kind = schema.get("type")
    valid = {"object": isinstance(value, dict), "array": isinstance(value, list),
             "string": isinstance(value, str), "boolean": type(value) is bool,
             "integer": type(value) is int,
             "number": type(value) in (int, float) and math.isfinite(value),
             "null": value is None}
    if kind and not valid.get(kind, False):
        raise ValueError(f"{path}: expected {kind}")
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError(f"{path}: unsupported value")
    if kind in {"integer", "number"}:
        if not math.isfinite(value) or ("minimum" in schema and value < schema["minimum"]):
            raise ValueError(f"{path}: invalid number")
    if kind == "object":
        properties = schema.get("properties", {})
        if any(key not in value for key in schema.get("required", [])):
            raise ValueError(f"{path}: missing required argument")
        if schema.get("additionalProperties") is False and set(value) - set(properties):
            raise ValueError(f"{path}: unknown arguments {sorted(set(value) - set(properties))}")
        for name, child in value.items():
            if name in properties:
                validate(child, properties[name], path + "/" + name)
    if kind == "array":
        if len(value) < schema.get("minItems", 0) or len(value) > schema.get("maxItems", 100000):
            raise ValueError(f"{path}: invalid array length")
        for index, child in enumerate(value):
            validate(child, schema.get("items", {}), f"{path}/{index}")


def validate_command(command, commands):
    if not isinstance(command, dict) or len(command) != 1:
        raise ValueError("A command must be a single-key object, e.g. {\"search\":{}}")
    name, args = next(iter(command.items()))
    if name not in commands:
        raise ValueError(f"Unsupported command: {name}")
    validate(args, commands[name]["schema"])
    if name == "plan":
        steps = args["steps"]
        if not 1 <= len(steps) <= 32:
            raise ValueError("A plan must contain 1..32 movement steps")
        for index, step in enumerate(steps):
            if step["command"] not in PLAN_COMMANDS:
                raise ValueError("Plans may contain only movement commands")
            validate_command({step["command"]: step["args"]}, commands)
            if step["command"] == "follow_object" and index + 1 < len(steps) and step["args"].get("duration") is None:
                raise ValueError("Following needs a duration when another plan step comes after it")


def nullable(schema):
    if schema.get("type") == "null" or any(nullable(child) for child in schema.get("anyOf", [])):
        return True
    return False


def strict_schema(schema):
    """All model fields are required; null represents an omitted optional field."""
    schema = copy.deepcopy(schema)
    schema.pop("defaults", None)
    if "anyOf" in schema:
        schema["anyOf"] = [strict_schema(child) for child in schema["anyOf"]]
    if schema.get("type") == "object":
        required = set(schema.get("required", []))
        properties = {}
        for name, child in schema.get("properties", {}).items():
            child = strict_schema(child)
            if name not in required and not nullable(child):
                child = {"anyOf": [child, {"type": "null"}]}
            properties[name] = child
        schema["properties"] = properties
        schema["required"] = list(properties)
        schema["additionalProperties"] = False
    if "items" in schema:
        schema["items"] = strict_schema(schema["items"])
    return schema


def normalize(value, schema, clear_fields, path=""):
    if "anyOf" in schema:
        for branch in schema["anyOf"]:
            try:
                validate(value, strict_schema(branch))
                return normalize(value, branch, clear_fields, path)
            except ValueError:
                pass
        return value
    if isinstance(value, dict) and schema.get("type") == "object":
        result = {}
        for name, child in value.items():
            child_path = path + "/" + name
            original = schema["properties"][name]
            if child is None and name not in schema.get("required", []):
                if child_path not in clear_fields:
                    continue
                if not nullable(original):
                    raise ValueError(f"Cannot clear nonnullable field {child_path}")
            result[name] = normalize(child, original, clear_fields, child_path)
        return result
    if isinstance(value, list) and schema.get("type") == "array":
        return [normalize(child, schema["items"], clear_fields, path + f"/{index}") for index, child in enumerate(value)]
    return value


def extract(transcript, args, http, choose):
    connection = args.mission_client
    commands = connection.commands
    choice_response = http.post("https://api.openai.com/v1/decisions", json={
        "model": args.mission_model, "input": transcript,
        "questions": [{"type": "choice", "name": "command",
                       "instructions": "Select the user's requested submarine command in English, Spanish or mixed speech. "
                       "Select plan for an ordered sequence of multiple movement commands. Select none for ambiguous, "
                       "negated, unrelated speech or configuration-file edits. Do not reinterpret body force as distance. "
                       "look around is search; follow an object continuously is follow_object; image-only centering is visual_target.",
                       "choices": [{"value": "none", "description": "No clear supported command"}] +
                       [{"value": name, "description": command["description"]} for name, command in commands.items()]}]})
    choice_response.raise_for_status()
    decision = choice_response.json()
    name = choose(decision, "command", ["none", *commands], args.min_confidence)
    if name == "none":
        raise ValueError("No clear supported submarine instruction")
    if name == "plan":
        step_schemas = [{"type": "object", "properties": {key: item["schema"]},
                         "required": [key], "additionalProperties": False}
                        for key, item in commands.items() if key in PLAN_COMMANDS]
        schema = {"type": "object", "properties": {"steps": {"type": "array", "minItems": 1,
                  "maxItems": 32, "items": {"anyOf": step_schemas}}}, "required": ["steps"], "additionalProperties": False}
    else:
        schema = commands[name]["schema"]
    output_schema = {"type": "object", "additionalProperties": False,
                     "properties": {"ready": {"type": "boolean"}, "reason": {"type": "string"},
                                    "clear_fields": {"type": "array", "items": {"type": "string"}},
                                    "args": {"anyOf": [strict_schema(schema), {"type": "null"}]}},
                     "required": ["ready", "reason", "clear_fields", "args"]}
    # Refresh context after classification; the submarine may still be moving.
    state = connection.status()
    response = http.post("https://api.openai.com/v1/responses", json={
        "model": args.mission_model,
        "instructions": "Extract exact arguments for the selected submarine command. Never execute anything yourself. "
        "Use the user's original English/Spanish wording. Do not guess missing distances, axes, durations, object IDs, "
        "coordinates or rates. For ambiguity set ready=false, args=null and explain the missing information. "
        "Body +Y is forward, -Y backward, +X right/sideways, -X left, +Z down and -Z up; mission z is positive down. "
        "Distances are metres, durations seconds, angles radians, angular rates rad/s. Convert explicit degrees to radians. "
        "360 degrees is 2*pi, not zero. Cardinal directions require the user's mission-frame mapping; do not assume north. "
        "Facing is a single-key tagged object, e.g. {look_at_object:7} or {hold_current:{}}. Preserve unspecified options "
        "by returning null in optional fields, including entire move/options objects. Only explicitly requested options "
        "should be set. For explicit clearing of a nullable option return null AND add its JSON pointer to clear_fields "
        "(relative to args, e.g. /move/timeout). Object selectors may use class or a currently observed ID; never invent an ID. "
        "Nearest class selection is supported; if 'this object' is ambiguous, ask for an ID or class. Search durations and "
        "motion options have server defaults; they need not be invented. Only use allow_pitch when pitch may be released; "
        "fixed preserves requested attitude. Only use body_move for force effort and duration, not a measured distance. "
        "For visual_target use all VisualOptions; a supplied selector resolves options.object_id. Plans are ordered "
        "movements, with a duration for continuous following unless it is the final step.",
        "input": json.dumps({"transcript": transcript, "command": name,
                             "description": commands[name]["description"], "schema": schema,
                             "current_state": state}, allow_nan=False),
        "text": {"format": {"type": "json_schema", "name": "submarine_command", "strict": True, "schema": output_schema}},
    })
    response.raise_for_status()
    body = response.json()
    if body.get("status") != "completed":
        raise ValueError("Command extraction did not complete")
    texts = []
    for output in body.get("output", []):
        for item in output.get("content", []):
            if item.get("type") == "refusal":
                raise ValueError(item.get("refusal", "Command extraction refused"))
            if item.get("type") == "output_text":
                texts.append(item["text"])
    result = loads("".join(texts))
    validate(result, output_schema, "response")
    if not result["ready"] or result["args"] is None:
        raise ValueError(result["reason"] or "More information is needed")
    arguments = normalize(result["args"], schema, set(result["clear_fields"]))
    if name == "plan":
        arguments["steps"] = [{"command": next(iter(step)), "args": next(iter(step.values()))} for step in arguments["steps"]]
    command = {name: arguments}
    validate_command(command, commands)
    return command


def execute(command, args):
    validate_command(command, args.mission_client.commands)
    if args.dry_run:
        return {"dry_run": True, "command": command}
    acknowledgement = args.mission_client.send_command(command)
    result = {"accepted": True, "request_id": acknowledgement["id"], "command": command,
              "state": acknowledgement.get("data")}
    state = acknowledgement.get("data") or {}
    if args.wait and (state.get("operation_id"), state.get("operation_session")) == (acknowledgement["id"], args.mission_client.session):
        result["completion"] = args.mission_client.wait(acknowledgement)
    return result


def process(transcript, args, http, choose, speech_end=None):
    print(f"Heard: {transcript}", flush=True)
    started = time.perf_counter()
    command = extract(transcript, args, http, choose)
    result = execute(command, args)
    result["command_ms"] = round((time.perf_counter() - started) * 1000)
    if speech_end is not None:
        result["speech_to_result_ms"] = round((time.perf_counter() - speech_end) * 1000)
    print(json.dumps(result, allow_nan=False), flush=True)
