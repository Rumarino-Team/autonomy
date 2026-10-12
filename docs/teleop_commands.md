# Voice and UDP mission commands

`teleop_command` runs Motion commands while receiving new UDP instructions.
It exposes movement and Navigation APIs through a versioned JSON protocol.
The server binds `127.0.0.1` at `teleop_port` (default `8080`). The existing
`teleop` mission still accepts the keyboard sender's six-float pose packets.

## Run

From the repository root, build and start the selected Proteus simulator:

```sh
zig build
./zig-out/bin/src_3 ./zig-out/lib/libauv_stonefish_sim.so teleop_command ./auvs/stonefish_sim/proteus.json
```

The Stonefish plugin must already be built. In another terminal:

```sh
python demo/voice.py --mission --push-to-send
```

This uses the existing microphone/transcription setup and `OPENAI_API_KEY`.
Speak an instruction and press **s** in the terminal. English and Spanish are
supported. For example:

- “Move forward two metres while maintaining the heading.”
- “Follow the gate at a distance of two metres.”
- “Go around the rectangle while looking at it; allow up to thirty degrees of pitch.”
- “Look around for a cube for twenty seconds.”
- “Pitch three hundred sixty degrees, then roll three hundred sixty degrees.”
- “Pause the current plan.” / “Resume.” / “Stop.”
- “What is the current tracking state?”

Typed interpretation, raw commands and capability discovery:

```sh
python demo/voice.py --mission --text 'move forward two metres' --dry-run
python demo/voice.py --mission --text 'pitch 360 degrees, then roll 360 degrees' --wait
python demo/voice.py --command-json '{"search":{"duration":20}}'
python demo/voice.py --command-json '{"stop":{}}'
python demo/voice.py --command-json '{"status":{}}'
python demo/voice.py --mission --controls
```

`--command-json` and `--controls` require no OpenAI credentials. All mission
modes need a running command server for the handshake and current schemas.
Use `--port` when the running backend's configuration differs from the selected
Stonefish controller. `--dry-run` performs discovery/interpretation and prints
the validated command without sending a motion request. `--wait` is for typed
or raw commands; microphone mode stays available to interrupt a maneuver.

Without `--mission`, the existing settings-editing demo continues to edit
`platform.json` and the robot controller JSON. In mission mode, command
classification uses Decisions; exact arguments use the Responses API's
[structured outputs](https://developers.openai.com/api/docs/guides/structured-outputs?api-mode=responses).
Missing required information produces a clarification instead of an assumed
distance, axis or object ID. Numeric motion arguments do not use score buckets.

## Coordinate and unit conventions

Positions and distances are metres; durations are seconds. Angles are radians
and angular rates are radians/second. Voice interpretation converts explicit
degrees into radians. Raw JSON must already use radians.

Mission/world Z is positive down. Body **+Y is forward**, **+X is sideways**,
and **+Z is down**. Body-relative offsets are transformed using the orientation
when the command starts. Roll, pitch and yaw rotate about body X, Y and Z,
respectively, matching this controller's convention. Yaw zero faces world +Y;
yaw `-pi/2` faces world +X. Cardinal north/east directions require an explicit
mapping to the mission frame.

An object selector is `{"id":7}`, `{"class":"gate"}`, or both. Classes are
`gate`, `cube` and `rect`. World commands require fresh measured position and
object observations. A class selects the nearest currently observed 3D object.
Image-only selection requires a unique match, or an explicit ID.

## Command set

Each command is a single-key object, e.g. `{"search":{"duration":20}}`.
Omitted optional arguments use the server's defaults. Nested options can be
partial; parent defaults are retained. `--controls` prints the current resolved
schemas and defaults generated from the Zig types.

| Command | Arguments / behavior |
| --- | --- |
| `move_to` | `position:[x,y,z]`, optional `move` |
| `move_relative` | `offset:[x,y,z]`, `frame:"body"` or `"world"`, `move` |
| `follow_path` | `waypoints`, `options:PathOptions` |
| `move_to_object` | `object`, `standoff` (default 2 m), `depth`, `move` |
| `follow_object` | Same, plus optional `duration`; continually updates the goal |
| `orbit_object` | `object`, `clearance` (default 2 m), `options:PathOptions` |
| `pass_gate` | `object` (default gate), `approach_distance`, `exit_distance`, `options` |
| `look_at_object` | `object`, `move`; hold position and face the object |
| `look_at_point` | `point`, `move`; hold position and face the point |
| `set_yaw` | `yaw`, `move`; hold position with the requested heading |
| `set_attitude` | `orientation:[x,y,z,w]`, `move`; full quaternion orientation |
| `rotate` | `axis:"roll"/"pitch"/"yaw"`, signed `angle`, `rate` (default 0.3), `move` |
| `search` | Every `SearchOptions` field; sweep/scan or find an object |
| `visual_target` | `options:VisualOptions`, optional `object` selector, `move` |
| `body_move` | `options:BodyOptions`, `move`; force effort for a duration |
| `hold` | `{}`; replace the plan with a hold of measured pose |
| `stop` | `{}`; cancel the plan and keep motor outputs zero |
| `cancel` | `{}`; cancel the plan and hold the measured pose |
| `pause` | `{}`; retain progress, freeze command timers and keep outputs zero |
| `resume` | `{}`; resume the retained paused/failed motion |
| `status` | `{}`; motion, tracking, options, route progress and allocation residual |
| `objects` | `{}`; status including currently observed 3D/image objects |
| `configure_motion` | `options`; patch the active MoveOptions, preserving omissions |
| `configure_navigation` | `options`; patch Navigation.Options, then recapture a hold |
| `reanchor` | `transform:{pos,quat}`, `epoch`; reconcile localization, then explicitly resume |
| `to_mission` | `position`; convert tracking coordinates to mission coordinates |
| `plan` | `steps:[{command,args},...]`; execute up to 32 movement commands in order |

Object approach/following preserves current depth when `depth` is omitted.
Its default facing looks toward the selected object; explicitly provided
`move.facing` overrides that default. Orbiting similarly defaults to object
facing with pitch adaptation. Gate crossing owns the fixed crossing heading.
Look/heading/orientation commands own the orientation they request. A visual
selector resolves `options.object_id`; with no selector, that ID is used directly.

Rotations use a continuous quaternion reference with bounded lead over measured
orientation, so a `2*pi` command performs a turn rather than selecting an
identical endpoint. The signed angle is limited to `32*pi` per command. Position
control is best effort through steep attitudes; the rotation completes from
orientation and angular-rate feedback, then captures the measured end pose.
Geometry, tilt and thrust limits still restrict physically achievable movement.

### All Motion options

`move` exposes every `Motion.MoveOptions` field:

```json
{
  "move_relative": {
    "offset": [1, 0, 0],
    "frame": "body",
    "move": {
      "facing": {"hold_current": {}},
      "attitude_policy": "allow_pitch",
      "max_pitch": 0.5235988,
      "pitch_rate_limit": 0.25,
      "translation": "simultaneous",
      "position_tolerance": 0.2,
      "recovery": {"strategy": "estimate", "max_duration": 2}
    }
  }
}
```

Facing presets use tagged objects:
`{"hold_current":{}}`, `{"face_goal":{}}`, `{"face_path":{}}`,
`{"fixed_yaw":0.5}`, `{"look_at_point":[1,2,3]}`, `{"look_at_object":7}`.

Available MoveOptions are `facing`, `attitude`, `attitude_policy`, `max_pitch`,
`pitch_rate_limit`, `attitude_tolerance`, `angular_rate_tolerance`, `translation`,
`yaw_rate_limit`, `yaw_tolerance`, `alignment_gate`, `position_tolerance`,
`require_yaw`, `settle_time`, `observation_timeout`, `timeout`, and `recovery`.

- PathOptions: `move`, `corner_radius`, `reference_speed`, `max_tracking_error`.
- RecoveryOptions: `strategy`, `max_duration`, `confirm_tracking`, `resume_ramp`,
  `max_rejoin_distance`, `max_heading_jump`, `max_effort`, `search`, `visual`.
- SearchOptions: `target_class`, `target_id`, `require_position`, `duration`,
  `sweep_angle`, `sweep_rate`, `detection_settle_time`, `body_effort`, `max_effort`, `depth`.
- VisualOptions: `object_id`, `target_height`, `height_tolerance`,
  `center_tolerance`, `align_vertical`, `max_effort`, `depth`.
- BodyOptions: `effort`, `duration`, `max_effort`, `depth`.

Existing API validation applies, including incompatible orientation/recovery
combinations. `allow_pitch` is supported for world moves with suitable thruster
geometry; body and visual commands reject it. During `rotate`, the trajectory
owns the attitude; configure other options or issue a new rotation to replace it.

### All Navigation options

`configure_navigation` exposes `max_sensor_age`, `body_from_imu`,
`body_from_camera:{pos,quat}`, `prediction_duration`, `prediction_error_limit`,
`prediction_error_growth`, and `max_prediction_speed`.

```json
{"configure_navigation":{"options":{"prediction_duration":2,"prediction_error_limit":0.8}}}
```

These are runtime changes. Applying navigation settings cancels the active
operation and captures a new hold once measured pose returns. The command
does not write controller files; a later file reload can replace runtime values.
`configure_motion` patches only the active command. Explicit `null` clears a
nullable setting, such as `{"configure_motion":{"options":{"timeout":null}}}`.

### Ordered plans

```json
{
  "plan": {
    "steps": [
      {"command":"pass_gate","args":{}},
      {"command":"rotate","args":{"axis":"pitch","angle":6.2831853}},
      {"command":"rotate","args":{"axis":"roll","angle":6.2831853}},
      {"command":"search","args":{"target_class":"rect","duration":60}},
      {"command":"orbit_object","args":{"object":{"class":"rect"}}}
    ]
  }
}
```

A later step starts after the previous step completes. Object availability and
command validity are checked when that step starts; a failure ends the plan and
reports its step/error. `follow_object` runs continuously unless `duration` is
provided; a nonfinal follow step requires a duration. A new movement/stop/hold
command replaces the whole active plan. Pause/resume retains its progress.

## UDP protocol v1

Send UTF-8 JSON datagrams, at most 60,000 bytes. Start with:

```json
{"version":1,"kind":"hello","id":0,"nonce":"unique-client-nonce"}
```

The `welcome` response echoes the nonce and returns `session`, the 10-second
wall-clock lease, `commands`, shared schema `definitions`, and navigation/status.
Object observations are omitted from the welcome; query `status` or `objects`.
Repeated hello packets with the same nonce/address preserve the session.
Only one peer owns the session at a time. The session is bound to its UDP
source address/port; this is a local coordination protocol, with a loopback server.

After the handshake:

```json
{
  "version":1,
  "kind":"request",
  "id":1,
  "session":"token-from-welcome",
  "command":{"search":{"duration":20}}
}
```

Responses are `{version,kind,id,session,ok,error,data}`. `kind:"reply"`
acknowledges acceptance/rejection. Acceptance means the command started or
the query succeeded; it does not mean movement completed. Completion/failure,
cancellation and plan transitions produce `kind:"event"` with operation status.

- IDs must increase within a session. Retries must reuse identical bytes and ID.
- The latest 32 request responses are cached. Replays return the same response
  without starting movement again. Changed bytes with the same ID are rejected.
- Older uncached IDs are rejected as stale and never executed again.
- The Python client uses a separate receiver and serializes requests/heartbeats.
- `heartbeat` packets use the same ID/session fields and return current status.
  The client sends one every two wall-clock seconds while listening/interpreting.
- `bye` releases ownership. Lease expiry and disconnect also permit a new
  handshake. Accepted autonomous operations continue until completion or an
  explicit replacement/stop; disconnect does not cancel them.
- Events are best effort UDP notifications. Poll `status` to recover missed
  events. Match both `operation_id` and `operation_session`.
- If an acknowledgement times out, the command may have executed. Query status
  before issuing a new request for the same movement. A server restart loses
  the session; reconnect explicitly rather than replaying an uncertain command.

`stop` keeps outputs zero until another motion command. `hold`/`cancel` uses
feedback to hold measured pose. `pause` preserves the plan with zero output,
so it does not physically freeze the vehicle against buoyancy or current.
