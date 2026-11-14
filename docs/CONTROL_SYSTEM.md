# Control systems and vehicle models

`src/ControlSystem.zig` composes two Zig types at compile time. A controller's
`update(input)` returns an `Intent`, and an actuator mapping's `map(intent)`
returns a `Command`. The intent types must match. Each vehicle defines the
meaning, units and limits of its own types; the pipeline does not assume six
axes or a matrix. Both operations can report errors. A driver is responsible
for sending the resulting command to its simulator or hardware.

`src/AuvControl.zig` supplies the active AUV implementation. Its pose
controller uses the existing mission state and PID history to produce a
six-axis body command. Its thruster allocator maps that command through the
configured TAM, clamps and scales the result, and applies the configured
thruster curve. `MissionContext` obtains the body error, runs the pipeline,
and sends its result through the existing AUV interface. The controller still
uses the TAM to account for achievable forces and moments; its controller state
remains in `MissionContext.Saved` for application reload.

`src/AckermannControl.zig` provides an independent car control example. Its
`PathTracker` uses a supplied lookahead point and the rear axle pose to compute
pure-pursuit curvature. Its `Steering` mapping converts desired curvature to a
virtual front-wheel steering angle, clamps it to the configured steering limit,
and clamps signed target speed to the forward and reverse limits. `derivative`
implements the no-slip kinematic bicycle model for a rear-axle state, given
measured speed, acceleration and steering angle.

```zig
const Control = @import("ControlSystem.zig");
const Car = @import("AckermannControl.zig");

var system: Control.System(Car.PathTracker, Car.Steering) = .{
    .controller = .{},
    .actuation = .{ .config = .{
        .wheelbase = 2.5,          // metres
        .max_steering_angle = 0.5, // radians, virtual bicycle wheel
        .max_forward_speed = 5,    // metres per second
        .max_reverse_speed = 2,
    } },
};
const command = try system.step(.{
    .x = 0, .y = 0, .heading = 0, // metres and radians, world frame
    .target_x = 5, .target_y = 1,  // selected lookahead point, world frame
    .target_speed = 2,             // signed metres per second
});
// Send command.speed and command.steering_angle through a car driver.
```

The path tracker does not select a lookahead point or control the physical
drive motor. If the lookahead point coincides with the vehicle, it returns
`error.AckermannLookaheadTooClose` so the caller can select another point or
stop. `Control.Direct(Car.Intent)` can be used when guidance already computes
the desired speed and curvature.

The running application, missions, native simulator and C interface currently
describe AUVs and thrusters. Adding a drivable car requires a car scene and
physics/driver interface, a source of car state and path references, and an
integration that sends `Car.Command` to its steering and drive actuators.
The bicycle model does not model tire slip, steering dynamics, or the separate
left and right wheel angles. A stateful new controller must also add its state
to the application's versioned reload snapshot before it can survive reloads.
