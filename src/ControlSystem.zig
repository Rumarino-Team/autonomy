//! A typed controller and actuator mapping. Each platform chooses the units and
//! shape of its input, intent, and commands; no vehicle geometry is assumed.

pub fn System(comptime Controller: type, comptime Actuation: type) type {
    if (Controller.Intent != Actuation.Intent)
        @compileError("controller output and actuator input must have the same type");
    return struct {
        const Self = @This();
        pub const Input = Controller.Input;
        pub const Command = Actuation.Command;

        controller: Controller,
        actuation: Actuation,

        pub fn step(self: *Self, input: Input) !Command {
            const intent = try self.controller.update(input);
            return try self.actuation.map(intent);
        }
    };
}

/// A useful controller when guidance already provides the platform intent.
pub fn Direct(comptime T: type) type {
    return struct {
        pub const Input = T;
        pub const Intent = T;

        pub fn update(_: *@This(), input: Input) !Intent {
            return input;
        }
    };
}
