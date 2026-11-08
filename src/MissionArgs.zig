const std = @import("std");
const MissionArgs = @This();
pub const MissionId = enum { prequalify_coco, prequalify_robosub, teleop };
application_path: []const u8 = "/proc/self/exe",
replay_file: ?[]const u8 = null,
backend_path: []const u8 = "",
executable_path: []const u8 = "/proc/self/exe",
fuzz: bool = false,
fuzz_nominal: bool = false,
master_seed: u64 = 1,
case_count: usize = 100,
workers: usize = 8,
replay_seed: ?u64 = null,
results_path: []const u8 = "fuzz-results",
goal_dist_threshold: f32 = 1,
auv_dynlib_path: []const u8 = "",
mission_id: MissionId = .prequalify_robosub,
live_config_path: []const u8 = "config/application.zig",
pub fn defaults() MissionArgs {
    const c = @import("../config/application.zig").config;
    return .{ .fuzz_nominal = c.fuzz.nominal, .master_seed = c.fuzz.seed, .case_count = c.fuzz.cases, .workers = c.fuzz.workers, .results_path = c.fuzz.results, .goal_dist_threshold = c.goal_dist_threshold, .auv_dynlib_path = c.plugin, .mission_id = @enumFromInt(@intFromEnum(c.mission)) };
}
pub fn init(argv: []const [:0]const u8) !MissionArgs {
    for (argv) |arg| if (std.mem.endsWith(u8, arg, ".json")) return error.JsonConfigurationRemovedUseConfigApplicationZig;
    var result = defaults();
    if (argv.len == 1 or (argv.len == 2 and std.mem.eql(u8, argv[1], "run"))) return result;
    if (argv.len == 2 and std.mem.eql(u8, argv[1], "fuzz")) {
        result.fuzz = true;
        if (result.mission_id == .teleop) return error.FuzzTeleopUnsupported;
        return result;
    }
    if (argv.len == 3 and std.mem.eql(u8, argv[1], "replay")) {
        result.fuzz = true;
        result.replay_seed = std.fmt.parseInt(u64, argv[2], 10) catch blk: {
            result.replay_file = argv[2];
            break :blk null;
        };
        result.case_count = 1;
        result.workers = 1;
        if (result.mission_id == .teleop) return error.FuzzTeleopUnsupported;
        return result;
    }
    return error.UsageRunFuzzReplay;
}
pub fn format(args: MissionArgs, writer: *std.Io.Writer) !void {
    try writer.print("mission={s}, plugin={s}", .{ @tagName(args.mission_id), args.auv_dynlib_path });
}
