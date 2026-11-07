const std = @import("std");
const Io = std.Io;

const MissionArgs = @This();

pub const MissionId = enum {
    prequalify_coco,
    prequalify_robosub,
    teleop,
};

fuzz: bool = false,
fuzz_nominal: bool = false,
master_seed: u64 = 1,
case_count: usize = 100,
workers: usize = 8,
replay_seed: ?u64 = null,
results_path: []const u8 = "fuzz-results",
goal_dist_threshold: f32,
auv_dynlib_path: []const u8,
mission_id: MissionId,
live_config_path: []const u8,

pub fn init(args_slice: []const [:0]const u8) !MissionArgs {
    if (args_slice.len < 4) {
        std.log.err("usage: {s} <auv_dynlib_path> <mission_name> <live_config_path>", .{args_slice[0]});
        return error.MissingArgs;
    }

    const auv_dynlib_path = args_slice[1];
    const mission_name = args_slice[2];
    const live_config_path = args_slice[3];
    const mission_id = std.meta.stringToEnum(MissionId, mission_name) orelse {
        std.log.err("mission_id `{s}` doesn't exist", .{mission_name});
        std.log.info("valids mission_id's are:", .{});
        for (std.enums.values(MissionId)) |mission_id| {
            const valid_mission_name = std.enums.tagName(MissionId, mission_id) orelse unreachable;
            std.log.info("\t{s}", .{valid_mission_name});
        }
        return error.InvalidMissionId;
    };

    var result: MissionArgs = .{
        .mission_id = mission_id,
        .auv_dynlib_path = auv_dynlib_path,
        .live_config_path = live_config_path,
        .goal_dist_threshold = 1.0,
    };
    var i: usize = 4;
    while (i < args_slice.len) : (i += 1) {
        const flag = args_slice[i];
        if (std.mem.eql(u8, flag, "--fuzz")) {
            result.fuzz = true;
            continue;
        }
        if (i + 1 == args_slice.len) return error.MissingOptionValue;
        i += 1;
        const value = args_slice[i];
        if (std.mem.eql(u8, flag, "--seed")) {
            result.master_seed = try std.fmt.parseInt(u64, value, 10);
        } else if (std.mem.eql(u8, flag, "--cases")) {
            result.case_count = try std.fmt.parseInt(usize, value, 10);
        } else if (std.mem.eql(u8, flag, "--workers")) {
            result.workers = try std.fmt.parseInt(usize, value, 10);
        } else if (std.mem.eql(u8, flag, "--replay-seed")) {
            result.replay_seed = try std.fmt.parseInt(u64, value, 10);
        } else if (std.mem.eql(u8, flag, "--profile")) {
            if (std.mem.eql(u8, value, "nominal")) {
                result.fuzz_nominal = true;
            } else if (std.mem.eql(u8, value, "default")) {
                result.fuzz_nominal = false;
            } else return error.UnknownFuzzProfile;
        } else if (std.mem.eql(u8, flag, "--results")) {
            result.results_path = value;
        } else return error.UnknownOption;
        if (!result.fuzz) return error.FuzzFlagRequired;
    }
    if (result.case_count == 0 or result.workers == 0 or result.results_path.len == 0)
        return error.InvalidFuzzOptions;
    if (result.replay_seed != null) {
        result.case_count = 1;
        result.workers = 1;
    }
    if (result.fuzz and mission_id == .teleop) return error.FuzzTeleopUnsupported;
    return result;
}

pub fn format(args: MissionArgs, writer: *std.Io.Writer) !void {
    try writer.print(
        "auv_dynlib_path = {s}, mission_name = {s}, goal_dist_threshold = {}",
        .{
            args.auv_dynlib_path,
            std.enums.tagName(MissionId, args.mission_id) orelse unreachable,
            args.goal_dist_threshold,
        },
    );
}
