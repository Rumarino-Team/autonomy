const std = @import("std");
const Io = std.Io;

const MissionArgs = @This();

pub const MissionId = enum {
    prequalify,
    teleop,
    teleop_command,
    batch,
    fuzz,
    record,
};

goal_dist_threshold: f32,
auv_dynlib_path: []const u8,
mission_id: MissionId,
live_config_path: []const u8,
master_seed: u64 = 1,
case_count: u64 = 32,
replay_seed: ?u64 = null,
record_seed: u64 = 0,
record_path: ?[:0]const u8 = null,

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

    var master_seed: u64 = 1;
    var case_count: u64 = 32;
    var replay_seed: ?u64 = null;
    if (mission_id == .fuzz and args_slice.len >= 5) {
        if (std.mem.eql(u8, args_slice[4], "replay")) {
            if (args_slice.len < 6) {
                std.log.err("usage: {s} <auv_dynlib_path> fuzz <live_config_path> replay <seed>", .{args_slice[0]});
                return error.MissingArgs;
            }
            replay_seed = parseU64(args_slice[5]) catch {
                std.log.err("replay seed `{s}` is not an integer", .{args_slice[5]});
                return error.InvalidFuzzSeed;
            };
        } else {
            master_seed = parseU64(args_slice[4]) catch {
                std.log.err("master seed `{s}` is not an integer", .{args_slice[4]});
                return error.InvalidFuzzSeed;
            };
            if (args_slice.len >= 6) {
                case_count = parseU64(args_slice[5]) catch {
                    std.log.err("case count `{s}` is not an integer", .{args_slice[5]});
                    return error.InvalidFuzzCount;
                };
                if (case_count == 0) return error.InvalidFuzzCount;
            }
        }
    }

    var record_seed: u64 = 0;
    var record_path: ?[:0]const u8 = null;
    if (mission_id == .record) {
        if (args_slice.len < 6) {
            std.log.err("usage: {s} <auv_dynlib_path> record <live_config_path> <seed> <out.rec.jsonl>", .{args_slice[0]});
            return error.MissingArgs;
        }
        record_seed = parseU64(args_slice[4]) catch {
            std.log.err("record seed `{s}` is not an integer", .{args_slice[4]});
            return error.InvalidFuzzSeed;
        };
        record_path = args_slice[5];
    }

    return .{
        .mission_id = mission_id,
        .auv_dynlib_path = auv_dynlib_path,
        .live_config_path = live_config_path,
        .goal_dist_threshold = 1.0,
        .master_seed = master_seed,
        .case_count = case_count,
        .replay_seed = replay_seed,
        .record_seed = record_seed,
        .record_path = record_path,
    };
}

fn parseU64(text: []const u8) !u64 {
    return std.fmt.parseInt(u64, text, 10);
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
    if (args.mission_id == .fuzz) {
        if (args.replay_seed) |seed| {
            try writer.print(", replay_seed = {d}", .{seed});
        } else {
            try writer.print(", master_seed = {d}, case_count = {d}", .{ args.master_seed, args.case_count });
        }
    }
    if (args.mission_id == .record) {
        try writer.print(", seed = {d}, record = {s}", .{ args.record_seed, args.record_path.? });
    }
}
