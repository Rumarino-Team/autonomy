const std = @import("std");
const Io = std.Io;
const MissionConfig = @import("MissionConfig.zig");

const MissionArgs = @This();

pub const MissionId = enum {
    prequalify_coco,
    prequalify_robosub,
    teleop,
    mission_control,
};

auv_dynlib_path: []const u8,
mission_id: MissionId,
mission_profile: MissionConfig.MissionProfileId,
fuzz: bool = false,

pub fn init(args_slice: []const [:0]const u8) !MissionArgs {
    if (args_slice.len < 4) {
        std.log.err("usage: {s} <auv_dynlib_path> <mission_name> <mission_profile>", .{args_slice[0]});
        std.log.info("mission_profile is one of: stonefish_sim, proteus_hwd", .{});
        return error.MissingArgs;
    }

    const auv_dynlib_path = args_slice[1];
    const mission_name = args_slice[2];
    const profile_name = args_slice[3];
    const mission_id = std.meta.stringToEnum(MissionId, mission_name) orelse {
        std.log.err("mission_id `{s}` doesn't exist", .{mission_name});
        std.log.info("valids mission_id's are:", .{});
        for (std.enums.values(MissionId)) |mission_id| {
            const valid_mission_name = std.enums.tagName(MissionId, mission_id) orelse unreachable;
            std.log.info("\t{s}", .{valid_mission_name});
        }
        return error.InvalidMissionId;
    };

    const mission_profile = MissionConfig.MissionProfileId.fromName(profile_name) orelse {
        std.log.err("mission_profile `{s}` doesn't exist", .{profile_name});
        std.log.info("valid mission_profile values are:", .{});
        for (std.enums.values(MissionConfig.MissionProfileId)) |profile| {
            const name = std.enums.tagName(MissionConfig.MissionProfileId, profile) orelse unreachable;
            std.log.info("\t{s}", .{name});
        }
        return error.InvalidMissionProfile;
    };

    return .{
        .mission_id = mission_id,
        .auv_dynlib_path = auv_dynlib_path,
        .mission_profile = mission_profile,
    };
}

pub fn configForProfile(profile: MissionConfig.MissionProfileId) MissionConfig.MissionConfig {
    return MissionConfig.compiledProfile(profile).*;
}

pub fn format(args: MissionArgs, writer: *std.Io.Writer) !void {
    try writer.print(
        "auv_dynlib_path = {s}, mission_name = {s}, mission_profile = {s}",
        .{
            args.auv_dynlib_path,
            std.enums.tagName(MissionId, args.mission_id) orelse unreachable,
            std.enums.tagName(MissionConfig.MissionProfileId, args.mission_profile) orelse unreachable,
        },
    );
}
