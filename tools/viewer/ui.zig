const std = @import("std");
const rl = @import("raylib");
const IO = @import("MissionIO");

pub const Workspace = enum { operation, mission, tuning };
pub const Status = enum { disabled, connecting, live, ready, stale, unavailable };
pub const Rect = rl.Rectangle;
const bg = color(9, 17, 24, 255);
const sidebar = color(12, 24, 32, 245);
const glass = color(21, 42, 51, 224);
const surface = color(18, 36, 45, 255);
const border = color(63, 101, 107, 180);
const white = color(235, 245, 242, 255);
const muted = color(157, 180, 183, 255);
const teal = color(99, 220, 191, 255);
const blue = color(133, 198, 242, 255);
const amber = color(244, 187, 113, 255);
const red = color(242, 123, 120, 255);
const axis_names = [_][:0]const u8{ "X", "Y", "Z", "ROLL", "PITCH", "YAW" };
const groups = [_][:0]const u8{ "P", "I", "D" };
const gain_max = [_]f32{ 10, 2, 10 };
pub const min_font_percent: u8 = 80;
pub const max_font_percent: u8 = 200;
var font_scale: f32 = 1.0;

pub fn adjustFontPercent(current: u8, delta: i16) u8 {
    return @intCast(std.math.clamp(@as(i16, current) + delta, min_font_percent, max_font_percent));
}

fn color(r: u8, g: u8, b: u8, a: u8) rl.Color {
    return .{ .r = r, .g = g, .b = b, .a = a };
}
fn rect(x: f32, y: f32, w: f32, h: f32) Rect {
    return .{ .x = x, .y = y, .width = w, .height = h };
}
fn inset(r: Rect, x: f32, y: f32) Rect {
    return rect(r.x + x, r.y + y, @max(1, r.width - 2 * x), @max(1, r.height - 2 * y));
}
fn box(r: Rect, c: rl.Color) void {
    rl.drawRectangleRec(r, c);
}
fn panel(r: Rect) void {
    rl.drawRectangleRounded(r, 0.035, 8, glass);
    rl.drawRectangleRoundedLinesEx(r, 0.035, 8, 1, border);
}
fn text(font: rl.Font, s: [:0]const u8, x: f32, y: f32, size: f32, c: rl.Color) void {
    rl.drawTextEx(font, s, .{ .x = x, .y = y }, size * font_scale, 0.15 * font_scale, c);
}
fn title(font: rl.Font, s: [:0]const u8, r: Rect) void {
    text(font, s, r.x + 14, r.y + 12, 13, muted);
}
fn line(x: f32, y: f32, w: f32) void {
    box(rect(x, y, w, 1), border);
}
fn statusText(s: Status) [:0]const u8 {
    return switch (s) {
        .disabled => "OFF",
        .connecting => "CONNECTING",
        .live => "LIVE",
        .ready => "READY",
        .stale => "STALE",
        .unavailable => "UNAVAILABLE",
    };
}
fn statusColor(s: Status) rl.Color {
    return switch (s) {
        .live, .ready => teal,
        .connecting, .stale => amber,
        .disabled => muted,
        .unavailable => red,
    };
}
fn pill(font: rl.Font, r: Rect, label: [:0]const u8, s: Status) void {
    rl.drawRectangleRounded(r, 0.25, 6, surface);
    rl.drawRectangleRoundedLinesEx(r, 0.25, 6, 1, border);
    rl.drawCircleV(.{ .x = r.x + 10, .y = r.y + r.height / 2 }, 3, statusColor(s));
    const label_text = rl.textFormat("%s %s", .{ label.ptr, statusText(s).ptr });
    const wanted_size = 11 * font_scale;
    const measured = rl.measureTextEx(font, label_text, wanted_size, 0.15 * font_scale);
    const fit = @min(@as(f32, 1), (r.width - 25) / @max(1, measured.x));
    rl.drawTextEx(font, label_text, .{ .x = r.x + 19, .y = r.y + 7 }, wanted_size * fit, 0.15 * font_scale * fit, statusColor(s));
}

pub const Layout = struct {
    video_enabled: bool,
    sidebar: Rect,
    top: Rect,
    content: Rect,
    video: Rect,
    scene: Rect,
    state: Rect,
    thrusters: Rect,
    controls: Rect,
    objects: Rect,
    goals: Rect,
    editor: Rect,
    gains: Rect,
    observed: Rect,
    update: Rect,
    pub fn init(w: f32, h: f32, video_enabled: bool) Layout {
        const side: f32 = 166;
        const gap: f32 = 14;
        const x = side + 20;
        const y: f32 = 135;
        const cw = @max(1, w - x - 20);
        const ch = @max(1, h - y - 18);
        const lw = @floor((cw - gap) * 0.56);
        const rw = cw - gap - lw;
        const rx = x + lw + gap;
        const upper = @floor((ch - gap) * 0.49);
        const state_h = @floor((ch - gap) * 0.34);
        const mission_scene = @floor((ch - gap) * 0.62);
        const gainw = @floor((cw - gap) * 0.70);
        const smallw = cw - gap - gainw;
        return .{
            .video_enabled = video_enabled,
            .sidebar = rect(0, 0, side, h),
            .top = rect(side, 0, w - side, 64),
            .content = rect(x, y, cw, ch),
            .video = rect(x, y, lw, upper),
            .scene = rect(x, y + upper + gap, lw, ch - upper - gap),
            .state = rect(rx, y, rw, state_h),
            .thrusters = rect(rx, y + state_h + gap, rw * 0.48 - gap / 2, ch - state_h - gap),
            .controls = rect(rx + rw * 0.48 + gap / 2, y + state_h + gap, rw * 0.52 - gap / 2, ch - state_h - gap),
            .objects = rect(x, y + mission_scene + gap, (lw - gap) / 2, ch - mission_scene - gap),
            .goals = rect(x + (lw + gap) / 2, y + mission_scene + gap, (lw - gap) / 2, ch - mission_scene - gap),
            .editor = rect(rx, y, rw, ch),
            .gains = rect(x, y, gainw, ch * 0.70 - gap / 2),
            .observed = rect(x + gainw + gap, y, smallw, ch * 0.43 - gap / 2),
            .update = rect(x, y + ch * 0.70 + gap / 2, gainw, ch * 0.30 - gap / 2),
        };
    }
    pub fn sceneContent(self: Layout, ws: Workspace) Rect {
        const r = if (ws == .mission)
            rect(self.content.x, self.content.y, self.scene.width, self.objects.y - self.content.y - 14)
        else if (!self.video_enabled)
            rect(self.content.x, self.content.y, self.scene.width, self.content.height)
        else
            self.scene;
        return rect(r.x + 8, r.y + 38, r.width - 16, r.height - 46);
    }
    pub fn pidRect(self: Layout, index: usize) Rect {
        const a = self.gains;
        const left: f32 = 58;
        const gap: f32 = 8;
        const cell = (a.width - left - 20 - 5 * gap) / 6;
        const row = (a.height - 102) / 3;
        return rect(a.x + left + @as(f32, @floatFromInt(index % 6)) * (cell + gap), a.y + 88 + @as(f32, @floatFromInt(index / 6)) * row, cell, 22);
    }
    pub fn listCapacity(r: Rect) usize {
        return @intFromFloat(@max(0, @floor((r.height - 78) / 20)));
    }
    pub fn navRect(self: Layout, i: usize) Rect {
        return rect(10, 105 + @as(f32, @floatFromInt(i)) * 44, self.sidebar.width - 20, 36);
    }
    pub fn fontButton(self: Layout, increase: bool) Rect {
        return rect(if (increase) 111 else 16, self.sidebar.height - 142, 38, 30);
    }
    pub fn missionButton(self: Layout, i: usize) Rect {
        return rect(self.editor.x + 14 + @as(f32, @floatFromInt(i)) * 98, self.editor.y + 82, 90, 32);
    }
    pub fn sendButton(self: Layout) Rect {
        return rect(self.editor.x + 14, self.editor.y + self.editor.height - 62, 170, 34);
    }
};

pub const Data = struct {
    font: rl.Font,
    font_percent: u8,
    workspace: Workspace,
    layout: Layout,
    frame: *const IO.TelemetryFrame,
    telemetry_status: Status,
    video_status: Status,
    video: rl.Texture2D,
    video_w: i32,
    video_h: i32,
    scene: rl.RenderTexture2D,
    mission_control: bool,
    socket_ok: bool,
    send_error: bool,
    translation: [3]f32,
    orientation: [4]f32,
    mission_kind: IO.MissionCommandKind,
    mission_maneuver: IO.MissionManeuver,
    mission_position: [3]f32,
    mission_look: [3]f32,
    mission_class: u8,
    target_initialized: bool,
    config: *IO.LiveConfigPacket,
    tune_selection: usize,
    mission_sent: bool,
    tune_sent: bool,
    object_scroll: usize,
    goal_scroll: usize,
};
fn gain(d: Data, i: usize) f32 {
    return switch (i / 6) {
        0 => d.config.odometry.kp[i % 6],
        1 => d.config.odometry.ki[i % 6],
        else => d.config.odometry.kd[i % 6],
    };
}
fn observedGain(d: Data, i: usize) f32 {
    return switch (i / 6) {
        0 => d.frame.kp[i % 6],
        1 => d.frame.ki[i % 6],
        else => d.frame.kd[i % 6],
    };
}
fn button(font: rl.Font, r: Rect, label: [:0]const u8, active: bool) void {
    rl.drawRectangleRounded(r, 0.18, 6, if (active) teal else surface);
    rl.drawRectangleRoundedLinesEx(r, 0.18, 6, 1, border);
    text(font, label, r.x + 10, r.y + 9, 12, if (active) bg else white);
}
fn header(d: Data) void {
    const l = d.layout;
    box(l.sidebar, sidebar);
    box(l.top, sidebar);
    text(d.font, "AUV", 18, 19, 18, teal);
    text(d.font, "AUTONOMY", 64, 20, 14, white);
    text(d.font, "VIEWER", 64, 39, 11, muted);
    line(14, 70, l.sidebar.width - 28);
    text(d.font, "WORKSPACES", 18, 84, 11, muted);
    const names = [_][:0]const u8{ "Operation", "Mission", "Tuning" };
    for (names, 0..) |name, i| {
        const r = l.navRect(i);
        if (@backingInt(d.workspace) == i) rl.drawRectangleRounded(r, 0.15, 6, color(28, 64, 65, 255));
        text(d.font, rl.textFormat("0%d", .{@as(c_int, @intCast(i + 1))}), r.x + 9, r.y + 10, 11, teal);
        text(d.font, name, r.x + 34, r.y + 9, 14, white);
    }
    line(14, l.sidebar.height - 181, l.sidebar.width - 28);
    text(d.font, "FONT SIZE", 18, l.sidebar.height - 169, 11, muted);
    const decrease = l.fontButton(false);
    const increase = l.fontButton(true);
    button(d.font, decrease, "-", false);
    button(d.font, increase, "+", false);
    text(d.font, rl.textFormat("%d%%", .{@as(c_int, d.font_percent)}), 63, l.sidebar.height - 134, 12, white);
    line(14, l.sidebar.height - 96, l.sidebar.width - 28);
    text(d.font, "CONTROL MODE", 18, l.sidebar.height - 82, 11, muted);
    text(d.font, if (d.mission_control) "Mission control" else "Teleoperation", 18, l.sidebar.height - 61, 13, white);
    text(d.font, "T  switch mode", 18, l.sidebar.height - 38, 11, muted);
    text(d.font, names[@backingInt(d.workspace)], l.top.x + 20, 12, 19, white);
    const sx = l.top.x + l.top.width;
    pill(d.font, rect(sx - 414, 19, 130, 28), "TLM", d.telemetry_status);
    pill(d.font, rect(sx - 275, 19, 115, 28), "VIDEO", d.video_status);
    pill(d.font, rect(sx - 151, 19, 137, 28), "SEND", if (!d.socket_ok or d.send_error) .unavailable else .ready);
    const headline = switch (d.workspace) {
        .operation => "Live operation",
        .mission => "Mission debugging",
        .tuning => "Controller tuning",
    };
    text(d.font, headline, l.content.x, 78, 24, white);
    if (d.workspace == .mission) {
        var step: [IO.telemetry_step_max + 1]u8 = undefined;
        const n = @min(@as(usize, d.frame.step_len), d.frame.step.len);
        @memcpy(step[0..n], d.frame.step[0..n]);
        step[n] = 0;
        text(d.font, if (d.telemetry_status == .live and n > 0) rl.textFormat("MISSION STEP: %s", .{step[0..n :0].ptr}) else "MISSION STEP: unavailable", l.content.x, 108, 13, muted);
    } else {
        text(d.font, if (d.workspace == .operation) "Vehicle, surroundings and active commands at a glance" else "Adjust gains live and compare with telemetry", l.content.x, 108, 13, muted);
    }
}
fn frameContent(d: Data, r: Rect) void {
    const src = rect(0, 0, @floatFromInt(d.scene.texture.width), -@as(f32, @floatFromInt(d.scene.texture.height)));
    rl.drawTexturePro(d.scene.texture, src, r, .{ .x = 0, .y = 0 }, 0, rl.Color.white);
}
fn scenePanel(d: Data, r: Rect) void {
    const panelrect = rect(r.x - 8, r.y - 38, r.width + 16, r.height + 46);
    panel(panelrect);
    title(d.font, "3D SCENE", panelrect);
    if (d.telemetry_status == .live) {
        frameContent(d, r);
        box(rect(r.x + 4, r.y + 4, @min(r.width - 8, 285), 55), color(9, 17, 24, 210));
        text(d.font, "Right drag: orbit  |  Wheel: zoom", r.x + 8, r.y + 8, 11, muted);
        text(d.font, "Blue: vehicle  Orange: nose  Green: command", r.x + 8, r.y + 24, 11, muted);
        text(d.font, "Pink: movement  Yellow: error  Lime: goals", r.x + 8, r.y + 40, 11, muted);
    } else {
        box(r, color(10, 25, 33, 255));
        text(d.font, "Telemetry unavailable", r.x + 16, r.y + r.height / 2 - 12, 16, amber);
    }
}
fn videoPanel(d: Data, p: Rect) void {
    panel(p);
    title(d.font, "FORWARD CAMERA", p);
    const r = rect(p.x + 8, p.y + 38, p.width - 16, p.height - 46);
    box(r, color(10, 25, 33, 255));
    if (d.video_status != .live or d.video_w <= 0 or d.video_h <= 0) {
        text(d.font, if (d.video_status == .disabled) "Video disabled" else if (d.video_status == .stale) "Video frames are stale" else if (d.video_status == .unavailable) "Video unavailable" else "Waiting for video", r.x + 16, r.y + r.height / 2 - 12, 16, amber);
        return;
    }
    const vw: f32 = @floatFromInt(d.video_w);
    const vh: f32 = @floatFromInt(d.video_h);
    const scale = @min(r.width / vw, r.height / vh);
    const dest = rect(r.x + (r.width - vw * scale) / 2, r.y + (r.height - vh * scale) / 2, vw * scale, vh * scale);
    rl.drawTexturePro(d.video, rect(0, 0, vw, vh), dest, .{ .x = 0, .y = 0 }, 0, rl.Color.white);
    if (d.telemetry_status == .live) for (d.frame.boxes[0..d.frame.boxes_len]) |b| {
        if (b.bottom_right.x < b.top_left.x or b.bottom_right.y < b.top_left.y) continue;
        const x0 = @min(@as(f32, @floatFromInt(b.top_left.x)), vw);
        const y0 = @min(@as(f32, @floatFromInt(b.top_left.y)), vh);
        const x1 = @min(@as(f32, @floatFromInt(b.bottom_right.x)), vw);
        const y1 = @min(@as(f32, @floatFromInt(b.bottom_right.y)), vh);
        rl.drawRectangleLinesEx(rect(dest.x + x0 * scale, dest.y + y0 * scale, (x1 - x0) * scale, (y1 - y0) * scale), 2, teal);
    };
}
fn thrusterPanel(d: Data, p: Rect) void {
    panel(p);
    title(d.font, "THRUSTERS", p);
    if (d.telemetry_status != .live) {
        text(d.font, "Unavailable", p.x + 14, p.y + 44, 13, amber);
        return;
    }
    if (d.frame.thrusters_len == 0) {
        text(d.font, "No data", p.x + 14, p.y + 44, 13, muted);
        return;
    }
    const count: usize = d.frame.thrusters_len;
    const step = @min(@as(f32, 27), (p.height - 55) / @as(f32, @floatFromInt(@max(count, 1))));
    for (d.frame.thrusters[0..count], 0..) |v, i| {
        const yy = p.y + 45 + @as(f32, @floatFromInt(i)) * step;
        const start = p.x + 37;
        const end = p.x + p.width - 45;
        const mid = (start + end) / 2;
        text(d.font, rl.textFormat("T%d", .{@as(c_int, @intCast(i))}), p.x + 12, yy, 11, muted);
        box(rect(start, yy + 7, @max(1, end - start), 1), border);
        box(rect(mid, yy + 2, 1, 12), muted);
        const width = @min(1, @abs(v)) * (end - start) / 2;
        box(rect(if (v < 0) mid - width else mid, yy + 3, @max(1, width), 8), blue);
        text(d.font, rl.textFormat("%+.2f", .{@as(f64, v)}), end + 4, yy, 10, white);
    }
}
fn statePanel(d: Data, p: Rect) void {
    panel(p);
    title(d.font, "VEHICLE STATE", p);
    if (d.telemetry_status != .live) {
        text(d.font, "Telemetry unavailable", p.x + 14, p.y + 48, 15, amber);
        return;
    }
    var buf: [IO.telemetry_step_max + 1]u8 = undefined;
    const n = @min(@as(usize, d.frame.step_len), d.frame.step.len);
    @memcpy(buf[0..n], d.frame.step[0..n]);
    buf[n] = 0;
    text(d.font, "DEPTH", p.x + 14, p.y + 40, 11, muted);
    text(d.font, if (d.frame.depth_ok) rl.textFormat("%.2f m", .{@as(f64, d.frame.depth)}) else "--", p.x + 14, p.y + 57, 23, white);
    text(d.font, "MISSION STEP", p.x + p.width * 0.47, p.y + 40, 11, muted);
    text(d.font, buf[0..n :0], p.x + p.width * 0.47, p.y + 62, 13, white);
    line(p.x + 14, p.y + 92, p.width - 28);
    text(d.font, rl.textFormat("Position: %.2f  %.2f  %.2f m", .{ @as(f64, d.frame.camera_pose.pos[0]), @as(f64, d.frame.camera_pose.pos[1]), @as(f64, d.frame.camera_pose.pos[2]) }), p.x + 14, p.y + 103, 13, white);
    text(d.font, rl.textFormat("Position xyz: %s %s %s", .{ if (d.frame.position_ok.x) "OK" else "--", if (d.frame.position_ok.y) "OK" else "--", if (d.frame.position_ok.z) "OK" else "--" }), p.x + 14, p.y + 126, 12, if (d.frame.position_ok.x and d.frame.position_ok.y and d.frame.position_ok.z) teal else amber);
    if (p.height > 175) text(d.font, rl.textFormat("Attitude xyz: %s %s %s", .{ if (d.frame.attitude_ok.x) "OK" else "--", if (d.frame.attitude_ok.y) "OK" else "--", if (d.frame.attitude_ok.z) "OK" else "--" }), p.x + 14, p.y + 148, 12, if (d.frame.attitude_ok.x and d.frame.attitude_ok.y and d.frame.attitude_ok.z) teal else amber);
}
fn controlPanel(d: Data, p: Rect) void {
    panel(p);
    title(d.font, "CONTROLS", p);
    const rows = [_][:0]const u8{ "W / S   forward / back", "A / D   left / right", "R / F   down / up", "Arrows  yaw / pitch", "I / K   roll", "T       change mode" };
    if (d.mission_control) {
        text(d.font, "Mission control active", p.x + 12, p.y + 43, 12, amber);
        text(d.font, "Open Mission workspace", p.x + 12, p.y + 65, 12, muted);
        return;
    }
    const spacing = @min(@as(f32, 29), (p.height - 55) / 6);
    for (rows, 0..) |row, i| text(d.font, row, p.x + 12, p.y + 44 + @as(f32, @floatFromInt(i)) * spacing, 11, white);
    if (p.height > 285) {
        const y = p.y + 48 + 6 * spacing;
        line(p.x + 12, y, p.width - 24);
        text(d.font, "COMMAND  X / Y / Z", p.x + 12, y + 12, 10, muted);
        text(d.font, rl.textFormat("%.1f  %.1f  %.1f", .{ @as(f64, d.translation[0]), @as(f64, d.translation[1]), @as(f64, d.translation[2]) }), p.x + 12, y + 30, 12, white);
    }
}
fn operation(d: Data) void {
    if (d.layout.video_enabled) videoPanel(d, d.layout.video);
    scenePanel(d, d.layout.sceneContent(.operation));
    statePanel(d, d.layout.state);
    thrusterPanel(d, d.layout.thrusters);
    controlPanel(d, d.layout.controls);
}
fn mission(d: Data) void {
    const l = d.layout;
    scenePanel(d, l.sceneContent(.mission));
    panel(l.objects);
    title(d.font, "OBJECTS", l.objects);
    panel(l.goals);
    title(d.font, "QUEUED GOALS", l.goals);
    if (d.telemetry_status == .live) {
        text(d.font, rl.textFormat("%d detected", .{@as(c_int, d.frame.objects_len)}), l.objects.x + 12, l.objects.y + 39, 12, muted);
        const maxrows = Layout.listCapacity(l.objects);
        const object_end = @min(@as(usize, d.frame.objects_len), d.object_scroll + maxrows);
        for (d.frame.objects[d.object_scroll..object_end], 0..) |o, i| text(d.font, rl.textFormat("ID %d  CLASS %d", .{ @as(c_uint, o.id), @as(c_uint, o.cls) }), l.objects.x + 12, l.objects.y + 62 + @as(f32, @floatFromInt(i)) * 20, 11, white);
        text(d.font, rl.textFormat("%d goals", .{@as(c_int, d.frame.goals_len)}), l.goals.x + 12, l.goals.y + 39, 12, muted);
        const goal_end = @min(@as(usize, d.frame.goals_len), d.goal_scroll + maxrows);
        for (d.frame.goals[d.goal_scroll..goal_end], 0..) |g, i| text(d.font, rl.textFormat("%d  %.1f  %.1f  %.1f", .{ @as(c_int, @intCast(d.goal_scroll + i + 1)), @as(f64, g.position[0]), @as(f64, g.position[1]), @as(f64, g.position[2]) }), l.goals.x + 12, l.goals.y + 62 + @as(f32, @floatFromInt(i)) * 20, 11, white);
        if (d.frame.objects_len > maxrows) text(d.font, "Scroll for more", l.objects.x + 12, l.objects.y + l.objects.height - 20, 11, muted);
        if (d.frame.goals_len > maxrows) text(d.font, "Scroll for more", l.goals.x + 12, l.goals.y + l.goals.height - 20, 11, muted);
    } else {
        text(d.font, "Unavailable", l.objects.x + 12, l.objects.y + 42, 12, amber);
        text(d.font, "Unavailable", l.goals.x + 12, l.goals.y + 42, 12, amber);
    }
    const p = l.editor;
    panel(p);
    title(d.font, "MISSION COMMAND", p);
    text(d.font, if (d.mission_control) "MISSION CONTROL ACTIVE" else "Press T to enable mission control", p.x + 14, p.y + 43, 12, if (d.mission_control) teal else amber);
    button(d.font, l.missionButton(0), "Point  P", d.mission_kind == .point);
    button(d.font, l.missionButton(1), "Object  O", d.mission_kind == .object);
    text(d.font, if (d.mission_kind == .point) "TARGET POSITION (X / Y / Z)" else "TARGET OBJECT CLASS", p.x + 14, p.y + 132, 11, muted);
    if (d.mission_kind == .point) {
        text(d.font, if (d.target_initialized and d.telemetry_status == .live) rl.textFormat("%.2f  %.2f  %.2f m", .{ @as(f64, d.mission_position[0]), @as(f64, d.mission_position[1]), @as(f64, d.mission_position[2]) }) else if (!d.mission_control) "Enable mission mode to set target" else "Waiting for fresh telemetry", p.x + 14, p.y + 154, 15, white);
        text(d.font, "LOOK DIRECTION", p.x + 14, p.y + 187, 11, muted);
        text(d.font, rl.textFormat("%.2f  %.2f  %.2f", .{ @as(f64, d.mission_look[0]), @as(f64, d.mission_look[1]), @as(f64, d.mission_look[2]) }), p.x + 14, p.y + 207, 14, white);
    } else {
        text(d.font, rl.textFormat("Class %d  |  %s", .{ @as(c_int, d.mission_class), @tagName(d.mission_maneuver).ptr }), p.x + 14, p.y + 154, 15, white);
        text(d.font, "Q / E class   M / B / C maneuver", p.x + 14, p.y + 187, 12, muted);
    }
    line(p.x + 14, p.y + 240, p.width - 28);
    text(d.font, "WASD / RF: target   Arrows: look", p.x + 14, p.y + 255, 12, muted);
    text(d.font, "UDP send does not confirm receipt.", p.x + 14, p.y + p.height - 110, 12, amber);
    button(d.font, l.sendButton(), "Send command  Enter", d.mission_control and d.socket_ok and (d.mission_kind != .point or (d.target_initialized and d.telemetry_status == .live)));
    text(d.font, if (d.send_error) "Send failed" else if (d.mission_sent) "Last command sent locally" else "No command sent", p.x + 14, p.y + p.height - 22, 11, if (d.send_error) red else muted);
}
fn tuning(d: Data) void {
    const l = d.layout;
    const p = l.gains;
    panel(p);
    title(d.font, "ODOMETRY PID GAINS", p);
    text(d.font, "Live updates: the full configuration is sent immediately", p.x + 14, p.y + 34, 12, amber);
    for (0..6) |a| {
        const r = l.pidRect(a);
        text(d.font, axis_names[a], r.x, r.y - 24, 11, muted);
    }
    for (0..18) |i| {
        const r = l.pidRect(i);
        if (i % 6 == 0) text(d.font, groups[i / 6], p.x + 18, r.y, 17, teal);
        const v = gain(d, i);
        box(r, color(41, 68, 75, 255));
        box(rect(r.x, r.y, r.width * std.math.clamp(v / gain_max[i / 6], 0, 1), r.height), if (i == d.tune_selection) amber else teal);
        rl.drawRectangleLinesEx(r, 1, border);
        text(d.font, rl.textFormat("%.2f", .{@as(f64, v)}), r.x + 3, r.y + r.height + 4, 11, white);
    }
    text(d.font, "Q / E select (teleop)   Z / X adjust   Drag a slider", p.x + 14, p.y + p.height - 22, 11, muted);
    panel(l.observed);
    title(d.font, "OBSERVED CONTROLLER", l.observed);
    if (d.telemetry_status == .live) {
        text(d.font, "TRANSLATION ERROR", l.observed.x + 14, l.observed.y + 42, 11, muted);
        text(d.font, rl.textFormat("%+.2f  %+.2f  %+.2f", .{ @as(f64, d.frame.translation_error[0]), @as(f64, d.frame.translation_error[1]), @as(f64, d.frame.translation_error[2]) }), l.observed.x + 14, l.observed.y + 62, 13, white);
        text(d.font, "ATTITUDE ERROR", l.observed.x + 14, l.observed.y + 94, 11, muted);
        text(d.font, rl.textFormat("%+.2f  %+.2f  %+.2f", .{ @as(f64, d.frame.attitude_error[0]), @as(f64, d.frame.attitude_error[1]), @as(f64, d.frame.attitude_error[2]) }), l.observed.x + 14, l.observed.y + 114, 13, white);
    } else text(d.font, "Telemetry unavailable", l.observed.x + 14, l.observed.y + 45, 13, amber);
    const t = rect(l.observed.x, l.observed.y + l.observed.height + 14, l.observed.width, l.content.y + l.content.height - l.observed.y - l.observed.height - 14);
    thrusterPanel(d, t);
    panel(l.update);
    title(d.font, "LIVE UPDATE", l.update);
    text(d.font, rl.textFormat("Requested: %.2f", .{@as(f64, gain(d, d.tune_selection))}), l.update.x + 14, l.update.y + 42, 14, white);
    text(d.font, if (d.telemetry_status == .live) rl.textFormat("Observed: %.2f", .{@as(f64, observedGain(d, d.tune_selection))}) else "Observed: unavailable", l.update.x + 14, l.update.y + 66, 13, teal);
    if (l.update.height > 110) text(d.font, if (d.send_error) "Send failed" else if (d.tune_sent) "Sent locally; no receipt confirmation" else "No gain update sent", l.update.x + 14, l.update.y + 91, 12, if (d.send_error) red else muted);
}
pub fn draw(d: Data) void {
    font_scale = @as(f32, @floatFromInt(d.font_percent)) / 100.0;
    rl.clearBackground(bg);
    rl.drawCircleGradient(.{ .x = d.layout.content.x + d.layout.content.width * 0.4, .y = 40 }, d.layout.content.width * 0.6, color(9, 84, 76, 105), color(9, 84, 76, 0));
    rl.drawCircleGradient(.{ .x = d.layout.content.x + d.layout.content.width, .y = d.layout.content.y + d.layout.content.height }, d.layout.content.width * 0.55, color(11, 48, 85, 100), color(11, 48, 85, 0));
    header(d);
    switch (d.workspace) {
        .operation => operation(d),
        .mission => mission(d),
        .tuning => tuning(d),
    }
}

test "workspace layouts stay within supported window sizes" {
    for ([_][2]f32{ .{ 1000, 700 }, .{ 1280, 1000 }, .{ 1920, 1080 } }) |size| {
        const l = Layout.init(size[0], size[1], true);
        const panels = [_]Rect{ l.video, l.scene, l.state, l.thrusters, l.controls, l.objects, l.goals, l.editor, l.gains, l.observed, l.update, l.sceneContent(.mission) };
        for (panels) |p| {
            try std.testing.expect(p.width > 0 and p.height > 0);
            try std.testing.expect(p.x >= 0 and p.y >= 0);
            try std.testing.expect(p.x + p.width <= size[0] and p.y + p.height <= size[1]);
        }
        for (0..18) |i| {
            const r = l.pidRect(i);
            try std.testing.expect(r.x >= l.gains.x and r.y >= l.gains.y);
            try std.testing.expect(r.x + r.width <= l.gains.x + l.gains.width);
            try std.testing.expect(r.y + r.height + 16 <= l.gains.y + l.gains.height);
            if (i % 6 != 0) try std.testing.expect(l.pidRect(i - 1).x + l.pidRect(i - 1).width <= r.x);
        }
    }
}
