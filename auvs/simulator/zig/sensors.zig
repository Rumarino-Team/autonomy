const std = @import("std");
const c = @import("sim_c");
const S = @import("support.zig");
const state = @import("state.zig");

fn extend(lo: *c.MathVector3f, hi: *c.MathVector3f, p: c.MathVector3f) void {
    var k: usize = 0;
    while (k < 3) : (k += 1) {
        lo.buf[k] = @min(lo.buf[k], p.buf[k]);
        hi.buf[k] = @max(hi.buf[k], p.buf[k]);
    }
}

fn objectBounds(object: c.ObjectBlock, lo: *c.MathVector3f, hi: *c.MathVector3f) void {
    @setFloatMode(.strict);
    lo.* = S.v(std.math.inf(f64), std.math.inf(f64), std.math.inf(f64));
    hi.* = S.v(-std.math.inf(f64), -std.math.inf(f64), -std.math.inf(f64));
    var i: u32 = 0;
    while (i < object.collider_count) : (i += 1) {
        const collider = object.colliders[i];
        const shape = collider.data;
        if (shape.kind == c.SIM_MESH) {
            var j: u32 = 0;
            while (j < collider.geometry.position_count) : (j += 1) extend(lo, hi, S.point(shape.pose, collider.geometry.positions[j]));
        } else {
            var size = shape.size;
            if (shape.kind == c.SIM_SPHERE) size = S.v(size.buf[0], size.buf[0], size.buf[0]);
            if (shape.kind == c.SIM_CYLINDER) size.buf[1] = size.buf[0];
            var k: u32 = 0;
            while (k < 8) : (k += 1) {
                extend(lo, hi, S.point(shape.pose, S.v(
                    if (k & 1 != 0) size.buf[0] else -size.buf[0],
                    if (k & 2 != 0) size.buf[1] else -size.buf[1],
                    if (k & 4 != 0) size.buf[2] else -size.buf[2],
                )));
            }
        }
    }
}

fn project(camera: c.CameraSettings, corners: [8]c.MathVector3f, out: *c.ObjectObservation) bool {
    @setFloatMode(.strict);
    var x0: f64 = std.math.inf(f64);
    var y0: f64 = std.math.inf(f64);
    var x1: f64 = -std.math.inf(f64);
    var y1: f64 = -std.math.inf(f64);
    const focal = @as(f64, @floatFromInt(camera.width)) * 0.5 / @tan(camera.horizontal_fov * std.math.pi / 360);
    const extendPixel = struct {
        fn apply(px0: *f64, py0: *f64, px1: *f64, py1: *f64, cam: c.CameraSettings, f: f64, p: c.MathVector3f) void {
            const x = @as(f64, @floatFromInt(cam.width)) * 0.5 + f * p.buf[0] / p.buf[2];
            const y = @as(f64, @floatFromInt(cam.height)) * 0.5 + f * p.buf[1] / p.buf[2];
            px0.* = @min(px0.*, x);
            px1.* = @max(px1.*, x);
            py0.* = @min(py0.*, y);
            py1.* = @max(py1.*, y);
        }
    }.apply;
    var a: u32 = 0;
    while (a < 8) : (a += 1) {
        var axis: u32 = 0;
        while (axis < 3) : (axis += 1) {
            const b = a ^ (@as(u32, 1) << @intCast(axis));
            if (a > b) continue;
            const p = corners[a];
            const delta = S.sub(corners[b], p);
            var first: f64 = 0;
            var last: f64 = 1;
            if (@abs(delta.buf[2]) < 1e-15) {
                if (p.buf[2] < camera.near_range or p.buf[2] > camera.far_range) continue;
            } else {
                var near = (camera.near_range - p.buf[2]) / delta.buf[2];
                var far = (camera.far_range - p.buf[2]) / delta.buf[2];
                if (near > far) std.mem.swap(f64, &near, &far);
                first = @max(first, near);
                last = @min(last, far);
                if (first > last) continue;
            }
            extendPixel(&x0, &y0, &x1, &y1, camera, focal, S.add(p, S.scale(delta, first)));
            extendPixel(&x0, &y0, &x1, &y1, camera, focal, S.add(p, S.scale(delta, last)));
        }
    }
    if (!std.math.isFinite(x0) or x1 < 0 or y1 < 0 or x0 >= @as(f64, @floatFromInt(camera.width)) or y0 >= @as(f64, @floatFromInt(camera.height))) return false;
    const width: f64 = @floatFromInt(camera.width - 1);
    const height: f64 = @floatFromInt(camera.height - 1);
    out.x0 = @intFromFloat(@floor(std.math.clamp(x0, 0, width)));
    out.y0 = @intFromFloat(@floor(std.math.clamp(y0, 0, height)));
    out.x1 = @intFromFloat(@ceil(std.math.clamp(x1, 0, width)));
    out.y1 = @intFromFloat(@ceil(std.math.clamp(y1, 0, height)));
    out.visible = 1;
    return true;
}

fn cameraSample(r: *state.Runtime, index: u32, reading: *c.SensorSample, mount: c.MathPose) void {
    @setFloatMode(.strict);
    const sensor = r.model.sensors[index];
    const objects = r.detections.ptr.? + @as(usize, index) * r.model.object_count;
    reading.value.camera = .{ .pose = mount, .objects = objects, .count = 0 };
    const inverse = S.conjugate(mount.quat);
    var i: u32 = 0;
    while (i < r.model.object_count) : (i += 1) {
        const object = r.model.object_table[i].*;
        if (object.body.tracked == 0 or @as(?*const c.ObjectBlock, @ptrCast(r.model.object_table[i])) == sensor.object) continue;
        var lo: c.MathVector3f = undefined;
        var hi: c.MathVector3f = undefined;
        objectBounds(object, &lo, &hi);
        var observation = std.mem.zeroes(c.ObjectObservation);
        observation.id = object.body.id;
        observation.cls = object.body.cls;
        observation.pose = .{ .pos = S.point(r.bodies.ptr.?[i].pose, S.scale(S.add(lo, hi), 0.5)), .quat = r.bodies.ptr.?[i].pose.quat };
        observation.size = S.sub(hi, lo);
        var corners: [8]c.MathVector3f = undefined;
        var k: u32 = 0;
        while (k < 8) : (k += 1) {
            corners[k] = S.rotate(inverse, S.sub(S.point(r.bodies.ptr.?[i].pose, S.v(
                if (k & 1 != 0) hi.buf[0] else lo.buf[0],
                if (k & 2 != 0) hi.buf[1] else lo.buf[1],
                if (k & 4 != 0) hi.buf[2] else lo.buf[2],
            )), mount.pos));
        }
        if (project(sensor.settings.camera, corners, &observation)) {
            objects[reading.value.camera.count] = observation;
            reading.value.camera.count += 1;
        }
    }
}

fn readSample(r: *const state.Runtime, sensor: ?*const c.SensorRec) ?*const c.SensorSample {
    const value = sensor orelse return null;
    const index = (@intFromPtr(value) - @intFromPtr(r.model.sensors)) / @sizeOf(c.SensorRec);
    return &r.samples.ptr.?[index];
}

fn assemble(r: *state.Runtime) void {
    @setFloatMode(.strict);
    var o = std.mem.zeroes(c.Observation);
    o.body.quat = S.q(0, 0, 0, 1);
    o.camera.quat = o.body.quat;
    o.imu = o.body.quat;
    const routes = r.model.routing;
    const tracking = readSample(r, routes.tracking);
    const imu = readSample(r, routes.imu);
    const pressure = readSample(r, routes.pressure);
    const camera = readSample(r, routes.camera);
    if (tracking != null and tracking.?.valid & c.SIM_SAMPLE_POSE != 0) {
        o.body.quat = S.multiply(tracking.?.value.pose.quat, S.conjugate(routes.tracking.*.pose.quat));
        o.attitude_axes = tracking.?.attitude_axes;
    }
    if (imu != null and imu.?.valid & c.SIM_SAMPLE_IMU != 0) {
        o.imu_ok = 1;
        o.gyro_ok = 1;
        o.imu = S.multiply(imu.?.value.imu.orientation, S.conjugate(routes.imu.*.pose.quat));
        o.gyro = S.rotate(routes.imu.*.pose.quat, imu.?.value.imu.gyro);
        o.accel = S.rotate(routes.imu.*.pose.quat, imu.?.value.imu.accel);
        if (o.attitude_axes != 7) {
            o.body.quat = o.imu;
            o.attitude_axes = 7;
        }
    }
    if (tracking != null and tracking.?.valid & c.SIM_SAMPLE_POSE != 0 and (o.attitude_axes == 7 or S.norm(routes.tracking.*.pose.pos) == 0)) {
        o.body.pos = S.sub(tracking.?.value.pose.pos, S.rotate(o.body.quat, routes.tracking.*.pose.pos));
        o.position_axes = tracking.?.position_axes;
    }
    if (pressure != null and pressure.?.valid & c.SIM_SAMPLE_DEPTH != 0) {
        o.depth = pressure.?.value.depth;
        o.depth_ok = 1;
        if (o.position_axes & 4 == 0 and (o.attitude_axes == 7 or S.norm(routes.pressure.*.pose.pos) == 0)) {
            o.body.pos.buf[2] = @floatCast(o.depth - S.rotate(o.body.quat, routes.pressure.*.pose.pos).buf[2]);
            o.position_axes |= 4;
        }
    }
    o.position_ok = if (o.position_axes == 7) 1 else 0;
    o.attitude_ok = if (o.attitude_axes == 7) 1 else 0;
    if (camera != null) {
        o.image_width = routes.camera.*.settings.camera.width;
        o.image_height = routes.camera.*.settings.camera.height;
        if (camera.?.valid & c.SIM_SAMPLE_CAMERA != 0) {
            o.camera = camera.?.value.camera.pose;
            o.objects = camera.?.value.camera.objects;
            o.object_count = camera.?.value.camera.count;
        }
    }
    r.observation = o;
}

pub fn sample(r: *state.Runtime) void {
    @setFloatMode(.strict);
    const dt = @as(f64, @floatFromInt(r.model.settings.dt_numerator)) / @as(f64, @floatFromInt(r.model.settings.dt_denominator));
    var i: u32 = 0;
    while (i < r.model.sensor_count) : (i += 1) {
        const sensor = r.model.sensors[i];
        if (r.tick != 0 and r.tick % sensor.period_ticks != 0) continue;
        var reading = std.mem.zeroes(c.SensorSample);
        reading.sensor_id = sensor.id;
        reading.kind = sensor.kind;
        reading.tick = r.tick;
        if (sensor.enabled != 0) {
            const body = r.bodies.ptr.?[sensor.object.*.id.index];
            const mount = S.compose(body.pose, sensor.pose);
            switch (sensor.kind) {
                c.SIM_SENSOR_POSE => {
                    reading.valid = c.SIM_SAMPLE_POSE;
                    reading.position_axes = sensor.settings.tracking.position_axes;
                    reading.attitude_axes = sensor.settings.tracking.attitude_axes;
                    reading.value.pose = mount;
                },
                c.SIM_SENSOR_IMU => {
                    reading.valid = c.SIM_SAMPLE_IMU;
                    const arm = S.sub(mount.pos, body.center_of_mass);
                    const alpha = S.scale(S.sub(body.angular_velocity, r.previous_angular.ptr.?[sensor.object.*.id.index]), 1 / dt);
                    const acceleration = S.add(S.add(S.scale(S.sub(body.linear_velocity, r.previous_linear.ptr.?[sensor.object.*.id.index]), 1 / dt), S.cross(alpha, arm)), S.cross(body.angular_velocity, S.cross(body.angular_velocity, arm)));
                    reading.value.imu.orientation = mount.quat;
                    reading.value.imu.gyro = S.rotate(S.conjugate(mount.quat), body.angular_velocity);
                    reading.value.imu.accel = S.rotate(S.conjugate(mount.quat), S.sub(acceleration, r.model.settings.gravity));
                },
                c.SIM_SENSOR_PRESSURE => {
                    reading.valid = c.SIM_SAMPLE_DEPTH;
                    reading.value.depth = mount.pos.buf[2];
                },
                c.SIM_SENSOR_CAMERA => {
                    reading.valid = c.SIM_SAMPLE_CAMERA;
                    cameraSample(r, i, &reading, mount);
                },
                else => {},
            }
            var j: u32 = 0;
            while (j < r.model.fault_count) : (j += 1) {
                const f = r.model.fault_table[j].*;
                if (f.sensor != &r.model.sensors[i] or r.tick < f.start or r.tick >= f.end) continue;
                if (f.kind == c.SIM_FAULT_GYRO) reading.value.imu.gyro.buf[0] += @floatCast(f.magnitude) else if (f.kind == c.SIM_FAULT_DEPTH) reading.value.depth += f.magnitude else reading.valid = 0;
            }
        }
        r.samples.ptr.?[i] = reading;
    }
    assemble(r);
}

pub fn hashSensors(r: *const state.Runtime, h: *S.Hash) void {
    for (r.samples.constItems()) |reading| {
        h.scalar(u32, reading.sensor_id);
        h.scalar(u32, reading.kind);
        h.scalar(u64, reading.tick);
        h.scalar(u32, reading.valid);
        h.scalar(u32, reading.position_axes);
        h.scalar(u32, reading.attitude_axes);
        if (reading.valid == 0) continue;
        if (reading.kind == c.SIM_SENSOR_POSE) {
            h.vec(reading.value.pose.pos);
            h.quat(reading.value.pose.quat);
        } else if (reading.kind == c.SIM_SENSOR_IMU) {
            h.quat(reading.value.imu.orientation);
            h.vec(reading.value.imu.gyro);
            h.vec(reading.value.imu.accel);
        } else if (reading.kind == c.SIM_SENSOR_PRESSURE) {
            h.scalar(f64, reading.value.depth);
        } else {
            h.vec(reading.value.camera.pose.pos);
            h.quat(reading.value.camera.pose.quat);
            h.scalar(u32, reading.value.camera.count);
            var i: u32 = 0;
            while (i < reading.value.camera.count) : (i += 1) {
                const o = reading.value.camera.objects[i];
                h.scalar(u32, o.id);
                h.scalar(u32, o.cls);
                h.vec(o.pose.pos);
                h.quat(o.pose.quat);
                h.vec(o.size);
                h.scalar(u32, o.x0);
                h.scalar(u32, o.y0);
                h.scalar(u32, o.x1);
                h.scalar(u32, o.y1);
            }
        }
    }
}
