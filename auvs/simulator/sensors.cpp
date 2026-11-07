#include "runtime_internal.hpp"
#include <numbers>

namespace {
using namespace core;

/* Model-space bounds include every contained collider, including noncolliding
 * detection proxies. Bounds are transformed with the object's world pose. */
void object_bounds(const ObjectBlock &object, MathVector3f &lo, MathVector3f &hi) {
    lo = v(INFINITY, INFINITY, INFINITY);
    hi = v(-INFINITY, -INFINITY, -INFINITY);
    auto extend = [&](MathVector3f p) {
        for (unsigned k = 0; k < 3; ++k) {
            lo.buf[k] = std::min(lo.buf[k], p.buf[k]);
            hi.buf[k] = std::max(hi.buf[k], p.buf[k]);
        }
    };
    for (const auto &collider : std::span(object.colliders, object.collider_count)) {
        const auto &s = collider.data;
        if (s.kind == SIM_MESH) {
            for (uint32_t j = 0; j < collider.geometry.position_count; ++j)
                extend(point(s.pose, collider.geometry.positions[j]));
        } else {
            auto size = s.size;
            if (s.kind == SIM_SPHERE)
                size = v(size.buf[0], size.buf[0], size.buf[0]);
            if (s.kind == SIM_CYLINDER)
                size.buf[1] = size.buf[0];
            for (unsigned k = 0; k < 8; ++k)
                extend(point(
                    s.pose,
                    v(k & 1 ? size.buf[0] : -size.buf[0],
                      k & 2 ? size.buf[1] : -size.buf[1],
                      k & 4 ? size.buf[2] : -size.buf[2])
                ));
        }
    }
}

bool project(const CameraSettings &c, const MathVector3f (&corners)[8], ObjectObservation &out) {
    double x0 = INFINITY, y0 = INFINITY, x1 = -INFINITY, y1 = -INFINITY;
    const double focal = c.width * .5 / std::tan(c.horizontal_fov * std::numbers::pi / 360);
    auto extend = [&](MathVector3f p) {
        const double x = c.width * .5 + focal * p.buf[0] / p.buf[2];
        const double y = c.height * .5 + focal * p.buf[1] / p.buf[2];
        x0 = std::min(x0, x);
        x1 = std::max(x1, x);
        y0 = std::min(y0, y);
        y1 = std::max(y1, y);
    };
    // Clip all twelve box edges against the near/far depth planes. Projecting
    // only original corners drops boxes that cross the camera's near plane.
    for (unsigned a = 0; a < 8; ++a)
        for (unsigned axis = 0; axis < 3; ++axis) {
            const unsigned b = a ^ (1u << axis);
            if (a > b)
                continue;
            auto p = corners[a], delta = corners[b] - p;
            double first = 0, last = 1;
            if (std::abs(delta.buf[2]) < 1e-15) {
                if (p.buf[2] < c.near_range || p.buf[2] > c.far_range)
                    continue;
            } else {
                double near = (c.near_range - p.buf[2]) / delta.buf[2];
                double far = (c.far_range - p.buf[2]) / delta.buf[2];
                if (near > far)
                    std::swap(near, far);
                first = std::max(first, near);
                last = std::min(last, far);
                if (first > last)
                    continue;
            }
            extend(p + delta * first);
            extend(p + delta * last);
        }
    if (!std::isfinite(x0) || x1 < 0 || y1 < 0 || x0 >= c.width || y0 >= c.height)
        return false;
    out.x0 = uint32_t(std::floor(std::clamp(x0, 0., double(c.width - 1))));
    out.y0 = uint32_t(std::floor(std::clamp(y0, 0., double(c.height - 1))));
    out.x1 = uint32_t(std::ceil(std::clamp(x1, 0., double(c.width - 1))));
    out.y1 = uint32_t(std::ceil(std::clamp(y1, 0., double(c.height - 1))));
    out.visible = 1;
    return true;
}

void camera_sample(SimulationRuntime &r, uint32_t index, SensorSample &sample, MathPose mount) {
    const auto &sensor = r.model->sensors[index];
    auto *objects = r.detections.data() + size_t(index) * r.model->object_count;
    sample.value.camera = {mount, objects, 0};
    const auto inverse = conjugate(mount.quat);
    for (uint32_t i = 0; i < r.model->object_count; ++i) {
        const auto &object = *r.model->object_table[i];
        if (!object.body.tracked || &object == sensor.object)
            continue;
        MathVector3f lo, hi;
        object_bounds(object, lo, hi);
        ObjectObservation observation{};
        observation.id = object.body.id;
        observation.cls = object.body.cls;
        observation.pose = {point(r.bodies[i].pose, (lo + hi) * .5), r.bodies[i].pose.quat};
        observation.size = hi - lo;
        MathVector3f corners[8];
        for (unsigned k = 0; k < 8; ++k)
            corners[k] = rotate(
                inverse,
                point(
                    r.bodies[i].pose,
                    v(k & 1 ? hi.buf[0] : lo.buf[0],
                      k & 2 ? hi.buf[1] : lo.buf[1],
                      k & 4 ? hi.buf[2] : lo.buf[2])
                ) - mount.pos
            );
        if (project(sensor.settings.camera, corners, observation))
            objects[sample.value.camera.count++] = observation;
    }
}

void assemble(SimulationRuntime &r) {
    Observation o{};
    o.body.quat = o.camera.quat = o.imu = q();
    const auto &routes = r.model->routing;
    auto read = [&](const SensorRec *sensor) -> const SensorSample * {
        if (!sensor)
            return nullptr;
        return &r.samples[size_t(sensor - r.model->sensors)];
    };
    auto *tracking = read(routes.tracking), *imu = read(routes.imu);
    auto *pressure = read(routes.pressure), *camera = read(routes.camera);
    if (tracking && (tracking->valid & SIM_SAMPLE_POSE)) {
        o.body.quat = multiply(tracking->value.pose.quat, conjugate(routes.tracking->pose.quat));
        o.attitude_axes = tracking->attitude_axes;
    }
    if (imu && (imu->valid & SIM_SAMPLE_IMU)) {
        o.imu_ok = o.gyro_ok = 1;
        o.imu = multiply(imu->value.imu.orientation, conjugate(routes.imu->pose.quat));
        o.gyro = rotate(routes.imu->pose.quat, imu->value.imu.gyro);
        o.accel = rotate(routes.imu->pose.quat, imu->value.imu.accel);
        if (o.attitude_axes != 7) {
            o.body.quat = o.imu;
            o.attitude_axes = 7;
        }
    }
    if (tracking && (tracking->valid & SIM_SAMPLE_POSE) &&
        (o.attitude_axes == 7 || norm(routes.tracking->pose.pos) == 0)) {
        o.body.pos = tracking->value.pose.pos - rotate(o.body.quat, routes.tracking->pose.pos);
        o.position_axes = tracking->position_axes;
    }
    if (pressure && (pressure->valid & SIM_SAMPLE_DEPTH)) {
        o.depth = pressure->value.depth;
        o.depth_ok = 1;
        if (!(o.position_axes & 4) &&
            (o.attitude_axes == 7 || norm(routes.pressure->pose.pos) == 0)) {
            o.body.pos.buf[2] =
                float(o.depth - rotate(o.body.quat, routes.pressure->pose.pos).buf[2]);
            o.position_axes |= 4;
        }
    }
    o.position_ok = o.position_axes == 7;
    o.attitude_ok = o.attitude_axes == 7;
    if (camera) {
        o.image_width = routes.camera->settings.camera.width;
        o.image_height = routes.camera->settings.camera.height;
        if (camera->valid & SIM_SAMPLE_CAMERA) {
            o.camera = camera->value.camera.pose;
            o.objects = camera->value.camera.objects;
            o.object_count = camera->value.camera.count;
        }
    }
    r.observation = o;
}
} // namespace

namespace core {
void sample_sensors(SimulationRuntime &r) {
    const double dt = double(r.model->settings.dt_numerator) / r.model->settings.dt_denominator;
    for (uint32_t i = 0; i < r.model->sensor_count; ++i) {
        const auto &sensor = r.model->sensors[i];
        if (r.tick && r.tick % sensor.period_ticks)
            continue;
        SensorSample sample{};
        sample.sensor_id = sensor.id;
        sample.kind = sensor.kind;
        sample.tick = r.tick;
        if (sensor.enabled) {
            const auto &body = r.bodies[sensor.object->id.index];
            const auto mount = compose(body.pose, sensor.pose);
            switch (sensor.kind) {
            case SIM_SENSOR_POSE:
                sample.valid = SIM_SAMPLE_POSE;
                sample.position_axes = sensor.settings.tracking.position_axes;
                sample.attitude_axes = sensor.settings.tracking.attitude_axes;
                sample.value.pose = mount;
                break;
            case SIM_SENSOR_IMU: {
                sample.valid = SIM_SAMPLE_IMU;
                auto arm = mount.pos - body.center_of_mass;
                auto alpha =
                    (body.angular_velocity - r.previous_angular_velocity[sensor.object->id.index]) /
                    dt;
                auto acceleration =
                    (body.linear_velocity - r.previous_velocity[sensor.object->id.index]) / dt +
                    cross(alpha, arm) +
                    cross(body.angular_velocity, cross(body.angular_velocity, arm));
                sample.value.imu.orientation = mount.quat;
                sample.value.imu.gyro = rotate(conjugate(mount.quat), body.angular_velocity);
                sample.value.imu.accel =
                    rotate(conjugate(mount.quat), acceleration - r.model->settings.gravity);
                break;
            }
            case SIM_SENSOR_PRESSURE:
                sample.valid = SIM_SAMPLE_DEPTH;
                sample.value.depth = mount.pos.buf[2];
                break;
            case SIM_SENSOR_CAMERA:
                sample.valid = SIM_SAMPLE_CAMERA;
                camera_sample(r, i, sample, mount);
                break;
            }
            for (uint32_t j = 0; j < r.model->fault_count; ++j) {
                const auto &f = *r.model->fault_table[j];
                if (f.sensor != &sensor || r.tick < f.start || r.tick >= f.end)
                    continue;
                if (f.kind == SIM_FAULT_GYRO)
                    sample.value.imu.gyro.buf[0] += float(f.magnitude);
                else if (f.kind == SIM_FAULT_DEPTH)
                    sample.value.depth += f.magnitude;
                else
                    sample.valid = 0;
            }
        }
        r.samples[i] = sample;
    }
    assemble(r);
}

void hash_sensors(const SimulationRuntime &r, Hash &h) {
    for (const auto &s : r.samples) {
        h.scalar(s.sensor_id);
        h.scalar(s.kind);
        h.scalar(s.tick);
        h.scalar(s.valid);
        h.scalar(s.position_axes);
        h.scalar(s.attitude_axes);
        if (!s.valid)
            continue;
        if (s.kind == SIM_SENSOR_POSE) {
            h.vec(s.value.pose.pos);
            h.quat(s.value.pose.quat);
        } else if (s.kind == SIM_SENSOR_IMU) {
            h.quat(s.value.imu.orientation);
            h.vec(s.value.imu.gyro);
            h.vec(s.value.imu.accel);
        } else if (s.kind == SIM_SENSOR_PRESSURE)
            h.scalar(s.value.depth);
        else {
            h.vec(s.value.camera.pose.pos);
            h.quat(s.value.camera.pose.quat);
            h.scalar(s.value.camera.count);
            for (uint32_t i = 0; i < s.value.camera.count; ++i) {
                auto &o = s.value.camera.objects[i];
                h.scalar(o.id);
                h.scalar(o.cls);
                h.vec(o.pose.pos);
                h.quat(o.pose.quat);
                h.vec(o.size);
                h.scalar(o.x0);
                h.scalar(o.y0);
                h.scalar(o.x1);
                h.scalar(o.y1);
            }
        }
    }
}
} // namespace core
