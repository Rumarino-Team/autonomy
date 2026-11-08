"""Explicit little-endian ASIMSC03 sections, including shared mesh tables.

Physics profile MessagePack helpers below are independent of the scene codec.
The field formats document scalar widths; native C++ struct padding is excluded.
"""
import math
import struct

MAGIC = b'ASIMSC03'
HEADER = struct.Struct('<8sQ')
MAX_BYTES = 128 * 1024 * 1024
_BODY = struct.Struct('<I64sIIIIId3d3d4d3d4d3d3ddd3dII')
_SHAPE = struct.Struct('<II3d4d3d3d3dddIIII')
_THRUSTER = struct.Struct('<II3d3d8d')
_HULL = struct.Struct('<IIII3d4d3d3d3d4dd')
_FAULT = struct.Struct('<IIQQd')
_NAME = struct.Struct('<32s')
_HEADER_PREFIX = struct.Struct('<18IQIId6d')
_CAMERA = struct.Struct('<IIIId3d4d')
_BOUNDS = struct.Struct('<3d')
assert _BODY.size == 328 and _SHAPE.size == 168 and _THRUSTER.size == 120
assert _HULL.size == 184 and _FAULT.size == 32 and _NAME.size == 32
assert _HEADER_PREFIX.size == 144 and _CAMERA.size == 80 and _BOUNDS.size == 24
assert _HEADER_PREFIX.size + _CAMERA.size + _BOUNDS.size + 32 + 64 + 64 == 408
_INTEGRATORS = {'default': 0, 'euler': 1, 'implicit': 2, 'implicitfast': 3, 'rk4': 4}
_FAULTS = ('tracking', 'imu', 'gyro', 'depth', 'dead', 'weak')


def pack(value):
    output = bytearray()

    def emit(v, depth=0):
        if depth > 64:
            raise ValueError('scene nesting limit exceeded')
        if v is None: output.append(0xc0)
        elif isinstance(v, bool): output.append(0xc3 if v else 0xc2)
        elif isinstance(v, int):
            output.extend(struct.pack('>BQ' if v >= 0 else '>Bq', 0xcf if v >= 0 else 0xd3, v))
        elif isinstance(v, float):
            if not math.isfinite(v): raise ValueError('non-finite scene value')
            output.extend(struct.pack('>Bd', 0xcb, v))
        elif isinstance(v, str):
            data = v.encode('utf-8')
            output.extend(struct.pack('>BI', 0xdb, len(data))); output.extend(data)
        elif isinstance(v, (list, tuple)):
            output.extend(struct.pack('>BI', 0xdd, len(v)))
            for item in v: emit(item, depth+1)
        elif isinstance(v, dict):
            output.extend(struct.pack('>BI', 0xdf, len(v)))
            for key in sorted(v):
                if not isinstance(key, str): raise ValueError('scene keys must be strings')
                emit(key, depth+1); emit(v[key], depth+1)
        else: raise ValueError('unsupported scene value: '+type(v).__name__)
        if len(output) > MAX_BYTES: raise ValueError('scene exceeds binary size limit')

    emit(value)
    return bytes(output)


def unpack(data):
    """Decode the JSON-compatible subset of MessagePack used by scene/profile data."""
    if not data or len(data) > MAX_BYTES: raise ValueError('invalid binary payload size')
    offset = 0

    def take(n):
        nonlocal offset
        if n < 0 or offset+n > len(data): raise ValueError('truncated binary payload')
        value = data[offset:offset+n]; offset += n
        return value

    def number(fmt): return struct.unpack('>'+fmt, take(struct.calcsize('>'+fmt)))[0]

    def read(depth=0):
        if depth > 64: raise ValueError('scene nesting limit exceeded')
        tag = take(1)[0]
        if tag <= 0x7f: return tag
        if tag >= 0xe0: return tag-256
        if tag in (0xc0, 0xc2, 0xc3): return {0xc0:None, 0xc2:False, 0xc3:True}[tag]
        formats = {0xcc:'B', 0xcd:'H', 0xce:'I', 0xcf:'Q', 0xd0:'b', 0xd1:'h', 0xd2:'i', 0xd3:'q', 0xca:'f', 0xcb:'d'}
        if tag in formats:
            value = number(formats[tag])
            if isinstance(value, float) and not math.isfinite(value): raise ValueError('non-finite binary value')
            return value
        if 0xa0 <= tag <= 0xbf or tag in (0xd9, 0xda, 0xdb):
            count = tag & 31 if tag <= 0xbf else number({0xd9:'B', 0xda:'H', 0xdb:'I'}[tag])
            return take(count).decode('utf-8')
        if 0x90 <= tag <= 0x9f or tag in (0xdc, 0xdd):
            count = tag & 15 if tag <= 0x9f else number('H' if tag == 0xdc else 'I')
            if count > len(data)-offset: raise ValueError('invalid array size')
            return [read(depth+1) for _ in range(count)]
        if 0x80 <= tag <= 0x8f or tag in (0xde, 0xdf):
            count = tag & 15 if tag <= 0x8f else number('H' if tag == 0xde else 'I')
            if count > (len(data)-offset)//2: raise ValueError('invalid map size')
            result = {}
            for _ in range(count):
                key = read(depth+1)
                if not isinstance(key, str) or key in result: raise ValueError('invalid or duplicate map key')
                result[key] = read(depth+1)
            return result
        raise ValueError('unsupported MessagePack value')

    value = read()
    if offset != len(data): raise ValueError('trailing binary payload')
    return value


def _num(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('non-finite scene value')
    return value


def _vec(value, default=(0.0, 0.0, 0.0)):
    value = default if value is None else value
    if len(value) != 3:
        raise ValueError('expected vector of length 3')
    return tuple(_num(v) for v in value)


def _quat(value, default=(0.0, 0.0, 0.0, 1.0)):
    value = default if value is None else value
    if len(value) != 4:
        raise ValueError('expected xyzw quaternion')
    return tuple(_num(v) for v in value)


def _pose(value):
    value = value or {}
    return _vec(value.get('position')), _quat(value.get('orientation'))


def _text(value, size):
    raw = str(value or '').encode()
    if len(raw) >= size:
        raise ValueError('text exceeds fixed field')
    return raw


def _text_out(raw, full=False):
    if b'\0' in raw:
        text, _, tail = raw.partition(b'\0')
        if tail.strip(b'\0'):
            raise ValueError('text field is not zero padded')
        return text.decode()
    if not full:
        raise ValueError('text field is not terminated')
    return raw.decode()


def _flag(value):
    if value in (True, 1, '1'):
        return 1
    if value in (False, 0, '0', None):
        return 0
    raise ValueError('expected a boolean flag')


def surface(kind, size, vertices=(), triangles=()):
    """Hull triangles for a shape that references primitive or mesh geometry."""
    out = []

    def triangle(a, b, c):
        out.append((a, b, c))

    if kind == 3:
        for a, b, c in triangles:
            triangle(vertices[a], vertices[b], vertices[c])
    elif kind == 0:
        x, y, z = size
        v = [(-x, -y, -z), (-x, y, -z), (x, y, -z), (x, -y, -z),
             (-x, -y, z), (-x, y, z), (x, y, z), (x, -y, z)]
        for a, b, c, d in ((0, 1, 2, 3), (3, 2, 6, 7), (4, 5, 1, 0),
                           (7, 6, 5, 4), (6, 2, 1, 5), (3, 7, 4, 0)):
            triangle(v[a], v[b], v[c])
            triangle(v[a], v[c], v[d])
        for _level in range(3):
            prev, out = out, []
            for a, b, c in prev:
                ab = tuple((p + q) * .5 for p, q in zip(a, b))
                bc = tuple((p + q) * .5 for p, q in zip(b, c))
                ca = tuple((p + q) * .5 for p, q in zip(c, a))
                triangle(a, ab, ca)
                triangle(b, bc, ab)
                triangle(c, ca, bc)
                triangle(ab, bc, ca)
    elif kind == 2:
        n, dp = 32, 2 * math.pi / 32
        radius = math.sqrt(dp * size[0] * size[0] / math.sin(dp))
        for i in range(n):
            a, b = i * dp, (i + 1) * dp
            t0 = (radius * math.cos(a), radius * math.sin(a), size[2])
            t1 = (radius * math.cos(b), radius * math.sin(b), size[2])
            b0, b1 = (t0[0], t0[1], -size[2]), (t1[0], t1[1], -size[2])
            triangle(t0, b0, t1)
            triangle(b0, b1, t1)
            triangle((0, 0, size[2]), t0, t1)
            triangle((0, 0, -size[2]), b1, b0)
    else:
        n, m = 16, 8
        def vertex(i, k):
            a, b = 2 * math.pi * i / n, math.pi * k / m
            return (size[0] * math.sin(b) * math.cos(a),
                    size[0] * math.sin(b) * math.sin(a), size[0] * math.cos(b))
        for k in range(m):
            for i in range(n):
                triangle(vertex(i, k), vertex(i, k + 1), vertex(i + 1, k + 1))
                triangle(vertex(i, k), vertex(i + 1, k + 1), vertex(i + 1, k))
    return out


def _hull_faces(entry, shapes):
    if 'faces' in entry:
        return [tuple(_vec(point) for point in face) for face in entry['faces']]
    shape = shapes[int(entry['shape'])]
    return surface(int(shape['kind']), _vec(shape.get('size'), (1, 1, 1)),
                   [_vec(v) for v in shape.get('vertices', ())],
                   [tuple(map(int, t)) for t in shape.get('triangles', ())])


def _encode_records(scene):
    """Flatten one evaluated scene dict into an ASIMSC03 snapshot."""
    if not isinstance(scene, dict) or 'bodies' not in scene:
        raise ValueError('scene snapshot requires a bodies array')
    bodies = scene['bodies']
    if not bodies or len(bodies) > 4096:
        raise ValueError('invalid bodies')
    identity = {int(body['id']): index for index, body in enumerate(bodies)}
    if len(identity) != len(bodies):
        raise ValueError('duplicate body id')
    dt = scene.get('dt', (1, 360))
    integrator = scene.get('integrator', 0)
    if isinstance(integrator, str):
        if integrator not in _INTEGRATORS:
            raise ValueError('unknown integrator')
        integrator = _INTEGRATORS[integrator]
    source = scene.get('source') or {}
    body_bytes = bytearray()
    shape_bytes = bytearray()
    vertices = bytearray()
    triangles = bytearray()
    shape_index = {}
    for index, body in enumerate(bodies):
        shapes = body.get('shapes') or []
        if not shapes:
            raise ValueError('body has no geometry')
        shape_begin = len(shape_bytes) // _SHAPE.size
        for shape in shapes:
            kind = int(shape['kind'])
            points = [_vec(v) for v in shape.get('vertices', ())]
            faces = [tuple(int(v) for v in face) for face in shape.get('triangles', ())]
            position, orientation = _pose(shape.get('pose'))
            size = _vec(shape.get('size'), (1.0, 1.0, 1.0))
            shape_bytes += _SHAPE.pack(
                kind, _flag(shape.get('collisions', True)), *position, *orientation,
                *size, *_vec(shape.get('drag')), *_vec(shape.get('skin')),
                _num(shape.get('friction', 0.5)), _num(shape.get('restitution', 0.0)),
                len(vertices) // 24, len(points), len(triangles) // 12, len(faces))
            for point in points:
                vertices += struct.pack('<3d', *point)
            for face in faces:
                triangles += struct.pack('<III', *face)
        position, orientation = _pose(body.get('pose'))
        center_position, center_orientation = _pose(body.get('center_of_mass'))
        mass = _num(body.get('mass', 0.0))
        body_bytes += _BODY.pack(
            int(body['id']), _text(body.get('name', body['id']), 64),
            int(body.get('class', 0)), int(body.get('motion', 0)),
            _flag(body.get('tracked')), _flag(body.get('fuzz')), 0, mass,
            *_vec(body.get('inertia'), (1.0, 1.0, 1.0)),
            *center_position, *center_orientation, *position, *orientation,
            *_vec(body.get('linear_velocity')), *_vec(body.get('angular_velocity')),
            _num(body.get('dry_mass', mass)), _num(body.get('volume', 0.0)),
            *_vec(body.get('buoyancy_center')), shape_begin, len(shapes))
        shape_index[index] = shapes
    thruster_bytes = bytearray()
    for thruster in scene.get('thrusters', ()):
        spec = thruster.get('spec') or {}
        thruster_bytes += _THRUSTER.pack(
            identity[int(thruster['body'])], 0, *_vec(thruster['position']),
            *_vec(thruster['direction']), _num(spec.get('max_rpm', 1000)),
            _num(spec.get('diameter', .18)),
            _num(spec.get('thrust_coeff', spec.get('thrust', .48))),
            _num(spec.get('torque_coeff', spec.get('torque', .05))),
            _num(spec.get('rotor_inertia', spec.get('inertia', .00146468))),
            _num(spec.get('kp', 1)), _num(spec.get('ki', 10)),
            _num(spec.get('command_sign', -1)))
    hull_bytes = bytearray()
    hull_vertices = bytearray()
    for hull in scene.get('hull', ()):
        owner = identity[int(hull['body'])]
        position, orientation = _pose(hull.get('pose'))
        faces = _hull_faces(hull, shape_index[owner])
        if not faces:
            raise ValueError('invalid hull')
        begin = len(hull_vertices) // 24
        for face in faces:
            for point in face:
                hull_vertices += struct.pack('<3d', *point)
        frame = _quat(hull.get('coefficient_orientation'), orientation)
        hull_bytes += _HULL.pack(
            owner, begin, len(faces) * 3, 0, *position, *orientation,
            *_vec(hull['drag']), *_vec(hull['skin']), *_vec(hull.get('centroid')),
            *frame, _num(hull.get('volume', 0.0)))
    fault_bytes = bytearray()
    for fault in scene.get('faults', ()):
        kind = fault['kind']
        if kind not in _FAULTS:
            raise ValueError('unknown fault kind')
        fault_bytes += _FAULT.pack(
            _FAULTS.index(kind), int(fault.get('thruster', 0)),
            int(fault['start_tick']), int(fault['end_tick']),
            _num(fault.get('magnitude', 0.0)))
    modules = scene.get('force_modules', ('gravity_compensation', 'hydrodynamics', 'thrusters'))
    module_bytes = b''.join(_NAME.pack(_text(name, 32)) for name in modules)
    commands = scene.get('command_sequence') or []
    width = len(commands[0]) if commands else 0
    command_bytes = bytearray()
    for row in commands:
        if len(row) != width:
            raise ValueError('command sequence width changed')
        command_bytes += struct.pack('<' + 'd' * width, *(_num(v) for v in row))
    camera = scene.get('camera') or {}
    position, orientation = _pose(camera.get('pose'))
    bounds = scene.get('assertions') or {}
    gravity = _vec(scene.get('gravity'), (0.0, 0.0, 9.81))
    current = _vec(scene.get('current'))
    camera_body = identity[int(camera.get('body', bodies[0]['id']))]
    header = _HEADER_PREFIX.pack(
        1, int(dt[0]), int(dt[1]), int(integrator),
        _flag(scene.get('native_hydrodynamics')), int(scene.get('hydro_period_ticks', 7)),
        len(bodies), len(shape_bytes) // _SHAPE.size, len(vertices) // 24,
        len(triangles) // 12, len(thruster_bytes) // _THRUSTER.size,
        len(hull_bytes) // _HULL.size, len(hull_vertices) // 24,
        len(fault_bytes) // _FAULT.size, len(modules), len(commands), width,
        int(source.get('frame') or 0), int(scene.get('case_seed', 0)),
        int(scene.get('generator_version', 0)), 0, _num(scene.get('water_density', 1025)),
        *gravity, *current) + _CAMERA.pack(
            camera_body, int(camera.get('width', 800)), int(camera.get('height', 600)), 0,
            _num(camera.get('horizontal_fov', 60)), *position, *orientation) + _BOUNDS.pack(
                _num(bounds.get('max_position_norm', 10000)),
                _num(bounds.get('max_speed', 1000)),
                _num(bounds.get('max_angular_speed', 1000))) + struct.pack(
                    '<32s64s64s', _text(source.get('kind'), 32),
                    _text(source.get('content_hash'), 65)[:64],
                    _text(source.get('scene'), 64))
    payload = b''.join((header, body_bytes, shape_bytes, vertices, triangles,
                        thruster_bytes, hull_bytes, hull_vertices, fault_bytes,
                        module_bytes, command_bytes))
    if not payload or len(payload) > MAX_BYTES:
        raise ValueError('invalid binary scene size')
    return HEADER.pack(MAGIC, len(payload)) + payload


class _Cursor:
    def __init__(self, data, at=0):
        self.data, self.at = data, at

    def take(self, size):
        if size < 0 or self.at + size > len(self.data):
            raise ValueError('truncated scene snapshot')
        out = self.data[self.at:self.at + size]
        self.at += size
        return out

    def items(self, kind, count):
        return [kind.unpack(self.take(kind.size)) for _ in range(count)]


def _pose_out(raw):
    return {'position': list(raw[:3]), 'orientation': list(raw[3:7])}


def _decode_records(data):
    """Read an ASIMSC03 snapshot into the scene dict used by the tools."""
    if len(data) < HEADER.size:
        raise ValueError('truncated binary scene header')
    magic, size = HEADER.unpack_from(data)
    if magic != MAGIC or not 0 < size <= MAX_BYTES or len(data) != HEADER.size + size:
        raise ValueError('invalid binary scene header/size')
    cursor = _Cursor(data, HEADER.size)
    prefix = cursor.items(_HEADER_PREFIX, 1)[0]
    (version, dt_n, dt_d, integrator, native, hydro, body_count, shape_count,
     vertex_count, triangle_count, thruster_count, hull_count, hull_vertex_count,
     fault_count, module_count, command_ticks, command_width, frame, case_seed,
     generator, reserved, water, *vectors) = prefix
    if version != 1 or reserved:
        raise ValueError('unsupported scene snapshot')
    camera = cursor.items(_CAMERA, 1)[0]
    bounds = cursor.items(_BOUNDS, 1)[0]
    kind, digest, scene_name = struct.unpack('<32s64s64s', cursor.take(160))
    bodies = cursor.items(_BODY, body_count)
    shapes = cursor.items(_SHAPE, shape_count)
    vertices = [cursor.items(struct.Struct('<3d'), 1)[0] for _ in range(vertex_count)]
    triangles = cursor.items(struct.Struct('<III'), triangle_count)
    thrusters = cursor.items(_THRUSTER, thruster_count)
    hulls = cursor.items(_HULL, hull_count)
    hull_vertices = [cursor.items(struct.Struct('<3d'), 1)[0] for _ in range(hull_vertex_count)]
    faults = cursor.items(_FAULT, fault_count)
    modules = cursor.items(_NAME, module_count)
    commands = [struct.unpack('<' + 'd' * command_width, cursor.take(8 * command_width))
                for _ in range(command_ticks)] if command_width else []
    if cursor.at != len(data):
        raise ValueError('scene snapshot has trailing bytes inside its payload')
    def shape_dict(raw):
        (kind_n, collisions, px, py, pz, qx, qy, qz, qw, sx, sy, sz, dx, dy, dz,
         kx, ky, kz, friction, restitution, v0, vn, t0, tn) = raw
        item = {'kind': kind_n, 'collisions': bool(collisions),
                'pose': _pose_out((px, py, pz, qx, qy, qz, qw)),
                'size': [sx, sy, sz], 'drag': [dx, dy, dz], 'skin': [kx, ky, kz],
                'friction': friction, 'restitution': restitution}
        if vn or tn:
            item['vertices'] = [list(vertices[v0 + i]) for i in range(vn)]
            item['triangles'] = [list(triangles[t0 + i]) for i in range(tn)]
        return item
    body_dicts = []
    for raw in bodies:
        (identifier, name, cls, motion, tracked, fuzz, pad, mass, ix, iy, iz,
         cx, cy, cz, cqx, cqy, cqz, cqw, px, py, pz, qx, qy, qz, qw, lx, ly, lz,
         ax, ay, az, dry, volume, bx, by, bz, begin, count) = raw
        if pad:
            raise ValueError('body padding is not zero')
        body_dicts.append({
            'id': identifier, 'name': _text_out(name), 'class': cls, 'motion': motion,
            'tracked': bool(tracked), 'fuzz': bool(fuzz), 'mass': mass,
            'inertia': [ix, iy, iz],
            'center_of_mass': _pose_out((cx, cy, cz, cqx, cqy, cqz, cqw)),
            'pose': _pose_out((px, py, pz, qx, qy, qz, qw)),
            'linear_velocity': [lx, ly, lz], 'angular_velocity': [ax, ay, az],
            'dry_mass': dry, 'volume': volume, 'buoyancy_center': [bx, by, bz],
            'shapes': [shape_dict(shapes[begin + k]) for k in range(count)]})
    def owner(index):
        return body_dicts[index]['id']
    document = {
        'version': 1, 'dt': [dt_n, dt_d], 'integrator': integrator,
        'gravity': list(vectors[:3]), 'water_density': water, 'current': list(vectors[3:]),
        'hydro_period_ticks': hydro, 'native_hydrodynamics': bool(native),
        'bodies': body_dicts,
        'thrusters': [{
            'body': owner(raw[0]), 'position': list(raw[2:5]), 'direction': list(raw[5:8]),
            'spec': {'max_rpm': raw[8], 'diameter': raw[9], 'thrust_coeff': raw[10],
                     'torque_coeff': raw[11], 'rotor_inertia': raw[12], 'kp': raw[13],
                     'ki': raw[14], 'command_sign': raw[15]}} for raw in thrusters],
        'hull': [],
        'camera': {'body': owner(camera[0]), 'width': camera[1], 'height': camera[2],
                   'horizontal_fov': camera[4], 'pose': _pose_out(camera[5:])},
        'assertions': {'max_position_norm': bounds[0], 'max_speed': bounds[1],
                       'max_angular_speed': bounds[2]},
        'faults': [{'kind': _FAULTS[raw[0]], 'thruster': raw[1], 'start_tick': raw[2],
                    'end_tick': raw[3], 'magnitude': raw[4]} for raw in faults],
        'force_modules': [_text_out(raw[0]) for raw in modules],
        'source': {'kind': _text_out(kind), 'content_hash': _text_out(digest, full=True),
                   'scene': _text_out(scene_name), 'frame': frame},
        'case_seed': case_seed, 'generator_version': generator}
    for raw in hulls:
        (owner_index, begin, count, pad, px, py, pz, qx, qy, qz, qw, dx, dy, dz,
         sx, sy, sz, cx, cy, cz, fx, fy, fz, fw, volume) = raw
        if pad or count % 3:
            raise ValueError('invalid hull record')
        faces = []
        for triangle in range(0, count, 3):
            faces.append([list(hull_vertices[begin + triangle + k]) for k in range(3)])
        document['hull'].append({
            'body': owner(owner_index), 'pose': _pose_out((px, py, pz, qx, qy, qz, qw)),
            'drag': [dx, dy, dz], 'skin': [sx, sy, sz], 'centroid': [cx, cy, cz],
            'coefficient_orientation': [fx, fy, fz, fw], 'volume': volume, 'faces': faces})
    if commands:
        document['command_sequence'] = [list(row) for row in commands]
    document['vehicle'] = next(body['name'] for body in body_dicts
                               if body['id'] == document['camera']['body'])
    return document


# ASIMSC03 section directory: u32 kind/count, u64 offset/byte-length.
# The private record helpers encode named fields; these are not native-memory
# structure dumps. Shared meshes are represented once, with explicit references.
_SECTION = struct.Struct('<IIQQ')
_MESH = struct.Struct('<IIII')
_WIDTHS = (408, 328, 168, 24, 12, 120, 184, 24, 32, 32, 8, 16, 4)


def encode(scene):
    raw = _encode_records(scene)[HEADER.size:]
    header = list(_HEADER_PREFIX.unpack(raw[:_HEADER_PREFIX.size]))
    counts = (1, header[6], header[7], header[8], header[9], header[10],
              header[11], header[12], header[13], header[14], header[15] * header[16])
    sections = []
    at = 0
    for count, width in zip(counts, _WIDTHS):
        end = at + count * width
        sections.append(raw[at:end])
        at = end
    if at != len(raw):
        raise ValueError('scene field schema mismatch')
    vertices, triangles, meshes, refs = bytearray(), bytearray(), bytearray(), bytearray()
    shapes, cache = bytearray(), {}
    for offset in range(0, len(sections[2]), _SHAPE.size):
        shape = list(_SHAPE.unpack_from(sections[2], offset))
        if shape[0] == 3:
            vb, vc, tb, tc = shape[-4:]
            points = sections[3][vb*24:(vb+vc)*24]
            faces = sections[4][tb*12:(tb+tc)*12]
            key = (points, faces)
            if key not in cache:
                index = len(meshes) // _MESH.size
                ranges = (len(vertices)//24, vc, len(triangles)//12, tc)
                cache[key] = (index, ranges)
                meshes += _MESH.pack(*ranges)
                vertices += points
                triangles += faces
            index, ranges = cache[key]
            shape[-4:] = ranges
        else:
            index = 0xffffffff
            shape[-4:] = (0, 0, 0, 0)
        shapes += _SHAPE.pack(*shape)
        refs += struct.pack('<I', index)
    # Empty primitive ranges use the final geometry cursors in both languages.
    for offset in range(0, len(shapes), _SHAPE.size):
        shape = list(_SHAPE.unpack_from(shapes, offset))
        if shape[0] != 3:
            shape[-4:] = (len(vertices)//24, 0, len(triangles)//12, 0)
            _SHAPE.pack_into(shapes, offset, *shape)
    header[8], header[9] = len(vertices)//24, len(triangles)//12
    sections[0] = _HEADER_PREFIX.pack(*header) + sections[0][_HEADER_PREFIX.size:]
    sections[2:5] = [bytes(shapes), bytes(vertices), bytes(triangles)]
    sections.extend((bytes(meshes), bytes(refs)))
    offset = 8 + len(sections) * _SECTION.size
    directory = bytearray(struct.pack('<II', len(sections), 1))
    for index, (section, width) in enumerate(zip(sections, _WIDTHS), 1):
        directory += _SECTION.pack(index, len(section)//width, offset, len(section))
        offset += len(section)
    payload = bytes(directory) + b''.join(sections)
    if not payload or len(payload) > MAX_BYTES:
        raise ValueError('invalid binary scene size')
    return HEADER.pack(MAGIC, len(payload)) + payload


def decode(data):
    if len(data) < HEADER.size:
        raise ValueError('truncated binary scene header')
    magic, length = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError('unsupported scene format; ASIMSC03 is required (regenerate old recordings)')
    if not length or length > MAX_BYTES or length != len(data) - HEADER.size:
        raise ValueError('invalid binary scene size')
    payload = memoryview(data)[HEADER.size:]
    if len(payload) < 8 or struct.unpack_from('<II', payload) != (13, 1):
        raise ValueError('unsupported scene directory')
    sections, counts, end = [], [], 8 + 13*_SECTION.size
    if len(payload) < end:
        raise ValueError('truncated scene directory')
    for i, width in enumerate(_WIDTHS):
        kind, count, offset, size = _SECTION.unpack_from(payload, 8+i*_SECTION.size)
        if kind != i+1 or offset != end or size != count*width or offset+size > length:
            raise ValueError('invalid scene section directory')
        sections.append(bytes(payload[offset:offset+size]))
        counts.append(count)
        end += size
    if end != length or counts[0] != 1 or not counts[1] or counts[1] > 4096:
        raise ValueError('invalid scene sections')
    h = _HEADER_PREFIX.unpack_from(sections[0])
    expected = [1, h[6], h[7], h[8], h[9], h[10], h[11], h[12], h[13], h[14], h[15]*h[16]]
    if counts[:11] != expected or counts[12] != counts[2]:
        raise ValueError('scene header disagrees with section counts')
    meshes = [_MESH.unpack_from(sections[11], i*_MESH.size) for i in range(counts[11])]
    for vb, vc, tb, tc in meshes:
        if vb+vc > counts[3] or tb+tc > counts[4]:
            raise ValueError('invalid mesh section range')
    for i in range(counts[2]):
        shape = _SHAPE.unpack_from(sections[2], i*_SHAPE.size)
        mesh, = struct.unpack_from('<I', sections[12], i*4)
        if shape[0] == 3:
            if mesh >= len(meshes) or shape[-4:] != meshes[mesh]:
                raise ValueError('invalid mesh reference')
        elif mesh != 0xffffffff:
            raise ValueError('primitive references a mesh')
    records = b''.join(sections[:11])
    return _decode_records(HEADER.pack(MAGIC, len(records)) + records)
