"""Normalize portable NED traces and experimental Blender recordings.

Keep presentation samples at scene FPS and retain the exact final sample.
The source file remains the complete, replayable simulation record.
"""
import json
import math


def blender_pose(pose):
    x, y, z = pose['position']
    qx, qy, qz, qw = pose['orientation']
    values = [-x, y, -z, qw, -qx, qy, -qz]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite pose in recording')
    return values


def read_recording(path, fps=24):
    header = end = last = None
    samples, events = [], []
    names = None
    next_time = 0
    with open(path, encoding='utf-8') as source:
        for line in source:
            # A live process may still be writing its final line.
            try:
                record = json.loads(line)
            except ValueError:
                if not line.endswith('\n'):
                    break
                raise ValueError('invalid JSON line in recording')
            kind = record.get('t')
            if kind == 'header':
                header = record
                if 'scenario' in header:
                    scenario = header['scenario']
                    names = {b['id']: b.get('name', str(b['id'])) for b in scenario['bodies']
                             if b.get('motion') == 1 or b.get('tracked', False) or b.get('fuzz',False)}
                    vehicle = next(b for b in scenario['bodies'] if b['id'] == scenario['camera']['body'])
                    header = dict(header, objects=[*names.values(), 'camera'], object_ids={**{name:number for number,name in names.items()},'camera':4294967295},vehicle=vehicle.get('name',str(vehicle['id'])),
                                  seed=scenario.get('case_seed', 0))
            elif kind == 'sample':
                if header is None:
                    raise ValueError('sample before trace header')
                if names is not None:
                    camera = blender_pose(record['camera'])
                    record = {'step': record['step'], 'time': record['timestamp_ns'] * 1e-9,
                              'poses': {names[b['id']]: blender_pose(b['pose']) for b in record['bodies'] if b['id'] in names},
                              'thr': record['thr']}
                    # Camera is a separate world transform, never the body origin.
                    record['poses']['camera'] = camera
                if last is not None and record['step'] <= last['step']:
                    raise ValueError('non-increasing sample ticks')
                last = record
                if record['time'] + 1e-9 >= next_time:
                    samples.append(record)
                    next_time = record['time'] + 1 / fps
            elif kind == 'event':
                events.append(record)
            elif kind == 'end':
                end = record
    if header is None or not samples:
        raise ValueError('recording has no header or samples')
    if samples[-1] is not last:
        samples.append(last)
    if end is not None and names is not None:
        reason = {0: 'complete', 1: 'tick budget', 2: 'failed'}.get(end.get('status'), end.get('reason', 'complete'))
        end = dict(end, reason=reason, time=last['time'])
    return {'header': header, 'samples': samples, 'events': events, 'end': end}
