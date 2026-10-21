"""Persistent, explicitly recorded return positions for each arm configuration."""
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import uuid


def reference_path():
    home = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
    return home.expanduser() / 'startup_poses' / 'recorded_startup.json'


def _key(session):
    identity = {'arm': session['arm'], 'joints': session['joint_names'],
                'configuration': session['identity']}
    return hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()


def _read(path):
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return {}
    except (ValueError, UnicodeError) as exc:
        raise ValueError('起始位置记录文件无法读取，请检查记录文件') from exc
    if (not isinstance(data, dict) or data.get('schema_version') != 1
            or not isinstance(data.get('poses'), dict)):
        raise ValueError('起始位置记录文件格式无效')
    return data['poses']


def _positions(values, expected):
    if (not isinstance(values, dict) or set(values) != set(expected)
            or any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in values.values())):
        raise ValueError('起始位置必须包含所选手臂全部关节的有效角度')
    return dict(values)


def load_reference(path, sessions):
    records = _read(Path(path))
    positions, identifiers = {}, []
    for session in sessions:
        record = records.get(_key(session))
        label = '左臂' if session['arm'] == 'left' else '右臂'
        if record is None:
            raise ValueError(f'尚未记录{label}当前配置的起始位置，请先运行 record_startup')
        if not isinstance(record, dict) or not isinstance(record.get('id'), str) or not record['id']:
            raise ValueError(f'{label}起始位置记录无效，请重新记录')
        positions.update(_positions(record.get('joint_positions'), session['joint_names']))
        # The target survives a restart; an already planned return must still
        # detect either a restarted bridge or a newly recorded target.
        identifiers.append(session['id'] + ':' + record['id'])
    return {'schema_version': 1, 'id': '|'.join(identifiers), 'joint_positions': positions}


def store_reference(path, sessions, positions, captured_ros_ns):
    positions = _positions(positions, [name for session in sessions for name in session['joint_names']])
    updates = {_key(session): {
        'id': uuid.uuid4().hex, 'arm': session['arm'], 'configuration': session['identity'],
        'captured_ros_ns': int(captured_ros_ns),
        'joint_positions': {name: positions[name] for name in session['joint_names']},
    } for session in sessions}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        records = _read(path)
        records.update(updates)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                             prefix='.recorded_startup_', delete=False) as stream:
                temporary = Path(stream.name)
                json.dump({'schema_version': 1, 'poses': records}, stream,
                          ensure_ascii=False, allow_nan=False, indent=2)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
