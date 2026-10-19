"""Per-device opening calibration above the SDK's public normalized API."""
from dataclasses import dataclass
import math
import json
import os
from pathlib import Path
import re
import tempfile
import time


@dataclass(frozen=True)
class GripperCalibration:
    closed: float = 0.
    opened: float = 1.
    command_closed: float | None = None
    command_opened: float | None = None

    def __post_init__(self):
        if (not math.isfinite(self.closed) or not math.isfinite(self.opened)
                or not 0 <= self.closed < self.opened <= 1):
            raise ValueError('夹爪标定需满足 0 <= closed_sdk_opening < open_sdk_opening <= 1')
        # Feedback at a loaded end stop is not the position command that drove
        # it there. Keep legacy/manual profiles compatible, but let automatic
        # calibration record the two coordinate ranges independently.
        if self.command_closed is None:
            object.__setattr__(self, 'command_closed', self.closed)
        if self.command_opened is None:
            object.__setattr__(self, 'command_opened', self.opened)
        if (not math.isfinite(self.command_closed) or not math.isfinite(self.command_opened)
                or not 0 <= self.command_closed < self.command_opened <= 1):
            raise ValueError('夹爪指令端点需满足 0 <= command_closed < command_opened <= 1')

    def to_sdk(self, opening):
        """Convert a command-ramp fraction to the native command coordinate."""
        if not math.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError('模型开度必须在 0..1 内')
        return self.command_closed + opening * (self.command_opened-self.command_closed)

    def target_to_sdk(self, opening):
        """Map model targets to native feedback positions, using the probed
        drive commands only at full open/close. Mechanical stop feedback does
        not imply an affine command bias across the whole stroke.
        """
        if not math.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError('模型开度必须在 0..1 内')
        if opening == 0.:
            return self.command_closed
        if opening == 1.:
            return self.command_opened
        return self.closed + opening * (self.opened-self.closed)

    def command_from_sdk(self, opening):
        """Initialize a ramp at measured native position without a command jump."""
        if not math.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError('SDK 开度必须在 0..1 内')
        return max(0., min(1., (opening-self.command_closed)
                          / (self.command_opened-self.command_closed)))

    def from_sdk(self, opening):
        if not math.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError('SDK 开度必须在 0..1 内')
        return max(0., min(1., (opening-self.closed)/(self.opened-self.closed)))


def default_calibration_path(ros_home, model, channel):
    name = re.sub(r'[^A-Za-z0-9_.-]', '_', f'{model}_{channel}_woan_picker')
    return Path(ros_home) / 'gripper_calibration' / (name + '.json')


def load_calibration(path, *, model, channel, simulate):
    path = Path(path)
    if not path.exists():
        return None
    record = json.loads(path.read_text())
    version = record.get('schema_version')
    expected = {'tool_model': 'woan_picker',
                'model': model, 'channel': channel, 'simulate': simulate}
    if version not in (1, 2) or any(record.get(k) != v for k, v in expected.items()):
        raise ValueError(f'夹爪标定文件的设备/模式不匹配：{path}')
    return GripperCalibration(record['closed_sdk_opening'], record['open_sdk_opening'],
                             record['command_closed_sdk_opening'] if version == 2 else None,
                             record['command_open_sdk_opening'] if version == 2 else None)


def save_calibration(path, calibration, *, model, channel, simulate, endpoints):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {'schema_version': 2, 'tool_model': 'woan_picker',
              'model': model, 'channel': channel, 'simulate': simulate,
              'closed_sdk_opening': calibration.closed,
              'open_sdk_opening': calibration.opened,
              'command_closed_sdk_opening': calibration.command_closed,
              'command_open_sdk_opening': calibration.command_opened,
              'measured_at_unix_s': time.time(), 'endpoint_samples': endpoints}
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=path.name+'.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(record, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
