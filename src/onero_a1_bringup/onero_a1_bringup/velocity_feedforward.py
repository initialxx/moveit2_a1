"""Validate a synchronized position/velocity command; no ROS or SDK access."""

import math


class VelocityFeedforward:
    """Access under the bridge's command lock. Never estimate velocity from position."""

    def __init__(self, joint_names, *, enabled=False, scale=None, timeout=0.1):
        self.names = tuple(joint_names)
        self.enabled = enabled
        self.scale = tuple(1.0 for _ in self.names)
        configured_scale = self.scale if scale is None else tuple(float(value) for value in scale)
        self.timeout = float(timeout)
        if configured_scale != self.scale:
            raise ValueError("速度前馈由 velocity_feedforward 统一控制全部关节；"
                             "velocity_feedforward_scale 仅兼容全 1，不再支持分关节开关或缩放")
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("velocity_feedforward_timeout 必须为有限正数")
        self.last_source_stamp_ns = None
        self.clear("no_command")

    def clear(self, reason, *, reset_order=False):
        self.raw = [0.0] * len(self.names)
        self.received_s = None
        self.source_stamp_ns = None
        self.reason = reason
        if reset_order:
            self.last_source_stamp_ns = None

    def capture(self, names, positions, velocities, *, stamp_ns, now_s, now_ros_ns):
        # Every frame replaces the previous velocity, including malformed and
        # position-only frames. They must never inherit a moving setpoint.
        self.clear("invalid_command")
        if stamp_ns > 0:
            if (self.last_source_stamp_ns is not None
                    and stamp_ns <= self.last_source_stamp_ns):
                self.reason = "out_of_order"
                return
            age = (now_ros_ns - stamp_ns) * 1e-9
            if age < -self.timeout or age > self.timeout:
                self.reason = "stale_header"
                return
            self.last_source_stamp_ns = stamp_ns
        n = len(self.names)
        if (len(names) != n or len(set(names)) != n or set(names) != set(self.names)
                or len(positions) != n or not all(math.isfinite(v) for v in positions)):
            return
        if not velocities:
            self.reason = "position_only"
            return
        if len(velocities) != n or not all(math.isfinite(v) for v in velocities):
            self.reason = "invalid_velocity"
            return
        if stamp_ns <= 0:
            self.reason = "unstamped"
            return
        by_name = dict(zip(names, velocities))
        self.raw = [float(by_name[name]) for name in self.names]
        self.received_s = now_s
        self.source_stamp_ns = stamp_ns
        self.reason = "ready"

    def sample(self, *, now_s, now_ros_ns):
        zeros = [0.0] * len(self.names)
        if self.received_s is None:
            return list(self.raw), zeros, self.reason
        receipt_age = now_s - self.received_s
        source_age = (now_ros_ns - self.source_stamp_ns) * 1e-9
        if (not 0 <= receipt_age <= self.timeout
                or not -self.timeout <= source_age <= self.timeout):
            return list(self.raw), zeros, "expired"
        if not self.enabled:
            return list(self.raw), zeros, "disabled"
        # JTC accepts endpoint velocities up to float epsilon as numerical zero.
        velocity = [v if abs(v) > 1.2e-7 else 0.0 for v in self.raw]
        return list(self.raw), velocity, "active" if any(velocity) else "zero"
