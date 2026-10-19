"""Cooperative, process-wide ownership for a SocketCAN arm channel."""

import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile


class ChannelOwnership:
    def __init__(self, channel, *, directory=None):
        key = hashlib.sha256(channel.encode()).hexdigest()[:20]
        self.path = Path(directory or tempfile.gettempdir()) / f"onero-can-{key}.lock"
        self.channel = channel
        self.stream = None

    def acquire(self):
        if self.stream is not None:
            return
        stream = self.path.open("a+")
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.seek(0)
            owner = stream.read().strip()
            stream.close()
            raise RuntimeError(f"CAN {self.channel} 已由另一控制进程占用：{owner}") from None
        stream.seek(0)
        stream.truncate()
        json.dump({"pid": os.getpid(), "channel": self.channel}, stream)
        stream.flush()
        self.stream = stream

    def close(self):
        if self.stream is not None:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.stream.close()
            self.stream = None

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *_):
        self.close()
