from __future__ import annotations

import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from ..contracts import PoseTarget
from ..wire import LatestTargetPublisher, WorkerStatusPacket, WorkerStatusReceiver, pose_target_packet


class NativeWorkerProcess:
    """Explicit subprocess lifetime; never relies on Python finalization."""

    def __init__(self, executable: str | Path, arguments: list[str]) -> None:
        self.executable = Path(executable)
        self.arguments = list(arguments)
        self.process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        if self.process is not None:
            raise RuntimeError("native worker is already started")
        self.process = subprocess.Popen([str(self.executable), *self.arguments])

    def stop(self, *, timeout_s: float = 5.0) -> int:
        if self.process is None:
            return 0
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=timeout_s)
        return int(self.process.returncode or 0)

    def __enter__(self) -> "NativeWorkerProcess":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()


@dataclass(slots=True)
class ArmOnlyRuntime:
    """Non-real-time arm supervisor composition.

    The composition intentionally contains only a target publisher and worker
    status receiver.  Device adapters and hand controllers are outside it.
    """

    publisher: LatestTargetPublisher
    status_receiver: WorkerStatusReceiver

    def dispatch(self, target: PoseTarget) -> bool:
        stamped = PoseTarget(
            source_id=target.source_id,
            sequence=target.sequence,
            target_frame_id=target.target_frame_id,
            pose=target.pose,
            timestamps=target.timestamps.with_stage(dispatch_ns=time.monotonic_ns()),
            linear_velocity_m_s=target.linear_velocity_m_s,
            angular_velocity_rad_s=target.angular_velocity_rad_s,
        )
        return self.publisher.publish(pose_target_packet(stamped, allow_motion=False))

    def latest_status(self) -> WorkerStatusPacket | None:
        return self.status_receiver.latest()

    def close(self) -> None:
        self.publisher.close()
        self.status_receiver.close()
