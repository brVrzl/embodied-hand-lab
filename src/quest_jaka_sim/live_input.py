"""Bounded Quest UDP receipt shared by simulation and physical entry points."""

from __future__ import annotations

from collections.abc import Callable
import queue
import threading

from motion_input.hts_transport import HtsUdpReceiver, ReceivedHtsDatagram


class QuestDatagramReceiverWorker:
    """Timestamp UDP arrival off the target/plant loops and retain a bounded FIFO."""

    def __init__(
        self,
        *,
        bind: str,
        port: int,
        allowed_sender: str | None,
        record: Callable[[ReceivedHtsDatagram], object],
        capacity: int = 256,
    ) -> None:
        if capacity < 1:
            raise ValueError("Quest receive queue capacity must be positive")
        self.bind = bind
        self.port = port
        self.allowed_sender = allowed_sender
        self.record = record
        self.queue: queue.Queue[ReceivedHtsDatagram] = queue.Queue(maxsize=capacity)
        self.stop_event = threading.Event()
        self.error: BaseException | None = None
        self.dropped = 0
        self.thread = threading.Thread(
            target=self._run, name="quest-hts-receive", daemon=True
        )

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def drain(self) -> list[ReceivedHtsDatagram]:
        datagrams: list[ReceivedHtsDatagram] = []
        while True:
            try:
                datagrams.append(self.queue.get_nowait())
            except queue.Empty:
                return datagrams

    def raise_if_failed(self) -> None:
        if self.error is not None:
            raise RuntimeError(f"Quest receive worker failed: {self.error}") from self.error

    def _run(self) -> None:
        try:
            with HtsUdpReceiver(
                self.bind,
                self.port,
                allowed_sender=self.allowed_sender,
            ) as receiver:
                while not self.stop_event.is_set():
                    datagram = receiver.receive(timeout_s=0.02)
                    if datagram is None:
                        continue
                    self.record(datagram)
                    try:
                        self.queue.put_nowait(datagram)
                    except queue.Full:
                        try:
                            self.queue.get_nowait()
                        except queue.Empty:
                            pass
                        self.dropped += 1
                        self.queue.put_nowait(datagram)
        except BaseException as exc:
            self.error = exc
