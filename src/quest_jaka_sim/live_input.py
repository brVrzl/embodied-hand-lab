"""Bounded Quest UDP receipt shared by simulation and physical entry points."""

from __future__ import annotations

from collections.abc import Callable
import queue
import threading

from motion_input.hts_transport import HtsUdpReceiver, ReceivedHtsDatagram


class QuestDatagramReceiverWorker:
    """Timestamp UDP arrival with ordered CTRL and latest-only HTS channels."""

    def __init__(
        self,
        *,
        bind: str,
        port: int,
        allowed_sender: str | None,
        record: Callable[[ReceivedHtsDatagram], object] | None = None,
        capacity: int = 256,
    ) -> None:
        if capacity < 1:
            raise ValueError("Quest receive queue capacity must be positive")
        self.bind = bind
        self.port = port
        self.allowed_sender = allowed_sender
        self.record = record
        self.queue: queue.Queue[ReceivedHtsDatagram] = queue.Queue(maxsize=capacity)
        self._latest_right_hand: ReceivedHtsDatagram | None = None
        self._latest_left_hand: ReceivedHtsDatagram | None = None
        self._latest_head: ReceivedHtsDatagram | None = None
        # Preserve one bounded delivery opportunity for malformed/unknown HTS
        # data so the existing producer-side parser can retain its faulting
        # behavior.  Valid HTS traffic always uses one of the three modality
        # slots above.
        self._latest_unclassified_hts: ReceivedHtsDatagram | None = None
        self._hand_head_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error: BaseException | None = None
        self.dropped = 0
        self.dropped_controller = 0
        self.dropped_hand_head = 0
        self.skipped_hand_head = 0
        self.right_hand_superseded = 0
        self.left_hand_superseded = 0
        self.head_superseded = 0
        self.thread = threading.Thread(
            target=self._run, name="quest-hts-receive", daemon=True
        )

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def drain(self, *, max_controller_packets: int | None = None) -> list[ReceivedHtsDatagram]:
        datagrams: list[ReceivedHtsDatagram] = []
        limit = None if max_controller_packets is None else max(1, int(max_controller_packets))
        while limit is None or len(datagrams) < limit:
            try:
                datagrams.append(self.queue.get_nowait())
            except queue.Empty:
                break
        with self._hand_head_lock:
            latest_right_hand = self._latest_right_hand
            latest_left_hand = self._latest_left_hand
            latest_head = self._latest_head
            latest_unclassified = self._latest_unclassified_hts
            self._latest_right_hand = None
            self._latest_left_hand = None
            self._latest_head = None
            self._latest_unclassified_hts = None
        for latest in (
            latest_right_hand,
            latest_left_hand,
            latest_head,
            latest_unclassified,
        ):
            if latest is not None:
                datagrams.append(latest)
        return datagrams

    def diagnostics(self) -> dict[str, int]:
        with self._hand_head_lock:
            return {
                "quest_receive_queue_dropped": self.dropped,
                "quest_controller_dropped": self.dropped_controller,
                "quest_hand_head_dropped": self.dropped_hand_head,
                "quest_hand_head_skipped": self.skipped_hand_head,
                "quest_right_hand_superseded": self.right_hand_superseded,
                "quest_left_hand_superseded": self.left_hand_superseded,
                "quest_head_superseded": self.head_superseded,
            }

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
                    if self.record is not None:
                        self.record(datagram)
                    if datagram.payload.startswith(b"CTRL,"):
                        try:
                            self.queue.put_nowait(datagram)
                        except queue.Full:
                            # Keep the CTRL FIFO ordered.  A full controller
                            # channel is observable and drops the newest
                            # packet rather than silently reordering edges.
                            self.dropped += 1
                            self.dropped_controller += 1
                    else:
                        self._store_latest_hts(datagram)
        except BaseException as exc:
            self.error = exc

    def _store_latest_hts(self, datagram: ReceivedHtsDatagram) -> None:
        modality = _classify_hts_datagram(datagram.payload)
        with self._hand_head_lock:
            if modality == "right_hand":
                attribute = "_latest_right_hand"
                counter = "right_hand_superseded"
            elif modality == "left_hand":
                attribute = "_latest_left_hand"
                counter = "left_hand_superseded"
            elif modality == "head":
                attribute = "_latest_head"
                counter = "head_superseded"
            else:
                attribute = "_latest_unclassified_hts"
                counter = None
            if getattr(self, attribute) is not None:
                self.dropped += 1
                self.dropped_hand_head += 1
                self.skipped_hand_head += 1
                if counter is not None:
                    setattr(self, counter, getattr(self, counter) + 1)
            setattr(self, attribute, datagram)


def _classify_hts_datagram(payload: bytes) -> str | None:
    """Classify HTS by its protocol labels without decoding payload values."""

    modalities: set[str] = set()
    for raw_line in payload.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        label, separator, _ = line.partition(b":")
        if not separator:
            return None
        label = label.split(b"|", 1)[0].strip()
        if label in {b"Right wrist", b"Right landmarks"}:
            modalities.add("right_hand")
        elif label in {b"Left wrist", b"Left landmarks"}:
            modalities.add("left_hand")
        elif label == b"Head pose":
            modalities.add("head")
        else:
            return None
    return next(iter(modalities)) if len(modalities) == 1 else None
