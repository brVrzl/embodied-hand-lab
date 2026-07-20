from __future__ import annotations

import asyncio
import json
import math
import socket
import threading
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from ..contracts import (
    ArmPoseSample,
    DiscontinuityKind,
    Pose3D,
    OperatorActionSample,
    RunGateSample,
    TimestampSet,
    TrackingState,
)
from ..transforms.se3 import matrix_to_quaternion_xyzw
from .interface import AdapterSnapshot


TELEDEX_SOURCE_ID = "teledex_phone"
TELEDEX_SOURCE_FRAME = "teledex_arkit_session_v0_0_7"


@dataclass(frozen=True, slots=True)
class _ParsedPacket:
    pose: Pose3D
    button_primary: bool
    button_secondary: bool
    toggle: bool
    source_timestamp_ns: int | None
    source_sequence: int | None
    tracking_state: TrackingState
    tracking_quality: float | None


class TeleDexPacketParser:
    """Parse the JSON object emitted by the TeleDex iOS WebSocket client.

    TeleDex 0.0.7's Python ``Session._process_data`` transposes the incoming
    matrix before exposing it.  This parser reproduces that adapter boundary
    explicitly.  Source axes/handedness and body-to-world semantics are not
    documented upstream and are therefore represented by a named uncalibrated
    frame; the central frame mapping owns their eventual calibration.
    """

    def __init__(
        self,
        *,
        transpose_incoming_rotation: bool = True,
        source_timestamp_field: str | None = None,
        source_sequence_field: str | None = None,
    ) -> None:
        self.transpose_incoming_rotation = bool(transpose_incoming_rotation)
        self.source_timestamp_field = source_timestamp_field
        self.source_sequence_field = source_sequence_field

    @staticmethod
    def _optional_nonnegative_int(payload: Mapping[str, Any], field: str | None) -> int | None:
        if field is None or field not in payload:
            return None
        value = payload[field]
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError(f"TeleDex {field} must be a non-negative integer")
        return value

    def parse(self, payload: Mapping[str, Any]) -> _ParsedPacket:
        if not isinstance(payload, Mapping):
            raise ValueError("TeleDex packet must be a JSON object")
        position = np.asarray(payload.get("position"), dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("TeleDex position must contain 3 finite metre values")
        rotation = np.asarray(payload.get("rotation"), dtype=np.float64)
        if rotation.shape == (9,):
            rotation = rotation.reshape(3, 3)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("TeleDex rotation must be a finite 3x3 matrix")
        if self.transpose_incoming_rotation:
            rotation = rotation.T
        quaternion = matrix_to_quaternion_xyzw(rotation)

        tracking_quality_raw = payload.get("tracking_quality")
        tracking_quality = None
        if tracking_quality_raw is not None:
            tracking_quality = float(tracking_quality_raw)
            if not math.isfinite(tracking_quality) or not 0.0 <= tracking_quality <= 1.0:
                raise ValueError("tracking_quality must be in [0, 1]")
        tracking_raw = payload.get("tracking_state")
        tracking_state = TrackingState.UNKNOWN if tracking_raw is None else TrackingState(str(tracking_raw))
        return _ParsedPacket(
            pose=Pose3D(tuple(float(value) for value in position), quaternion),
            button_primary=bool(payload.get("button", False)),
            button_secondary=bool(payload.get("button_secondary", False)),
            toggle=bool(payload.get("toggle", False)),
            source_timestamp_ns=self._optional_nonnegative_int(payload, self.source_timestamp_field),
            source_sequence=self._optional_nonnegative_int(payload, self.source_sequence_field),
            tracking_state=tracking_state,
            tracking_quality=tracking_quality,
        )


class TeleDexAdapter:
    """Bounded latest-value adapter from TeleDex packets to generic contracts."""

    def __init__(
        self,
        *,
        parser: TeleDexPacketParser | None = None,
        stale_after_ns: int = 100_000_000,
        source_frame_id: str = TELEDEX_SOURCE_FRAME,
    ) -> None:
        if stale_after_ns <= 0:
            raise ValueError("stale_after_ns must be positive")
        self.parser = parser or TeleDexPacketParser()
        self.stale_after_ns = int(stale_after_ns)
        self.source_frame_id = source_frame_id
        self._lock = threading.Lock()
        self._snapshot: AdapterSnapshot | None = None
        self._connected = False
        self._connection_epoch = 0
        self._sequence = 0
        self._generation = 0
        self._ever_connected = False
        self._pending_discontinuity = DiscontinuityKind.INITIAL
        self._stale_reported_for_sequence: int | None = None
        self._last_secondary_button = False
        self.invalid_packets = 0
        self.received_packets = 0

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def connection_epoch(self) -> int:
        with self._lock:
            return self._connection_epoch

    def on_connect(self, *, receive_ns: int | None = None) -> None:
        del receive_ns
        with self._lock:
            self._connection_epoch += 1
            self._connected = True
            self._pending_discontinuity = (
                DiscontinuityKind.RECONNECT if self._ever_connected else DiscontinuityKind.INITIAL
            )
            self._ever_connected = True

    def on_disconnect(self, *, receive_ns: int | None = None) -> None:
        now_ns = time.monotonic_ns() if receive_ns is None else int(receive_ns)
        with self._lock:
            self._connected = False
            self._generation += 1
            self._snapshot = AdapterSnapshot(
                pose=None,
                run_gate=RunGateSample(
                    TELEDEX_SOURCE_ID,
                    self._sequence,
                    now_ns,
                    engaged=False,
                    valid=False,
                    connection_epoch=self._connection_epoch,
                    reason="transport_disconnected",
                ),
                connected=False,
                generation=self._generation,
                reason="transport_disconnected",
                operator_action=OperatorActionSample(
                    TELEDEX_SOURCE_ID,
                    self._sequence,
                    now_ns,
                    valid=False,
                    reason="transport_disconnected",
                ),
            )
            self._stale_reported_for_sequence = None
            self._last_secondary_button = False

    def ingest(self, payload: Mapping[str, Any], *, receive_ns: int | None = None) -> None:
        received = time.monotonic_ns() if receive_ns is None else int(receive_ns)
        if received < 0:
            raise ValueError("receive_ns must be non-negative")
        try:
            parsed = self.parser.parse(payload)
        except (TypeError, ValueError) as exc:
            with self._lock:
                self.invalid_packets += 1
                self._sequence += 1
                self._generation += 1
                self._snapshot = AdapterSnapshot(
                    pose=None,
                    run_gate=RunGateSample(
                        TELEDEX_SOURCE_ID,
                        self._sequence,
                        received,
                        engaged=False,
                        valid=False,
                        connection_epoch=self._connection_epoch,
                        reason=f"invalid_packet:{exc}",
                    ),
                    connected=self._connected,
                    generation=self._generation,
                    reason=f"invalid_packet:{exc}",
                    operator_action=OperatorActionSample(
                        TELEDEX_SOURCE_ID,
                        self._sequence,
                        received,
                        valid=False,
                        reason="invalid_packet",
                    ),
                )
            return
        processed = time.monotonic_ns()
        if processed < received:
            processed = received
        with self._lock:
            if not self._connected:
                self._connection_epoch += 1
                self._connected = True
                self._pending_discontinuity = (
                    DiscontinuityKind.RECONNECT if self._ever_connected else DiscontinuityKind.INITIAL
                )
                self._ever_connected = True
            self.received_packets += 1
            self._sequence += 1
            self._generation += 1
            sequence = self._sequence
            discontinuity = self._pending_discontinuity
            self._pending_discontinuity = DiscontinuityKind.NONE
            tracking_valid = parsed.tracking_state != TrackingState.INVALID
            pose = ArmPoseSample(
                source_id=TELEDEX_SOURCE_ID,
                sequence=sequence,
                frame_id=self.source_frame_id,
                pose=parsed.pose,
                timestamps=TimestampSet(
                    local_receive_ns=received,
                    source_capture_ns=parsed.source_timestamp_ns,
                    processing_ns=processed,
                ),
                tracking_valid=tracking_valid,
                tracking_quality=parsed.tracking_quality,
                tracking_state=parsed.tracking_state,
                validity_reason="ok" if tracking_valid else "source_tracking_invalid",
                sample_age_ns=processed - received,
                connection_epoch=self._connection_epoch,
                discontinuity=discontinuity,
                source_sequence=parsed.source_sequence,
            )
            gate_valid = tracking_valid and discontinuity not in {
                DiscontinuityKind.RECONNECT,
                DiscontinuityKind.TRACKING_RECOVERY,
                DiscontinuityKind.RELOCALIZATION,
            }
            gate = RunGateSample(
                source_id=TELEDEX_SOURCE_ID,
                sequence=sequence,
                local_receive_ns=received,
                engaged=parsed.button_primary and gate_valid,
                valid=gate_valid,
                connection_epoch=self._connection_epoch,
                reason="button_primary" if gate_valid else f"reclutch_required:{discontinuity.value}",
            )
            secondary_rising = parsed.button_secondary and not self._last_secondary_button
            self._last_secondary_button = parsed.button_secondary
            action = OperatorActionSample(
                source_id=TELEDEX_SOURCE_ID,
                sequence=sequence,
                local_receive_ns=received,
                recenter_requested=secondary_rising and gate_valid,
                valid=gate_valid,
                reason="button_secondary_rising" if secondary_rising else "none",
            )
            self._snapshot = AdapterSnapshot(
                pose=pose,
                run_gate=gate,
                connected=True,
                generation=self._generation,
                reason="ok",
                operator_action=action,
            )
            self._stale_reported_for_sequence = None

    def latest(self, *, now_ns: int, after_generation: int = -1) -> AdapterSnapshot | None:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        with self._lock:
            snapshot = self._snapshot
            connected = self._connected
        if snapshot is None:
            return None
        if snapshot.pose is None:
            return None if snapshot.generation <= after_generation else snapshot
        age = max(0, now_ns - snapshot.pose.timestamps.local_receive_ns)
        if not connected or age >= self.stale_after_ns:
            reason = "transport_disconnected" if not connected else "sample_stale"
            with self._lock:
                if self._stale_reported_for_sequence == snapshot.pose.sequence:
                    return None
                self._stale_reported_for_sequence = snapshot.pose.sequence
                self._generation += 1
                generation = self._generation
            return AdapterSnapshot(
                pose=replace(
                    snapshot.pose,
                    tracking_valid=False,
                    tracking_state=TrackingState.INVALID,
                    validity_reason=reason,
                    sample_age_ns=age,
                ),
                run_gate=replace(snapshot.run_gate, engaged=False, valid=False, reason=reason),
                connected=connected,
                generation=generation,
                reason=reason,
                operator_action=OperatorActionSample(
                    TELEDEX_SOURCE_ID,
                    snapshot.pose.sequence,
                    now_ns,
                    valid=False,
                    reason=reason,
                ),
            )
        if snapshot.generation <= after_generation:
            return None
        return AdapterSnapshot(
            pose=replace(snapshot.pose, sample_age_ns=age),
            run_gate=snapshot.run_gate,
            connected=connected,
            generation=snapshot.generation,
            reason=snapshot.reason,
            operator_action=snapshot.operator_action,
        )

    def close(self) -> None:
        self.on_disconnect()


def assert_tcp_port_free(host: str, port: int) -> None:
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
        probe.bind((host, int(port)))
    except OSError as exc:
        raise RuntimeError(f"TeleDex port {host}:{port} is already in use") from exc
    finally:
        probe.close()


class TeleDexWebSocketServer:
    """Small audited TeleDex transport with one-client and depth-one semantics."""

    def __init__(
        self,
        adapter: TeleDexAdapter,
        *,
        host: str = "0.0.0.0",
        port: int = 8888,
        start_timeout_s: float = 3.0,
    ) -> None:
        self.adapter = adapter
        self.host = host
        self.port = int(port)
        self.start_timeout_s = float(start_timeout_s)
        self._stop = threading.Event()
        self._started = threading.Event()
        self._thread: threading.Thread | None = None
        self._startup_error: BaseException | None = None
        self._client_lock = threading.Lock()
        self._client_active = False

    async def _handler(self, websocket: Any) -> None:
        with self._client_lock:
            if self._client_active:
                await websocket.close(code=1013, reason="one TeleDex client only")
                return
            self._client_active = True
        self.adapter.on_connect(receive_ns=time.monotonic_ns())
        try:
            async for message in websocket:
                received = time.monotonic_ns()
                try:
                    payload = json.loads(message)
                except (json.JSONDecodeError, TypeError):
                    self.adapter.ingest({}, receive_ns=received)
                    continue
                self.adapter.ingest(payload, receive_ns=received)
        finally:
            self.adapter.on_disconnect(receive_ns=time.monotonic_ns())
            with self._client_lock:
                self._client_active = False

    async def _serve(self) -> None:
        try:
            import websockets

            async with websockets.serve(
                self._handler,
                self.host,
                self.port,
                max_queue=1,
                ping_interval=20.0,
                ping_timeout=10.0,
            ):
                self._started.set()
                while not self._stop.is_set():
                    await asyncio.sleep(0.02)
        except BaseException as exc:
            self._startup_error = exc
            self._started.set()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("TeleDex transport already started")
        assert_tcp_port_free(self.host, self.port)
        self._thread = threading.Thread(target=lambda: asyncio.run(self._serve()), daemon=True)
        self._thread.start()
        if not self._started.wait(self.start_timeout_s):
            self.stop()
            raise RuntimeError("TeleDex WebSocket server start timed out")
        if self._startup_error is not None:
            error = self._startup_error
            self.stop()
            raise RuntimeError("TeleDex WebSocket server failed to start") from error

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                raise RuntimeError("TeleDex WebSocket server did not stop")
            self._thread = None

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "TeleDexWebSocketServer":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()
