from __future__ import annotations

import asyncio
import json
import socket
import time

from teleoperation.input.teledex import TeleDexAdapter, TeleDexWebSocketServer


def free_port() -> int:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = int(sock.getsockname()[1])
    sock.close()
    return port


def test_websocket_transport_receives_and_cleans_up() -> None:
    port = free_port()
    adapter = TeleDexAdapter(stale_after_ns=1_000_000_000)
    server = TeleDexWebSocketServer(adapter, host="127.0.0.1", port=port)
    server.start()

    async def send() -> None:
        import websockets

        async with websockets.connect(f"ws://127.0.0.1:{port}") as websocket:
            await websocket.send(
                json.dumps(
                    {
                        "position": [0.0, 0.0, 0.0],
                        "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                        "button": False,
                    }
                )
            )
            await asyncio.sleep(0.02)

    asyncio.run(send())
    deadline = time.monotonic() + 1.0
    snapshot = None
    while snapshot is None and time.monotonic() < deadline:
        snapshot = adapter.latest(now_ns=time.monotonic_ns())
        time.sleep(0.001)
    server.stop()
    assert snapshot is not None
    assert adapter.received_packets == 1
    probe = socket.socket()
    probe.settimeout(0.1)
    assert probe.connect_ex(("127.0.0.1", port)) != 0
    probe.close()
