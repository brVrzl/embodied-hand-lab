from .interfaces import HandBackend, HandCommand
from .mock_backend import MockRH56Backend
from .serial_backend import RH56SerialBackend

__all__ = [
    "HandBackend",
    "HandCommand",
    "MockRH56Backend",
    "RH56Driver",
    "RH56SerialBackend",
    "RH56JakaToolBackend",
]


def __getattr__(name: str):
    if name == "RH56Driver":
        from .node import RH56Driver

        return RH56Driver
    if name == "RH56JakaToolBackend":
        from .jaka_tool_backend import RH56JakaToolBackend

        return RH56JakaToolBackend
    raise AttributeError(name)
