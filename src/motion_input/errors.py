"""Motion Input Platform exceptions."""


class MotionInputError(Exception):
    """Base class for motion-input failures."""


class ProtocolValidationError(MotionInputError, ValueError):
    """A UMIP value violates a protocol invariant."""


class SerializationError(MotionInputError, ValueError):
    """A serialized record cannot be decoded as UMIP."""


class ProviderStateError(MotionInputError, RuntimeError):
    """A provider operation is invalid in its current lifecycle state."""


class SourceDisconnected(MotionInputError, ConnectionError):
    """A live input source disconnected."""
