"""Initialize the optional vendor DDS participant once per process."""
import threading

_lock = threading.Lock()
_interface: str | None = None


def initialize(interface: str):
    global _interface
    with _lock:
        if _interface is not None:
            if interface != _interface:
                raise ValueError("DDS already initialized on a different network interface")
            return
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface)
        _interface = interface
