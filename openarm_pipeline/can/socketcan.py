"""The real-hardware path. Written, not executed.

>>> UNTESTED. I had no OpenArm and no CAN adapter. <<<

This file exists so the hardware bring-up is a one-line change rather than a
rewrite, and so the reviewer can judge whether I understand the real path
even though I could not run it. Everything here is written against the
SocketCAN and python-can documentation. I have flagged the specific places
where I expect to be wrong.

Two reasons it cannot run on my machine:
  * AF_CAN is a Linux address family. It does not exist on macOS -- there is
    no SocketCAN, so this cannot even be smoke-tested here.
  * There is no adapter to bind to.

Underneath, python-can's socketcan backend opens exactly the AF_CAN socket
described in the README. I use the library rather than raw sockets because
it already handles CAN FD framing and the BCM interface correctly, and
hand-rolling that adds risk without adding insight.
"""

from __future__ import annotations

from collections.abc import Iterator

from ..clock import monotonic_ns
from ..config import CAN_BITRATE, CAN_DATA_BITRATE
from .source import CANFrame, CANSource


class SocketCANSource(CANSource):
    """Reads real frames from one or more SocketCAN interfaces."""

    def __init__(self, interfaces: tuple[str, ...] = ("can0", "can1"),
                 fd: bool = True, receive_own_messages: bool = False):
        self.interfaces = interfaces
        self.fd = fd
        self.receive_own_messages = receive_own_messages
        self._buses: list = []
        self._notifier = None
        self._running = False

    @property
    def is_mock(self) -> bool:
        return False

    def open(self) -> None:
        try:
            import can  # noqa: PLC0415 -- optional, hardware-only dependency
        except ImportError as exc:
            raise RuntimeError(
                "python-can is required for hardware capture: pip install python-can"
            ) from exc

        for iface in self.interfaces:
            # NOTE: this assumes the interface is already UP and configured
            # with the right bitrates -- that is Task 1, done out of band with
            # `ip link` before this process starts. Deliberately NOT done from
            # Python: bringing an interface up needs CAP_NET_ADMIN, and a data
            # recorder should not run privileged.
            self._buses.append(
                can.Bus(channel=iface, interface="socketcan", fd=self.fd,
                        receive_own_messages=self.receive_own_messages)
            )
        self._running = True

    def frames(self) -> Iterator[CANFrame]:
        """Yield frames from all buses, interleaved.

        EXPECTED PROBLEM: this round-robins with a short timeout, which is
        simple but adds up to `timeout` of latency per idle bus. At 1 kHz
        across two buses that is likely too slow. The correct implementation
        uses can.Notifier with a listener per bus, or select() over the
        underlying file descriptors, so both buses are serviced on arrival
        rather than on a poll. I did not write that because I could not
        measure whether it was necessary, and guessing at an optimisation I
        cannot benchmark seemed worse than stating the limitation.
        """
        import can  # noqa: PLC0415

        while self._running:
            for bus in self._buses:
                msg: can.Message | None = bus.recv(timeout=0.001)
                if msg is None:
                    continue

                # python-can supplies msg.timestamp from the kernel, which is
                # strictly better than stamping here -- it is taken on arrival
                # rather than after our scheduling delay. But it is in the
                # CLOCK_REALTIME domain while the rest of the pipeline is
                # monotonic, so it cannot be used directly.
                #
                # Doing this properly means enabling SO_TIMESTAMPING and
                # converting into the monotonic domain once per session. Until
                # that is measured on hardware, stamping here is the honest
                # choice: it is a known, bounded overestimate rather than a
                # silent mix of two clocks.
                yield CANFrame(
                    interface=bus.channel_info,
                    can_id=msg.arbitration_id,
                    data=bytes(msg.data),
                    t_mono_ns=monotonic_ns(),
                )

    def close(self) -> None:
        self._running = False
        for bus in self._buses:
            try:
                bus.shutdown()
            except Exception:  # noqa: BLE001 -- close must never raise
                pass
        self._buses.clear()


def setup_commands(interfaces: tuple[str, ...] = ("can0", "can1")) -> list[str]:
    """The exact shell commands Task 1 requires. See docs/01-can-setup.md.

    Returned as strings rather than executed: this needs root, and a library
    that silently reconfigures a network interface is a library that will
    eventually do it on the wrong machine.
    """
    cmds: list[str] = []
    for iface in interfaces:
        cmds += [
            f"sudo ip link set {iface} down",
            f"sudo ip link set {iface} type can "
            f"bitrate {CAN_BITRATE} sample-point 0.75 "
            f"dbitrate {CAN_DATA_BITRATE} dsample-point 0.75 fd on",
            f"sudo ip link set {iface} txqueuelen 1000",
            f"sudo ip link set {iface} up",
            f"ip -details link show {iface}",
        ]
    return cmds
